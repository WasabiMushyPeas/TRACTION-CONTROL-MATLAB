// traction_controller.cpp
//
// Implementation of traction_controller.hpp. Each section of step() names the
// TC_organized.slx block it reproduces.

#include "traction_controller.hpp"

#include <math.h>  // expf, sinf, atanf (init only)

namespace tc {
namespace {

constexpr float kPi = 3.14159265358979f;

inline float minf(float a, float b) { return a < b ? a : b; }
inline float maxf(float a, float b) { return a > b ? a : b; }
inline float clampf(float x, float lo, float hi) { return minf(maxf(x, lo), hi); }

inline bool isFront(int wheel) { return wheel == FL || wheel == FR; }

// First-order low-pass y = alpha*u + (1 - alpha)*y_prev. Same response as the
// model's Discrete Filter blocks (numerator alpha, denominator [1, -(1-alpha)]).
inline float lowPass(float alpha, float input, float previousOutput) {
    return alpha * input + (1.0f - alpha) * previousOutput;
}

}  // namespace

ControllerParams defaultParams() {
    constexpr float kLbToKg = 0.45359237f;
    constexpr float kInToM = 0.0254f;

    ControllerParams p = {};

    VehicleParams& v = p.vehicle;
    v.mass = 473.92f * kLbToKg + 150.0f * kLbToKg;  // Car + 150 lbm driver = 283.0 kg
    v.wheelbase = 60.25f * kInToM;
    v.cgHeight = 10.4f * kInToM;
    v.frontWeightFraction = 0.50f;
    v.gravity = 9.80665f;
    v.tireRadius = 0.5f * 16.0f * kInToM;
    v.gearRatio = 12.5f;
    v.drivetrainEfficiency = 0.92f;
    v.motorInverterEfficiency = 1.00f;
    v.wheelInertia = 0.35f;
    v.airDensity = 1.225f;
    v.dragArea = 1.20f;
    v.liftArea = 4.40f;
    v.aeroFrontFraction = 0.50f;
    v.maxMotorTorque = 21.5f;
    v.maxMotorRpm = 20000.0f;
    v.speedTaperRpm = 500.0f;
    v.maxTractivePower = 80000.0f;
    v.pacejkaB = 10.4f;
    v.pacejkaC = -1.58f;
    v.pacejkaD1 = -3.02f;
    v.pacejkaD2 = 0.8f;
    v.slipSpeedFloor = 0.5f;

    p.sampleTime = 0.002f;
    p.slipFilterHz = 25.0f;
    p.feedforwardFilterHz = 20.0f;
    p.pidFilterN = 100.0f;
    p.pidMin = -v.maxMotorTorque;
    p.pidMax = 0.25f * v.maxMotorTorque;
    p.gripFactor = 0.60f;
    p.launchThrottle = v.maxMotorTorque;
    p.emulateSimulinkDelays = false;

    //                       target  kff   kp     ki      kd    kb (Ki/Kp)      kt     rise    fall
    const CornerParams front{0.12f,  0.9f, 15.0f, 480.0f, 0.0f, 480.0f / 15.0f, 50.0f, 800.0f, 2000.0f};
    const CornerParams rear{0.145f,  0.9f, 20.0f, 240.0f, 0.0f, 240.0f / 20.0f, 50.0f, 800.0f, 2000.0f};
    p.corner[FL] = front;
    p.corner[FR] = front;
    p.corner[RL] = rear;
    p.corner[RR] = rear;
    return p;
}

void TractionController::init(const ControllerParams& params) {
    params_ = params;
    const VehicleParams& v = params_.vehicle;
    const float ts = params_.sampleTime;

    slipAlpha_ = 1.0f - expf(-2.0f * kPi * params_.slipFilterHz * ts);
    ffAlpha_ = 1.0f - expf(-2.0f * kPi * params_.feedforwardFilterHz * ts);
    frontStaticAxleLoad_ = v.mass * v.gravity * v.frontWeightFraction;
    rearStaticAxleLoad_ = v.mass * v.gravity * (1.0f - v.frontWeightFraction);
    wheelForcePerMotorTorque_ = v.gearRatio * v.drivetrainEfficiency / v.tireRadius;
    inertiaTorquePerAccel_ =
        v.wheelInertia / (v.tireRadius * v.gearRatio * v.drivetrainEfficiency);
    effectiveMass_ = v.mass + 4.0f * v.wheelInertia / (v.tireRadius * v.tireRadius);
    maxWheelOmega_ = v.maxMotorRpm * 2.0f * kPi / 60.0f / v.gearRatio;
    taperOmega_ = v.speedTaperRpm * 2.0f * kPi / 60.0f / v.gearRatio;

    for (int i = 0; i < kNumWheels; ++i) {
        const CornerParams& c = params_.corner[i];
        // Fraction of peak tire force at the slip target (Force_At_Target_Slip).
        targetForceShape_[i] = -sinf(v.pacejkaC * atanf(v.pacejkaB * c.slipTarget));
        // Tire force to motor torque, scaled by Kff (Tire_To_Motor_Torque).
        gripToMotorTorque_[i] = c.kff * v.tireRadius / (v.gearRatio * v.drivetrainEfficiency);
    }
    reset();
}

// Load-sensitive friction of the longitudinal tire fit (Pacejka_Tire_Model).
float TractionController::tireMu(float normalLoad) const {
    const VehicleParams& v = params_.vehicle;
    return maxf(-(v.pacejkaD1 + v.pacejkaD2 * normalLoad / 1000.0f) * params_.gripFactor, 0.0f);
}

void TractionController::reset() {
    const VehicleParams& v = params_.vehicle;

    // Predicted launch operating point (params.m): each corner delivers the
    // lesser of its motor force and tire grip, and the resulting acceleration
    // moves load rearward. Iterate the coupled loads to a fixed point.
    const float motorForceMax = v.maxMotorTorque * wheelForcePerMotorTorque_;
    float accel = 0.0f;
    float loadFront = 0.0f;
    float loadRear = 0.0f;
    for (int iteration = 0; iteration < 50; ++iteration) {
        const float transfer = v.mass * accel * v.cgHeight / (2.0f * v.wheelbase);  // Per wheel [N]
        loadFront = 0.5f * frontStaticAxleLoad_ - transfer;
        loadRear = 0.5f * rearStaticAxleLoad_ + transfer;
        const float force = 2.0f * minf(motorForceMax, tireMu(loadFront) * loadFront) +
                            2.0f * minf(motorForceMax, tireMu(loadRear) * loadRear);
        accel = 0.5f * accel + 0.5f * force / v.mass;
    }
    const float launchGripFront = tireMu(loadFront) * loadFront;  // [N]
    const float launchGripRear = tireMu(loadRear) * loadRear;     // [N]

    for (int i = 0; i < kNumWheels; ++i) {
        const float launchGrip = isFront(i) ? launchGripFront : launchGripRear;
        // Grip_Filter starts at the launch grip, and the feedforward at the
        // torque it implies (Feedforward_Launch_XX), instead of ramping from 0.
        gripFiltered_[i] = launchGrip;
        feedforwardState_[i] =
            minf(v.maxMotorTorque, gripToMotorTorque_[i] * launchGrip * targetForceShape_[i]);
        // The predictor's "previous command" starts there too (Cmd_Previous_Tick).
        previousCommand_[i] = feedforwardState_[i];
        slipFiltered_[i] = 0.0f;
        slipDelayed_[i] = 0.0f;
        throttleDelayed_[i] = params_.launchThrottle;
        integrator_[i] = 0.0f;
        derivativeFilter_[i] = 0.0f;
    }
    accelFiltered_ = 0.0f;
    powerScaleDelayed_ = 1.0f;
}

void TractionController::step(const Inputs& in, Outputs& out) {
    const VehicleParams& v = params_.vehicle;
    const float ts = params_.sampleTime;
    const float speed = in.vehicleSpeed;

    // --- Load_Transfer_Predictor ---------------------------------------------
    // Predict acceleration from last tick's motor commands rather than waiting
    // for the measured acceleration, then the load transfer and the friction
    // at each predicted load. 4*J/r^2 is the wheels' equivalent mass.
    const float commandSum =
        previousCommand_[FL] + previousCommand_[FR] + previousCommand_[RL] + previousCommand_[RR];
    const float drag = 0.5f * v.airDensity * v.dragArea * speed * speed;
    const float predictedAccel = (commandSum * wheelForcePerMotorTorque_ - drag) / effectiveMass_;
    const float axleTransfer = v.mass * predictedAccel * v.cgHeight / v.wheelbase;  // [N]
    const float downforce = 0.5f * v.airDensity * v.liftArea * speed * speed;
    const float loadFront = maxf(
        0.5f * (frontStaticAxleLoad_ + v.aeroFrontFraction * downforce - axleTransfer), 0.0f);
    const float loadRear = maxf(
        0.5f * (rearStaticAxleLoad_ + (1.0f - v.aeroFrontFraction) * downforce + axleTransfer),
        0.0f);
    const float muFront = tireMu(loadFront);
    const float muRear = tireMu(loadRear);
    out.predictedAccel = predictedAccel;

    // --- Accel_Sample_500Hz + Accel_Filter (identical for every wheel) ---------
    accelFiltered_ = lowPass(ffAlpha_, in.longitudinalAccel, accelFiltered_);

    for (int i = 0; i < kNumWheels; ++i) {
        const CornerParams& c = params_.corner[i];
        const float omega = in.wheelSpeed[i];

        // --- Slip_Estimator + Slip_Sample_500Hz + Slip_Filter ------------------
        // Slip ratio with a low-speed floor on the denominator.
        const float slip = (omega * v.tireRadius - speed) / maxf(speed, v.slipSpeedFloor);
        slipFiltered_[i] = lowPass(slipAlpha_, slip, slipFiltered_[i]);

        // One-tick delays the model uses to stand in for hardware latency.
        float slipForControl = slipFiltered_[i];
        float throttle = in.driverTorqueRequest[i];
        if (params_.emulateSimulinkDelays) {
            slipForControl = slipDelayed_[i];  // Slip_Compute_Delay
            slipDelayed_[i] = slipFiltered_[i];
            throttle = throttleDelayed_[i];  // Driver_Request_Delay
            throttleDelayed_[i] = in.driverTorqueRequest[i];
        }

        // --- Grip_Estimate: filtered mu*Fz at the predicted load ----------------
        const float load = isFront(i) ? loadFront : loadRear;
        const float mu = isFront(i) ? muFront : muRear;
        gripFiltered_[i] = lowPass(ffAlpha_, mu * load, gripFiltered_[i]);

        // --- Feedforward ----------------------------------------------------------
        // Torque the tire can take at the slip target plus the torque that spins
        // the wheel up with the car, slew limited, then capped at the motor limit.
        // The slew state is kept before the cap, as in Feedforward_Memory.
        const float feedforwardRaw =
            gripFiltered_[i] * targetForceShape_[i] * gripToMotorTorque_[i] +
            accelFiltered_ * inertiaTorquePerAccel_;
        feedforwardState_[i] += clampf(feedforwardRaw - feedforwardState_[i],
                                       -c.ffFallRate * ts, c.ffRiseRate * ts);
        const float feedforward = clampf(feedforwardState_[i], 0.0f, v.maxMotorTorque);

        // --- PID ---------------------------------------------------------------------
        // Simulink PID Controller: discrete, parallel form, Forward Euler,
        // output limits, back-calculation anti-windup, and tracking mode.
        //   u_raw = Kp*e + I + N*(Kd*e - F)        (P + integrator + filtered D)
        //   u     = clamp(u_raw, pidMin, pidMax)
        const float error = c.slipTarget - slipForControl;  // Slip_Error
        const float derivative = params_.pidFilterN * (c.kd * error - derivativeFilter_[i]);
        const float pidRaw = c.kp * error + integrator_[i] + derivative;
        const float pid = clampf(pidRaw, params_.pidMin, params_.pidMax);

        // --- Torque_Limiter: cap at the driver request and the motor limit --------
        const float torqueRequest =
            clampf(minf(feedforward + pid, throttle), 0.0f, v.maxMotorTorque);

        // PID states update after the output (Forward Euler: new state = old +
        // Ts * input). Tracking pulls the integrator toward the correction that
        // actually got through the Torque_Limiter (Tracking_Residual); back-
        // calculation bleeds it off while the PID's own limits are active.
        const float tracking = torqueRequest - feedforward;
        integrator_[i] += ts * (c.ki * error + c.kt * (tracking - pid) + c.kb * (pid - pidRaw));
        derivativeFilter_[i] += ts * derivative;

        // --- SpeedLimitTaper: torque fades to zero over the last taper band -------
        const float taper = clampf((maxWheelOmega_ - omega) / taperOmega_, 0.0f, 1.0f);

        out.slip[i] = slip;
        out.feedforward[i] = feedforward;
        out.pidCorrection[i] = pid;
        out.torqueRequest[i] = torqueRequest;
        out.speedTaper[i] = taper;
        out.predictedNormalLoad[i] = load;
        out.predictedMu[i] = mu;
    }

    // --- Power_Limit -------------------------------------------------------------
    // Scale all four motors equally so the requested electrical power stays at
    // or below the limit.
    float requestedPower = 0.0f;  // [W]
    for (int i = 0; i < kNumWheels; ++i) {
        requestedPower += out.torqueRequest[i] * out.speedTaper[i] * in.wheelSpeed[i] *
                          v.gearRatio / v.motorInverterEfficiency;
    }
    const float powerScaleNow = minf(v.maxTractivePower / maxf(requestedPower, 1.0f), 1.0f);
    float powerScale = powerScaleNow;
    if (params_.emulateSimulinkDelays) {
        powerScale = powerScaleDelayed_;  // Power_Command_Delay_2ms
        powerScaleDelayed_ = powerScaleNow;
    }
    out.powerScale = powerScale;

    // --- Motor commands (ApplySpeedLimit, ApplyPowerScale) -------------------------
    for (int i = 0; i < kNumWheels; ++i) {
        out.motorTorqueCommand[i] = out.torqueRequest[i] * out.speedTaper[i] * powerScale;
        previousCommand_[i] = out.motorTorqueCommand[i];  // Predictor input next tick
    }
}

}  // namespace tc
