// traction_controller.cpp
//
// Implementation of traction_controller.hpp. Each section of step() names the
// TC_organized.slx block it reproduces.

#include "traction_controller.hpp"

#include <algorithm>
#include <cmath>

namespace tc {
namespace {

constexpr Real kPi = 3.14159265358979323846;

Real clamp(Real x, Real lo, Real hi) { return std::min(std::max(x, lo), hi); }

bool isFront(std::size_t wheel) { return wheel == FL || wheel == FR; }

// First-order low-pass y = alpha*u + (1 - alpha)*y_prev. Same response as the
// model's Discrete Filter blocks (numerator alpha, denominator [1, -(1-alpha)]).
Real lowPass(Real alpha, Real input, Real previousOutput) {
    return alpha * input + (1 - alpha) * previousOutput;
}

}  // namespace

ControllerParams defaultParams() {
    constexpr Real kLbToKg = 0.45359237;
    constexpr Real kInToM = 0.0254;

    ControllerParams p{};

    VehicleParams& v = p.vehicle;
    v.mass = 473.92 * kLbToKg + 150.0 * kLbToKg;  // Car + 150 lbm driver = 283.0 kg
    v.wheelbase = 60.25 * kInToM;
    v.cgHeight = 10.4 * kInToM;
    v.frontWeightFraction = 0.50;
    v.gravity = 9.80665;
    v.tireRadius = 0.5 * 16.0 * kInToM;
    v.gearRatio = 12.5;
    v.drivetrainEfficiency = 0.92;
    v.motorInverterEfficiency = 1.00;
    v.wheelInertia = 0.35;
    v.airDensity = 1.225;
    v.dragArea = 1.20;
    v.liftArea = 4.40;
    v.aeroFrontFraction = 0.50;
    v.maxMotorTorque = 21.5;
    v.maxMotorRpm = 20000.0;
    v.speedTaperRpm = 500.0;
    v.maxTractivePower = 80e3;
    v.pacejkaB = 10.4;
    v.pacejkaC = -1.58;
    v.pacejkaD1 = -3.02;
    v.pacejkaD2 = 0.8;
    v.slipSpeedFloor = 0.5;

    p.sampleTime = 0.002;
    p.slipFilterHz = 25.0;
    p.feedforwardFilterHz = 20.0;
    p.pidFilterN = 100.0;
    p.pidMin = -v.maxMotorTorque;
    p.pidMax = 0.25 * v.maxMotorTorque;
    p.gripFactor = 0.60;
    p.launchThrottle = v.maxMotorTorque;
    p.emulateSimulinkDelays = false;

    //                      target  kff  kp    ki     kd   kb (Ki/Kp)    kt    rise   fall
    const CornerParams front{0.12,  0.9, 15.0, 480.0, 0.0, 480.0 / 15.0, 50.0, 800.0, 2000.0};
    const CornerParams rear{0.145,  0.9, 20.0, 240.0, 0.0, 240.0 / 20.0, 50.0, 800.0, 2000.0};
    p.corner = {front, front, rear, rear};
    return p;
}

TractionController::TractionController(const ControllerParams& params) : params_(params) {
    const VehicleParams& v = params_.vehicle;
    const Real ts = params_.sampleTime;

    slipAlpha_ = 1 - std::exp(-2 * kPi * params_.slipFilterHz * ts);
    ffAlpha_ = 1 - std::exp(-2 * kPi * params_.feedforwardFilterHz * ts);
    frontStaticAxleLoad_ = v.mass * v.gravity * v.frontWeightFraction;
    rearStaticAxleLoad_ = v.mass * v.gravity * (1 - v.frontWeightFraction);
    wheelForcePerMotorTorque_ = v.gearRatio * v.drivetrainEfficiency / v.tireRadius;
    inertiaTorquePerAccel_ =
        v.wheelInertia / (v.tireRadius * v.gearRatio * v.drivetrainEfficiency);
    effectiveMass_ = v.mass + 4 * v.wheelInertia / (v.tireRadius * v.tireRadius);
    maxWheelOmega_ = v.maxMotorRpm * 2 * kPi / 60 / v.gearRatio;
    taperOmega_ = v.speedTaperRpm * 2 * kPi / 60 / v.gearRatio;

    for (std::size_t i = 0; i < kNumWheels; ++i) {
        const CornerParams& c = params_.corner[i];
        // Fraction of peak tire force at the slip target (Force_At_Target_Slip).
        targetForceShape_[i] = -std::sin(v.pacejkaC * std::atan(v.pacejkaB * c.slipTarget));
        // Tire force to motor torque, scaled by Kff (Tire_To_Motor_Torque).
        gripToMotorTorque_[i] = c.kff * v.tireRadius / (v.gearRatio * v.drivetrainEfficiency);
    }
    reset();
}

// Load-sensitive friction of the longitudinal tire fit (Pacejka_Tire_Model).
Real TractionController::tireMu(Real normalLoad) const {
    const VehicleParams& v = params_.vehicle;
    return std::max(-(v.pacejkaD1 + v.pacejkaD2 * normalLoad / 1000) * params_.gripFactor,
                    Real{0});
}

void TractionController::reset() {
    const VehicleParams& v = params_.vehicle;

    // Predicted launch operating point (params.m): each corner delivers the
    // lesser of its motor force and tire grip, and the resulting acceleration
    // moves load rearward. Iterate the coupled loads to a fixed point.
    const Real motorForceMax = v.maxMotorTorque * wheelForcePerMotorTorque_;
    Real accel = 0;
    Real loadFront = 0;
    Real loadRear = 0;
    for (int iteration = 0; iteration < 50; ++iteration) {
        const Real transfer = v.mass * accel * v.cgHeight / (2 * v.wheelbase);  // Per wheel [N]
        loadFront = frontStaticAxleLoad_ / 2 - transfer;
        loadRear = rearStaticAxleLoad_ / 2 + transfer;
        const Real force = 2 * std::min(motorForceMax, tireMu(loadFront) * loadFront) +
                           2 * std::min(motorForceMax, tireMu(loadRear) * loadRear);
        accel = 0.5 * accel + 0.5 * force / v.mass;
    }
    const Real launchGripFront = tireMu(loadFront) * loadFront;  // [N]
    const Real launchGripRear = tireMu(loadRear) * loadRear;     // [N]

    for (std::size_t i = 0; i < kNumWheels; ++i) {
        const Real launchGrip = isFront(i) ? launchGripFront : launchGripRear;
        // Grip_Filter starts at the launch grip, and the feedforward at the
        // torque it implies (Feedforward_Launch_XX), instead of ramping from 0.
        gripFiltered_[i] = launchGrip;
        feedforwardState_[i] =
            std::min(v.maxMotorTorque, gripToMotorTorque_[i] * launchGrip * targetForceShape_[i]);
        // The predictor's "previous command" starts there too (Cmd_Previous_Tick).
        previousCommand_[i] = feedforwardState_[i];
        slipFiltered_[i] = 0;
        slipDelayed_[i] = 0;
        throttleDelayed_[i] = params_.launchThrottle;
        integrator_[i] = 0;
        derivativeFilter_[i] = 0;
    }
    accelFiltered_ = 0;
    powerScaleDelayed_ = 1;
}

Outputs TractionController::step(const Inputs& in) {
    const VehicleParams& v = params_.vehicle;
    const Real ts = params_.sampleTime;
    const Real speed = in.vehicleSpeed;
    Outputs out{};

    // --- Load_Transfer_Predictor ---------------------------------------------
    // Predict acceleration from last tick's motor commands rather than waiting
    // for the measured acceleration, then the load transfer and the friction
    // at each predicted load. 4*J/r^2 is the wheels' equivalent mass.
    const Real commandSum =
        previousCommand_[FL] + previousCommand_[FR] + previousCommand_[RL] + previousCommand_[RR];
    const Real drag = 0.5 * v.airDensity * v.dragArea * speed * speed;
    const Real predictedAccel = (commandSum * wheelForcePerMotorTorque_ - drag) / effectiveMass_;
    const Real axleTransfer = v.mass * predictedAccel * v.cgHeight / v.wheelbase;  // [N]
    const Real downforce = 0.5 * v.airDensity * v.liftArea * speed * speed;
    const Real loadFront = std::max(
        0.5 * (frontStaticAxleLoad_ + v.aeroFrontFraction * downforce - axleTransfer), Real{0});
    const Real loadRear = std::max(
        0.5 * (rearStaticAxleLoad_ + (1 - v.aeroFrontFraction) * downforce + axleTransfer),
        Real{0});
    const Real muFront = tireMu(loadFront);
    const Real muRear = tireMu(loadRear);
    out.predictedAccel = predictedAccel;

    // --- Accel_Sample_500Hz + Accel_Filter (identical for every wheel) ---------
    accelFiltered_ = lowPass(ffAlpha_, in.longitudinalAccel, accelFiltered_);

    for (std::size_t i = 0; i < kNumWheels; ++i) {
        const CornerParams& c = params_.corner[i];
        const Real omega = in.wheelSpeed[i];

        // --- Slip_Estimator + Slip_Sample_500Hz + Slip_Filter ------------------
        // Slip ratio with a low-speed floor on the denominator.
        const Real slip = (omega * v.tireRadius - speed) / std::max(speed, v.slipSpeedFloor);
        slipFiltered_[i] = lowPass(slipAlpha_, slip, slipFiltered_[i]);

        // One-tick delays the model uses to stand in for hardware latency.
        Real slipForControl = slipFiltered_[i];
        Real throttle = in.driverTorqueRequest[i];
        if (params_.emulateSimulinkDelays) {
            slipForControl = slipDelayed_[i];  // Slip_Compute_Delay
            slipDelayed_[i] = slipFiltered_[i];
            throttle = throttleDelayed_[i];  // Driver_Request_Delay
            throttleDelayed_[i] = in.driverTorqueRequest[i];
        }

        // --- Grip_Estimate: filtered mu*Fz at the predicted load ----------------
        const Real load = isFront(i) ? loadFront : loadRear;
        const Real mu = isFront(i) ? muFront : muRear;
        gripFiltered_[i] = lowPass(ffAlpha_, mu * load, gripFiltered_[i]);

        // --- Feedforward ----------------------------------------------------------
        // Torque the tire can take at the slip target plus the torque that spins
        // the wheel up with the car, slew limited, then capped at the motor limit.
        // The slew state is kept before the cap, as in Feedforward_Memory.
        const Real feedforwardRaw = gripFiltered_[i] * targetForceShape_[i] * gripToMotorTorque_[i] +
                                    accelFiltered_ * inertiaTorquePerAccel_;
        feedforwardState_[i] += clamp(feedforwardRaw - feedforwardState_[i],
                                      -c.ffFallRate * ts, c.ffRiseRate * ts);
        const Real feedforward = clamp(feedforwardState_[i], 0, v.maxMotorTorque);

        // --- PID ---------------------------------------------------------------------
        // Simulink PID Controller: discrete, parallel form, Forward Euler,
        // output limits, back-calculation anti-windup, and tracking mode.
        //   u_raw = Kp*e + I + N*(Kd*e - F)        (P + integrator + filtered D)
        //   u     = clamp(u_raw, pidMin, pidMax)
        const Real error = c.slipTarget - slipForControl;  // Slip_Error
        const Real derivative = params_.pidFilterN * (c.kd * error - derivativeFilter_[i]);
        const Real pidRaw = c.kp * error + integrator_[i] + derivative;
        const Real pid = clamp(pidRaw, params_.pidMin, params_.pidMax);

        // --- Torque_Limiter: cap at the driver request and the motor limit --------
        const Real torqueRequest = clamp(std::min(feedforward + pid, throttle), 0, v.maxMotorTorque);

        // PID states update after the output (Forward Euler: new state = old +
        // Ts * input). Tracking pulls the integrator toward the correction that
        // actually got through the Torque_Limiter (Tracking_Residual); back-
        // calculation bleeds it off while the PID's own limits are active.
        const Real tracking = torqueRequest - feedforward;
        integrator_[i] += ts * (c.ki * error + c.kt * (tracking - pid) + c.kb * (pid - pidRaw));
        derivativeFilter_[i] += ts * derivative;

        // --- SpeedLimitTaper: torque fades to zero over the last taper band -------
        const Real taper = clamp((maxWheelOmega_ - omega) / taperOmega_, 0, 1);

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
    Real requestedPower = 0;  // [W]
    for (std::size_t i = 0; i < kNumWheels; ++i) {
        requestedPower += out.torqueRequest[i] * out.speedTaper[i] * in.wheelSpeed[i] *
                          v.gearRatio / v.motorInverterEfficiency;
    }
    const Real powerScaleNow =
        std::min(v.maxTractivePower / std::max(requestedPower, Real{1}), Real{1});
    Real powerScale = powerScaleNow;
    if (params_.emulateSimulinkDelays) {
        powerScale = powerScaleDelayed_;  // Power_Command_Delay_2ms
        powerScaleDelayed_ = powerScaleNow;
    }
    out.powerScale = powerScale;

    // --- Motor commands (ApplySpeedLimit, ApplyPowerScale) -------------------------
    for (std::size_t i = 0; i < kNumWheels; ++i) {
        out.motorTorqueCommand[i] = out.torqueRequest[i] * out.speedTaper[i] * powerScale;
        previousCommand_[i] = out.motorTorqueCommand[i];  // Predictor input next tick
    }
    return out;
}

}  // namespace tc
