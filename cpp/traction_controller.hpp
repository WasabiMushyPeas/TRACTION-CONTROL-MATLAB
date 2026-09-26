// traction_controller.hpp
//
// C++ port of the TC_organized.slx traction controller for a 500 Hz loop on an
// STM32H5 (Cortex-M33, single-precision FPU).
//
// One call to TractionController::step() is one 2 ms tick of the Simulink
// controller:
//   Load_Transfer_Predictor -> per wheel: Grip_Estimate -> Feedforward,
//   slip -> PID, FF + PID -> Torque_Limiter -> speed-limit taper -> power limit.
//
// Embedded constraints:
//   - float only: the M33 FPU is single precision, double would be emulated.
//   - No C++ standard library, heap, exceptions, RTTI, or static constructors.
//   - step() calls no library functions. init()/reset() use expf, sinf, and
//     atanf from <math.h> (newlib libm, part of the STM32CubeIDE toolchain).
//
// Names in [brackets] are params.m variables; block names are from
// TC_organized.slx.

#pragma once

namespace tc {

constexpr int kNumWheels = 4;

// Wheel order used by every array.
enum Wheel { FL = 0, FR = 1, RL = 2, RR = 3 };

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

struct CornerParams {
    float slipTarget;  // Target slip ratio [-]                       [Slip_Target_XX]
    float kff;         // Fraction of tire grip the feedforward uses  [Kff_XX]
    float kp;          // Proportional gain [N*m per unit slip]       [Kp_XX]
    float ki;          // Integral gain [N*m/s per unit slip]         [Ki_XX]
    float kd;          // Derivative gain [N*m*s per unit slip]       [Kd_XX]
    float kb;          // Back-calculation anti-windup gain [1/s]     [Kaw_XX]
    float kt;          // Tracking gain [1/s]                         [Ktrack_XX]
    float ffRiseRate;  // Feedforward slew limit, rising [N*m/s]      [Feedforward_RiseRate_XX]
    float ffFallRate;  // Feedforward slew limit, falling [N*m/s]     [Feedforward_FallRate_XX]
};

struct VehicleParams {
    float mass;                     // Vehicle + driver [kg]                [Mv]
    float wheelbase;                // [m]                                  [W]
    float cgHeight;                 // [m]                                  [h_cg]
    float frontWeightFraction;      // Static front weight fraction [-]     [frontWeightFraction]
    float gravity;                  // [m/s^2]                              [gravity]
    float tireRadius;               // [m]                                  [r]
    float gearRatio;                // Motor-to-wheel ratio [-]             [fd]
    float drivetrainEfficiency;     // Gearbox efficiency [-]               [Drive_Train]
    float motorInverterEfficiency;  // Motor + inverter efficiency [-]      [motorInverterEfficiency]
    float wheelInertia;             // Rotating inertia per corner [kg*m^2] [J_wheel_side]
    float airDensity;               // [kg/m^3]                             [rho]
    float dragArea;                 // Cd*A [m^2]                           [CDA]
    float liftArea;                 // Cl*A [m^2]                           [CLA]
    float aeroFrontFraction;        // Downforce share on front axle [-]    [aeroFrontFraction]
    float maxMotorTorque;           // [N*m]                                [Max_Motor_Torque]
    float maxMotorRpm;              // [rpm]                                [Max_Motor_RPM]
    float speedTaperRpm;            // Torque taper band below max [rpm]    [Speed_Limit_Taper_RPM]
    float maxTractivePower;         // Electrical power limit [W]           [maxTractivePower]
    float pacejkaB;                 // Longitudinal tire fit                [Pacejka_B]
    float pacejkaC;                 //                                      [Pacejka_C]
    float pacejkaD1;                //                                      [Pacejka_D1]
    float pacejkaD2;                // [1/kN]                               [Pacejka_D2]
    float slipSpeedFloor;           // Slip denominator floor [m/s]         [slipSpeedFloor]
};

struct ControllerParams {
    float sampleTime;           // Controller period [s]                      [Ts]
    float slipFilterHz;         // Slip measurement low-pass [Hz]             [Slip_Filter_Hz]
    float feedforwardFilterHz;  // Grip and acceleration low-pass [Hz]        [Feedforward_Filter_Hz]
    float pidFilterN;           // PID derivative filter coefficient [1/s]    (PID block N)
    float pidMin;               // PID output limits [N*m]                    [Cmin]
    float pidMax;               //                                            [Cmax]
    float gripFactor;           // Road grip scale on the tire fit [-]        [Grip_Fact]
    float launchThrottle;       // Throttle assumed before the first tick     [Launch_Pedal_Initial]

    // true: add the three one-tick delays that TC_organized uses to stand in
    // for hardware latency (Slip_Compute_Delay, Driver_Request_Delay,
    // Power_Command_Delay_2ms), for comparing against the Simulink model.
    // Keep false on the car: its loop already has real sensor/CAN latency.
    bool emulateSimulinkDelays;

    VehicleParams vehicle;
    CornerParams corner[kNumWheels];  // FL, FR, RL, RR
};

// Current params.m values (grip factor 0.60, front slip target 0.12, ...).
ControllerParams defaultParams();

// ---------------------------------------------------------------------------
// Signals
// ---------------------------------------------------------------------------

// Everything the controller needs each tick.
struct Inputs {
    // Wheel angular speed [rad/s], positive forward. From each inverter's
    // actual motor speed: motor_rpm * 2*pi/60 / gearRatio.
    float wheelSpeed[kNumWheels];

    // Vehicle ground speed [m/s], positive forward. All four wheels are
    // driven, so this must come from an independent sensor (optical-flow
    // ground-speed sensor or the VectorNav INS), not from wheel speeds.
    float vehicleSpeed;

    // Longitudinal acceleration [m/s^2], positive forward, from the IMU with
    // gravity and pitch removed.
    float longitudinalAccel;

    // Driver torque request per motor [N*m], 0..maxMotorTorque, from the pedal
    // map. It caps the final command in the Torque_Limiter.
    float driverTorqueRequest[kNumWheels];
};

struct Outputs {
    // Motor torque command for each inverter [N*m at the motor shaft],
    // 0..maxMotorTorque. Simulink signal: TorqueCommand.
    float motorTorqueCommand[kNumWheels];

    // Diagnostics (worth logging over CAN):
    float slip[kNumWheels];                 // Measured slip ratio [-]
    float feedforward[kNumWheels];          // Feedforward torque [N*m]
    float pidCorrection[kNumWheels];        // PID output [N*m]
    float torqueRequest[kNumWheels];        // After Torque_Limiter [N*m] (TorqueRequested)
    float speedTaper[kNumWheels];           // Speed-limit factor [0..1]
    float predictedNormalLoad[kNumWheels];  // [N]
    float predictedMu[kNumWheels];          // [-]
    float predictedAccel;                   // [m/s^2]
    float powerScale;                       // Power-limit factor [0..1]
};

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

class TractionController {
public:
    // Load the calibration, compute derived constants, and reset(). Call once
    // before the first step().
    void init(const ControllerParams& params);

    // Restore the launch initial conditions the simulation starts from
    // (feedforward at the predicted launch grip, PID states zero). Call when
    // a launch begins, e.g. while stationary with the throttle released or on
    // launch-control release. Without it, holding zero throttle lets tracking
    // wind the PID down to about -feedforward (-15 N*m after 1 s) and the
    // predictor drift to static loads, so the launch starts near 1 N*m.
    void reset();

    // Run one tick. Call exactly once per sampleTime (2 ms).
    void step(const Inputs& in, Outputs& out);

    // Update the road grip estimate; used from the next step() and reset().
    void setGripFactor(float gripFactor) { params_.gripFactor = gripFactor; }

    const ControllerParams& params() const { return params_; }

private:
    float tireMu(float normalLoad) const;

    ControllerParams params_ = {};

    // Constants derived from params_ in init().
    float slipAlpha_ = 0.0f;                 // Slip filter coefficient
    float ffAlpha_ = 0.0f;                   // Grip/acceleration filter coefficient
    float frontStaticAxleLoad_ = 0.0f;       // [N]
    float rearStaticAxleLoad_ = 0.0f;        // [N]
    float wheelForcePerMotorTorque_ = 0.0f;  // fd*eta/r [N per N*m]
    float inertiaTorquePerAccel_ = 0.0f;     // J/(r*fd*eta) [N*m per m/s^2]
    float effectiveMass_ = 0.0f;             // m + 4*J/r^2 [kg]
    float maxWheelOmega_ = 0.0f;             // [rad/s]
    float taperOmega_ = 0.0f;                // [rad/s]
    float targetForceShape_[kNumWheels] = {};   // Pacejka shape at the slip target
    float gripToMotorTorque_[kNumWheels] = {};  // kff*r/(fd*eta) [N*m per N]

    // State carried between ticks.
    float previousCommand_[kNumWheels] = {};   // Last motor commands (predictor input)
    float slipFiltered_[kNumWheels] = {};      // Slip_Filter output
    float slipDelayed_[kNumWheels] = {};       // Slip_Compute_Delay (emulation only)
    float throttleDelayed_[kNumWheels] = {};   // Driver_Request_Delay (emulation only)
    float gripFiltered_[kNumWheels] = {};      // Grip_Filter output [N]
    float feedforwardState_[kNumWheels] = {};  // Feedforward_Memory (before the limit)
    float integrator_[kNumWheels] = {};        // PID integrator
    float derivativeFilter_[kNumWheels] = {};  // PID derivative filter state
    float accelFiltered_ = 0.0f;               // Accel_Filter output [m/s^2]
    float powerScaleDelayed_ = 1.0f;           // Power_Command_Delay_2ms (emulation only)
};

}  // namespace tc
