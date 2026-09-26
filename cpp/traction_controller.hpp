// traction_controller.hpp
//
// C++ port of the TC_organized.slx traction controller, for a 500 Hz loop.
//
// One call to TractionController::step() is one 2 ms tick of the Simulink
// controller:
//   Load_Transfer_Predictor -> per wheel: Grip_Estimate -> Feedforward,
//   slip -> PID, FF + PID -> Torque_Limiter -> speed-limit taper -> power limit.
//
// Names in [brackets] are params.m variables; block names are from
// TC_organized.slx. With emulateSimulinkDelays = true the outputs match the
// Simulink model to floating-point rounding.

#pragma once

#include <array>
#include <cstddef>

namespace tc {

// double reproduces Simulink exactly. float also works; on a single-precision
// FPU switch this and add 'f' suffixes to the literals in the .cpp.
using Real = double;

constexpr std::size_t kNumWheels = 4;

// Wheel order used by every array.
enum Wheel : std::size_t { FL = 0, FR = 1, RL = 2, RR = 3 };

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

struct CornerParams {
    Real slipTarget;  // Target slip ratio [-]                       [Slip_Target_XX]
    Real kff;         // Fraction of tire grip the feedforward uses  [Kff_XX]
    Real kp;          // Proportional gain [N*m per unit slip]       [Kp_XX]
    Real ki;          // Integral gain [N*m/s per unit slip]         [Ki_XX]
    Real kd;          // Derivative gain [N*m*s per unit slip]       [Kd_XX]
    Real kb;          // Back-calculation anti-windup gain [1/s]     [Kaw_XX]
    Real kt;          // Tracking gain [1/s]                         [Ktrack_XX]
    Real ffRiseRate;  // Feedforward slew limit, rising [N*m/s]      [Feedforward_RiseRate_XX]
    Real ffFallRate;  // Feedforward slew limit, falling [N*m/s]     [Feedforward_FallRate_XX]
};

struct VehicleParams {
    Real mass;                     // Vehicle + driver [kg]              [Mv]
    Real wheelbase;                // [m]                                [W]
    Real cgHeight;                 // [m]                                [h_cg]
    Real frontWeightFraction;      // Static front weight fraction [-]   [frontWeightFraction]
    Real gravity;                  // [m/s^2]                            [gravity]
    Real tireRadius;               // [m]                                [r]
    Real gearRatio;                // Motor-to-wheel ratio [-]           [fd]
    Real drivetrainEfficiency;     // Gearbox efficiency [-]             [Drive_Train]
    Real motorInverterEfficiency;  // Motor + inverter efficiency [-]    [motorInverterEfficiency]
    Real wheelInertia;             // Rotating inertia per corner [kg*m^2] [J_wheel_side]
    Real airDensity;               // [kg/m^3]                           [rho]
    Real dragArea;                 // Cd*A [m^2]                         [CDA]
    Real liftArea;                 // Cl*A [m^2]                         [CLA]
    Real aeroFrontFraction;        // Downforce share on front axle [-]  [aeroFrontFraction]
    Real maxMotorTorque;           // [N*m]                              [Max_Motor_Torque]
    Real maxMotorRpm;              // [rpm]                              [Max_Motor_RPM]
    Real speedTaperRpm;            // Torque taper band below max [rpm]  [Speed_Limit_Taper_RPM]
    Real maxTractivePower;         // Electrical power limit [W]         [maxTractivePower]
    Real pacejkaB;                 // Longitudinal tire fit              [Pacejka_B]
    Real pacejkaC;                 //                                    [Pacejka_C]
    Real pacejkaD1;                //                                    [Pacejka_D1]
    Real pacejkaD2;                // [1/kN]                             [Pacejka_D2]
    Real slipSpeedFloor;           // Slip denominator floor [m/s]       [slipSpeedFloor]
};

struct ControllerParams {
    Real sampleTime;           // Controller period [s]                      [Ts]
    Real slipFilterHz;         // Slip measurement low-pass [Hz]             [Slip_Filter_Hz]
    Real feedforwardFilterHz;  // Grip and acceleration low-pass [Hz]        [Feedforward_Filter_Hz]
    Real pidFilterN;           // PID derivative filter coefficient [1/s]    (PID block N)
    Real pidMin;               // PID output limits [N*m]                    [Cmin]
    Real pidMax;               //                                            [Cmax]
    Real gripFactor;           // Road grip scale on the tire fit [-]        [Grip_Fact]
    Real launchThrottle;       // Throttle assumed before the first tick     [Launch_Pedal_Initial]

    // true: add the three one-tick delays that TC_organized uses to stand in
    // for hardware latency (Slip_Compute_Delay, Driver_Request_Delay,
    // Power_Command_Delay_2ms), so step() matches the model exactly.
    // Keep false on the car: its loop already has real sensor/CAN latency.
    bool emulateSimulinkDelays;

    VehicleParams vehicle;
    std::array<CornerParams, kNumWheels> corner;  // FL, FR, RL, RR
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
    std::array<Real, kNumWheels> wheelSpeed;

    // Vehicle ground speed [m/s], positive forward. All four wheels are
    // driven, so this must come from an independent sensor (optical-flow
    // ground-speed sensor or the VectorNav INS), not from wheel speeds.
    Real vehicleSpeed;

    // Longitudinal acceleration [m/s^2], positive forward, from the IMU with
    // gravity and pitch removed.
    Real longitudinalAccel;

    // Driver torque request per motor [N*m], 0..maxMotorTorque, from the pedal
    // map. It caps the final command in the Torque_Limiter.
    std::array<Real, kNumWheels> driverTorqueRequest;
};

struct Outputs {
    // Motor torque command for each inverter [N*m at the motor shaft],
    // 0..maxMotorTorque. Simulink signal: TorqueCommand.
    std::array<Real, kNumWheels> motorTorqueCommand;

    // Diagnostics (worth logging over CAN):
    std::array<Real, kNumWheels> slip;                 // Measured slip ratio [-]
    std::array<Real, kNumWheels> feedforward;          // Feedforward torque [N*m]
    std::array<Real, kNumWheels> pidCorrection;        // PID output [N*m]
    std::array<Real, kNumWheels> torqueRequest;        // After Torque_Limiter [N*m] (TorqueRequested)
    std::array<Real, kNumWheels> speedTaper;           // Speed-limit factor [0..1]
    std::array<Real, kNumWheels> predictedNormalLoad;  // [N]
    std::array<Real, kNumWheels> predictedMu;          // [-]
    Real predictedAccel;                               // [m/s^2]
    Real powerScale;                                   // Power-limit factor [0..1]
};

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

class TractionController {
public:
    explicit TractionController(const ControllerParams& params = defaultParams());

    // Restore the launch initial conditions the simulation starts from
    // (feedforward at the predicted launch grip, PID states zero). Call when
    // a launch begins, e.g. while stationary with the throttle released or on
    // launch-control release. Without it, holding zero throttle lets tracking
    // wind the PID down to about -feedforward (-15 N*m after 1 s) and the
    // predictor drift to static loads, so the launch starts near 1 N*m.
    void reset();

    // Run one tick. Call exactly once per sampleTime (2 ms).
    Outputs step(const Inputs& in);

    // Update the road grip estimate; used from the next step() and reset().
    void setGripFactor(Real gripFactor) { params_.gripFactor = gripFactor; }

    const ControllerParams& params() const { return params_; }

private:
    Real tireMu(Real normalLoad) const;

    ControllerParams params_;

    // Constants derived from params_ in the constructor.
    Real slipAlpha_ = 0;                // Slip filter coefficient
    Real ffAlpha_ = 0;                  // Grip/acceleration filter coefficient
    Real frontStaticAxleLoad_ = 0;      // [N]
    Real rearStaticAxleLoad_ = 0;       // [N]
    Real wheelForcePerMotorTorque_ = 0; // fd*eta/r [N per N*m]
    Real inertiaTorquePerAccel_ = 0;    // J/(r*fd*eta) [N*m per m/s^2]
    Real effectiveMass_ = 0;            // m + 4*J/r^2 [kg]
    Real maxWheelOmega_ = 0;            // [rad/s]
    Real taperOmega_ = 0;               // [rad/s]
    std::array<Real, kNumWheels> targetForceShape_{};  // Pacejka shape at the slip target
    std::array<Real, kNumWheels> gripToMotorTorque_{}; // kff*r/(fd*eta) [N*m per N]

    // State carried between ticks.
    std::array<Real, kNumWheels> previousCommand_{};   // Last motor commands (predictor input)
    std::array<Real, kNumWheels> slipFiltered_{};      // Slip_Filter output
    std::array<Real, kNumWheels> slipDelayed_{};       // Slip_Compute_Delay (emulation only)
    std::array<Real, kNumWheels> throttleDelayed_{};   // Driver_Request_Delay (emulation only)
    std::array<Real, kNumWheels> gripFiltered_{};      // Grip_Filter output [N]
    std::array<Real, kNumWheels> feedforwardState_{};  // Feedforward_Memory (before the limit)
    std::array<Real, kNumWheels> integrator_{};        // PID integrator
    std::array<Real, kNumWheels> derivativeFilter_{};  // PID derivative filter state
    Real accelFiltered_ = 0;                           // Accel_Filter output [m/s^2]
    Real powerScaleDelayed_ = 1;                       // Power_Command_Delay_2ms (emulation only)
};

}  // namespace tc
