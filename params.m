Ts = 0.002;             % 500 Hz

Kp = 200;               % Test value only
Ki = 800;               % Test value only
Kaw = Ki/Kp;            % Initial anti-windup setting

Cmin = -50;             % Maximum torque cut [Nm]
Cmax = 0;               % Controller cannot add torque

%% ---------------- Simulation ----------------
timedomain = 10;
simout = sim("TC_SIM.slx", timedomain);