clear all;

% add path
addpath('functions')

% Load Vehicle Parameter
VehicleParam;
%=================================================
% SIMULATION PARAMETERS
% Time
dt = 0.01;            % Time step [s]
endTime = 60;         % End of simulation time[s]
T = (0:dt:endTime)';  % Time array [s]
N = size(T,1);        % Number of frames

% Input
FX1 = 0e2 * ones(N,1);  % Longitudinal input force for front left  [N]
FX2 = 0e2 * ones(N,1);  % Longitudinal input force for front right [N]
FX3 = 1e3 * ones(N,1);  % Longitudinal input force for rear left   [N]
FX4 = 1e3 * ones(N,1);  % Longitudinal input force for rear right  [N]

period = 10;                         % Period of cyclic input [s]
amp = 10 *pi/180;                    % Amplitude of angle[rad]
DELTAF = amp*sin(T *(2*pi)/ period); % Sine curve input
%DELTAF = 10*pi/180 * ones(N,1);      % Constant input
DELTAR = -DELTAF;
[DELTA1,DELTA2,DELTA3,DELTA4] = ackermannNl(DELTAF,DELTAR,d,l);

% Initialize position, angle, velocity
psi = -pi/2;     % Azimuth angle; North = 0 [rad]
east = 0;        % East position [m]
north = 0;       % North position [m]
r = 0;           % Yawrate [rad/s]
vx = 5;          % Longitudinal Velocity [m/s] ** Zero is not allowed (causes zero-division when calculate slip angle)
vy = 0;          % Lateral Velocity [m/s]
%=================================================

E_A = [];
N_A = [];
PSI_A = [];
YAWRATE_A = [];
VX_A = [];
VY_A = [];
FY1_A = [];
FY2_A = [];
FY3_A = [];
FY4_A = [];

for i = 1:N
    Fx1 = FX1(i);
    Fx2 = FX2(i);
    Fx3 = FX3(i);
    Fx4 = FX4(i);
    delta1 = DELTA1(i);
    delta2 = DELTA2(i);
    delta3 = DELTA3(i);
    delta4 = DELTA4(i);
    Fy1 = C1 * (-delta1 + atan2(vy + a * r, vx - d * r));
    Fy2 = C2 * (-delta2 + atan2(vy + a * r, vx + d * r));
    Fy3 = C3 * (-delta3 + atan2(vy - b * r, vx - d * r));
    Fy4 = C4 * (-delta4 + atan2(vy - b * r, vx + d * r));
       
    
    % Simulation
    F1cx = Fx1 * cos(delta1) - Fy1 * sin(delta1);
    F2cx = Fx2 * cos(delta2) - Fy2 * sin(delta2);
    F3cx = Fx3 * cos(delta3) - Fy3 * sin(delta3);
    F4cx = Fx4 * cos(delta4) - Fy4 * sin(delta4);
    F1cy = Fx1 * sin(delta1) + Fy1 * cos(delta1);
    F2cy = Fx2 * sin(delta2) + Fy2 * cos(delta2);
    F3cy = Fx3 * sin(delta3) + Fy3 * cos(delta3);
    F4cy = Fx4 * sin(delta4) + Fy4 * cos(delta4);
    Sigma1 = F1cx + F2cx + F3cx + F4cx;
    Sigma2 = F1cy + F2cy + F3cy + F4cy;  

    Psidot = r;
    Edot = -vy * cos(psi) - vx * sin(psi);
    Ndot =  vx * cos(psi) - vy * sin(psi);
    rdot = 1 / Iz * ( d * ( - F1cx + F2cx - F3cx + F4cx ) + a * ( F1cy + F2cy )...
        - b * ( F3cy + F4cy) );
    Vxdot =  r * vy + 1 / M * Sigma1;
    Vydot = -r * vx + 1 / M * Sigma2;
    
    psi = psi + Psidot * dt;
    east = east + Edot * dt;
    north = north + Ndot * dt;
    r = r + rdot * dt;
    vx = vx + Vxdot * dt;
    vy = vy + Vydot * dt;
    
    % Store Data
    E_A = [E_A; east];
    N_A = [N_A; north];
    PSI_A = [PSI_A; psi];
    YAWRATE_A = [YAWRATE_A; r];
    VX_A = [VX_A; vx];
    VY_A = [VY_A; vy];
    FY1_A = [FY1_A; Fy1];
    FY2_A = [FY2_A; Fy2];
    FY3_A = [FY3_A; Fy3];
    FY4_A = [FY4_A; Fy4];
end
    
% Plot Results
PlotSim;
