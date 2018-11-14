% VEHICLE PARAMETERS

M = 1964;     % Mass         [kg]
a = 1.50;     % Front length [m]
b = 1.37;     % Rear length  [m]
l = a+b;      % Wheel base   [m]
d = 0.82;     % Half Tread   [m]
Iz = 2900;    % Inertia      [kg-m^2] 2250 = 1000*1.5*1.5
Cf = -140000; % Cornering Stiffness [N/rad]
Cr = -190000; % Cornering Stiffness [N/rad]
C1 = Cf/2;
C2 = Cf/2;
C3 = Cr/2;
C4 = Cr/2;

