% Animation sim
%{
Using these variables to draw.
N
E_A
N_A
PSI_A
DELTA1
DELTA2
DELTA3
DELTA4
FX1
FX2
FX3
FX4
FY1_A
FY2_A
FY3_A
FY4_A
%}
h = figure(100);clf;
interval = 10;

%rangeX = [min(E_A),max(E_A)];
%rangeY = [min(E_A),max(E_A)];

for i = 1:interval:N
    if(ishghandle(h) == 0) 
        % is figure is closed then return.
        return;
    end
    clf;
    plot(E_A(1:interval:i),N_A(1:interval:i));
    hold on;
    east = E_A(i);
    north = N_A(i);
    psi = PSI_A(i);
    delta1 = DELTA1(i);
    delta2 = DELTA2(i);
    delta3 = DELTA3(i);
    delta4 = DELTA4(i);
    fx1 = FX1(i);
    fx2 = FX2(i);
    fx3 = FX3(i);
    fx4 = FX4(i);
    fy1 = FY1_A(i);
    fy2 = FY2_A(i);
    fy3 = FY3_A(i);
    fy4 = FY4_A(i);
    plotVehicle(east,north,psi,delta1,delta2,delta3,delta4,fx1,fx2,fx3,fx4,fy1,fy2,fy3,fy4);
    %xlim(rangeX);
    %ylim(rangeY);
    grid on;
    axis equal;
    drawnow;
end