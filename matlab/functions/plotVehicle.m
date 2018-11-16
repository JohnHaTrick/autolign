function plotVehicle(east, north, psi, delta1, delta2, delta3, delta4, fx1, fx2, fx3, fx4, fy1, fy2, fy3, fy4)

% Param from VehicleParam
%-------------------------------------------------------
a = 1.50;             % Front length          [m]
b = 1.37;             % Rear length           [m]
d = 0.82;             % Half Tread            [m]
tw = 0.15;            % Tire half width       [m]
tl = 0.4;             % Tire half length      [m]
FORCE_ZOOM_X = 1e-3;  % Long. force indicator [-]
FORCE_ZOOM_Y = 1e-3;  % Lat. force indicator  [-]
%-------------------------------------------------------

% plot vehicle chassis
% tire matrix
tire = [tl tl -tl -tl tl; tw -tw -tw tw tw];

f1 = [0 fx1 0 0; 0 0 0 fy1];
f2 = [0 fx2 0 0; 0 0 0 fy2];
f3 = [0 fx3 0 0; 0 0 0 fy3];
f4 = [0 fx4 0 0; 0 0 0 fy4];

psi = psi + pi/2;
tread_f = d;
tread_r = d;
fm = 1.5*tw; % force margin to plot
%tread_r2 = tw+0.1;

% cog
gpos = [east;north]; 

fpos = trans2d([a; 0],[1;1], gpos, psi);
flpos = trans2d([a; tread_f],[1;1], gpos, psi);
frpos = trans2d([a; -tread_f],[1;1], gpos, psi);
f1pos = trans2d([a; tread_f+fm],[1;1], gpos, psi);
f2pos = trans2d([a; -(tread_f+fm)],[1;1], gpos, psi);

rpos = trans2d([-b; 0],[1;1], gpos, psi);
rlpos = trans2d([-b; tread_r],[1;1], gpos, psi);
rrpos = trans2d([-b; -tread_r],[1;1], gpos, psi);
f3pos = trans2d([-b; tread_r+fm],[1;1], gpos, psi);
f4pos = trans2d([-b; -(tread_r+fm)],[1;1], gpos, psi);

tire1 = trans2d(tire,[1;1],flpos,psi+delta1);
tire2 = trans2d(tire,[1;1],frpos,psi+delta2);
tire3 = trans2d(tire,[1;1],rlpos,psi+delta3);
tire4 = trans2d(tire,[1;1],rrpos,psi+delta4);
tirecolor = 'c';

force1 = trans2d(f1,[FORCE_ZOOM_X; FORCE_ZOOM_Y],f1pos,psi+delta1);
force2 = trans2d(f2,[FORCE_ZOOM_X; FORCE_ZOOM_Y],f2pos,psi+delta2);
force3 = trans2d(f3,[FORCE_ZOOM_X; FORCE_ZOOM_Y],f3pos,psi+delta3);
force4 = trans2d(f4,[FORCE_ZOOM_X; FORCE_ZOOM_Y],f4pos,psi+delta4);

hold on
% chassis center
plot([gpos(1),fpos(1)],[gpos(2),fpos(2)],'k','LineWidth',2);
plot([gpos(1),rpos(1)],[gpos(2),rpos(2)],'k','LineWidth',2);

% chassis front
plot([flpos(1),frpos(1)],[flpos(2),frpos(2)],'k','LineWidth',2);
plot([flpos(1),frpos(1)],[flpos(2),frpos(2)],'k','LineWidth',2);

% chassis rear
plot([rlpos(1),rrpos(1)],[rlpos(2),rrpos(2)],'k','LineWidth',2);
plot([rlpos(1),rrpos(1)],[rlpos(2),rrpos(2)],'k','LineWidth',2);

% Front tire
plot(tire1(1,:),tire1(2,:),'k','LineWidth',2)
fill(tire1(1,:),tire1(2,:),[.4,.4,.4]);
plot(tire2(1,:),tire2(2,:),'k','LineWidth',2)
fill(tire2(1,:),tire2(2,:),[.4,.4,.4]);

% Rear Tire
plot(tire3(1,:),tire3(2,:),'k','LineWidth',2)
fill(tire3(1,:),tire3(2,:),[.4,.4,.4]);
plot(tire4(1,:),tire4(2,:),'k','LineWidth',2)
fill(tire4(1,:),tire4(2,:),[.4,.4,.4]);
% plot(rtire(1,:)+rr1pos(1),rtire(2,:)+rr1pos(2),'k','LineWidth',1)
% fill(rtire(1,:)+rr1pos(1),rtire(2,:)+rr1pos(2),'w');
% plot(rtire(1,:)+rr2pos(1),rtire(2,:)+rr2pos(2),'k','LineWidth',1)
% fill(rtire(1,:)+rr2pos(1),rtire(2,:)+rr2pos(2),'w');
% plot(rtire(1,:)+rl1pos(1),rtire(2,:)+rl1pos(2),'k','LineWidth',1)
% fill(rtire(1,:)+rl1pos(1),rtire(2,:)+rl1pos(2),'w');
% plot(rtire(1,:)+rl2pos(1),rtire(2,:)+rl2pos(2),'k','LineWidth',1)
% fill(rtire(1,:)+rl2pos(1),rtire(2,:)+rl2pos(2),'w');

% Force
%keyboard;
% 
plot(force1(1,1:2) ,force1(2,1:2),'r','LineWidth',2);
plot(force1(1,3:4),force1(2,3:4),'g','LineWidth',2);
plot(force2(1,1:2) ,force2(2,1:2),'r','LineWidth',2);
plot(force2(1,3:4),force2(2,3:4),'g','LineWidth',2);
plot(force3(1,1:2) ,force3(2,1:2),'r','LineWidth',2);
plot(force3(1,3:4),force3(2,3:4),'g','LineWidth',2);
plot(force4(1,1:2) ,force4(2,1:2),'r','LineWidth',2);
plot(force4(1,3:4),force4(2,3:4),'g','LineWidth',2);

% draw_cog
drawCog(east,north,psi+pi/2,0.2);
axis equal;
%keyboard;
end
