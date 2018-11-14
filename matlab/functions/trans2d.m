function[Zout] = trans2d(Zin, Zoom, Move, theta)
% Zin = [xin; yin]
% Zoom = [zoomX; zoomY]
% Move = [moveX; moveY]
% xin, yin : row vector
zoomX = Zoom(1,:);
zoomY = Zoom(2,:);
moveX = Move(1,:);
moveY = Move(2,:);
ONE = ones(1,size(Zin,2));
Zin = [Zin; ONE];

Rot = [zoomX*cos(theta), zoomY*-sin(theta), 0;...
    zoomX*sin(theta), zoomY*cos(theta), 0;...
    0, 0, 1]; % rotation matrix
Z1 = Rot * Zin;

Move = [1 0 moveX; 0 1 moveY; 0 0 1];
Zout = Move * Z1;

Zout = Zout(1:2,:);

end


