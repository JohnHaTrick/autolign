function drawCog(x,y,psi,radius)
alpha = linspace(0, 90, 6)/180*pi + psi;
patch(x+[0 radius*cos(alpha) 0], y+[0 radius*sin(alpha) 0], 'k')
hold on;
alpha = linspace(90, 180, 6)/180*pi + psi;
plot(x+radius*cos(alpha), y+radius*sin(alpha),'k');
alpha = linspace(180, 270, 6)/180*pi + psi;
patch(x+[0 radius*cos(alpha) 0], y+[0 radius*sin(alpha) 0], 'k')
alpha = linspace(270, 360, 6)/180*pi + psi;
plot(x+radius*cos(alpha), y+radius*sin(alpha),'k');
end