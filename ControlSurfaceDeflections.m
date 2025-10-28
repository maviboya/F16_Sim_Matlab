function [de_der,da_der,dr_der,dt_yuzde]= ControlSurfaceDeflections(joystick)
[axes, ~, ~] = read(joystick); 
de_der = axes(2)*25; % Initialize deflection for elevator
da_der = axes(1)*20; % Initialize deflection for aileron
dr_der = axes(3)*30; % Initialize deflection for rudder
dt_yuzde = interp1([-1 1],[1 0],axes(4));
end