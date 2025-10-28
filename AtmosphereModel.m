function [T,P,rho,a] = AtmosphereModel(altitude)
rho0 = 1.225;
T0 = 273.15 + 15;
P0 = 101325;
a0 = 340;

teta = 1 - 0.0000068756*altitude;
delta = teta^5.2561;
gama = teta^4.2561;

T = T0 * teta;
P = P0 * delta;
rho = rho0 * gama;
a = a0; % düzelt bu ses hızı şeysini

end