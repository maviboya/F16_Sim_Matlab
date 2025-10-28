%% F-16 Ucus Simulasyonu
clear;
clc;
g = 9.81;
% sabitler
Sref = 27.871;
b = 9.144;
c = 3.450336;

m = 11339.809;

V = [50;0;0];
w = [0;0;0];
t = 0;
dt_s = 0.001;

X = [0;0;-5000];
u = X;
eul = [0;0;0];
alt_list = [];
tic;
dcm = [1 0 0;0 1 0;0 0 1];
while true
    alt = -X(3);
    [de_der, da_der, dr_der, dt_yuzde] = ControlSurfaceDeflections();
    [Talt,P,rho,a] = AtmosphereModel(alt);
    [A,AM,alfa,beta,TAS,pdin,Mach] = AeroModel(V,w,rho,Sref,b,c,de_der,da_der,dr_der,a);
    [T,TM] = ThrustModel(dt_yuzde,dt_s,alt,Mach);
    [W,I] = WeightModel(dcm);
    F = T + A + W;
    M = TM + AM;
    [states,dcm] = EOM(F,M,m,I,V,w,X,eul,dt_s);

    if t > 200 || alt <= 0
        break;
    end
    V = states(:,1);
    w = states(:,2);
    X = states(:,3);
    eul = states(:,4);
    % u = [u X];
    t = t + dt_s;
end
toc;
% plot3(u(1,:),u(2,:),-u(3,:))