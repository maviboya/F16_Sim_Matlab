%% F-16 Ucus Simulasyonu
clear;
clc;
g = 9.81;
% sabitler
Sref = 27.871;
b = 9.144;
c = 3.450336;

m = 20500 * 0.45359237;

V = [50;0;0];
w = [0;0;0];
t = 0;
dt_s = 0.001;

X = [0;0;-5000];
Xref = [0;0;0];
Xcg = [0;0;0];
u = X;
eul = [0;0;0];
alt_list = [];

dcm = [1 0 0;0 1 0;0 0 1];

joystick = sim3d.io.Joystick('ID', 1);
% fig = axes;

t_gecmis = tic; % Simülasyonun genel süresini ölçmek için
while true
    % 1. Adım Başlangıcını Kaydet
    step_start_time = tic;

    alt = -X(3);
    [de_der, da_der, dr_der, dt_yuzde] = ControlSurfaceDeflections(joystick);
    [Talt,P,rho,a] = AtmosphereModel(alt);
    [A,AM,alfa,beta,TAS,pdin,Mach] = AeroModel(V,w,rho,Sref,b,c,de_der,da_der,dr_der,a,Xref,Xcg);
    [T,TM] = ThrustModel(dt_yuzde,dt_s,alt,Mach);
    [W,I] = WeightModel(dcm);
    F = T + A + W;
    M = TM + AM;
    [states,dcm] = EOM(F,M,m,I,V,w,X,eul,dt_s);


    V = states(:,1);
    w = states(:,2);
    X = states(:,3);
    eul = states(:,4);
    % u = [u X];
    t = t + dt_s;
    % if mod(round(t/dt_s), 50) == 0
    %     plot3(fig,X(1),X(2),-X(3),'ro','MarkerSize',3);
    %     hold on;
    %     drawnow limitrate;
    % end

    % 6. Gerçek Zaman Senkronizasyonu (PAUSE yerine zaman kontrolü)
    % elapsed_time = toc(step_start_time);
    % sleep_time = dt_s - elapsed_time;


    % if sleep_time > 0
    %     pause(sleep_time);
    %     % else
    %     % disp(['UYARI: Gerçek zamanlı çalışmıyor. Hesaplama süresi: ' num2str(elapsed_time*1000) ' ms']);
    % end

    if t > 10 || alt <= 0
        break;
    end
end
% toc(t_gecmis);