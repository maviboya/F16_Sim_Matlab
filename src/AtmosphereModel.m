function [T,P,rho,a] = AtmosphereModel(altitude)
%ATMOSPHEREMODEL 20 km'ye kadar SI birimlerinde standart atmosfer yaklaşımı.
% Girdi geometrik irtifadır; bu aralıkta jeopotansiyel irtifaya yakın alınır.
% Çıktılar sırasıyla sıcaklık K, basınç Pa, yoğunluk kg/m^3, ses hızı m/s.
R = 287.05;
gamma = 1.4;
g0 = 9.80665;
T0 = 288.15;
P0 = 101325;
L = 0.0065;  % K/m
% Model aralığı dışındaki sorgular sınır irtifasının atmosferini kullanır.
h = min(max(altitude,0),20000);
if h <= 11000
    % Troposfer: sıcaklık doğrusal azalır, basınç hidrostatik kuvvet yasasıdır.
    T = T0-L*h;
    P = P0*(T/T0)^(g0/(R*L));
else
    % 11–20 km: izotermal tabaka ve üstel basınç değişimi.
    T = T0-L*11000;
    P11 = P0*(T/T0)^(g0/(R*L));
    P = P11*exp(-g0*(h-11000)/(R*T));
end
% İdeal gaz bağıntısı ve sabit gamma ile ses hızı.
rho = P/(R*T);
a = sqrt(gamma*R*T);
end
