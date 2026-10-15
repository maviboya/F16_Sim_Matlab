function [force, moment, powerDot] = F16ThrustRK4(throttle, power, altitude, mach, p)
%F16THRUSTRK4 Motor modeli; güç yüzdesi RK4 ile entegre edilen bir durumdur.
% İtki tablolarında irtifa metre, itki newton cinsindendir.
% Girdi: gaz 0..1, gerçek motor gücü 0..100, irtifa m, Mach boyutsuz.
% Çıktı: gövde kuvveti N, ağırlık merkezine göre moment N*m, güç türevi %/s.
throttle = min(max(throttle,0),1);
% Stevens gaz kolu aktarımı: %77'den sonra art yakıcı bölgesine geçilir.
if throttle <= 0.77
    command = 64.94 * throttle;
else
    command = 217.38 * throttle - 117.38;
end
% 50 güç eşiği geçilirken ara hedef kullanılır; motor ani kip atlamaz.
if command >= 50 && power < 50
    target = 60;
elseif command < 50 && power >= 50
    target = 40;
else
    target = command;
end
delta = target - power;
% rate, zaman sabitinin tersi (1/s); güç durumunun türevini belirler.
if power >= 50
    rate = 5;
elseif delta <= 25
    rate = 1;
elseif delta < 50
    rate = 1.9 - 0.036 * delta;
else
    rate = 0.1;
end
powerDot = rate * delta;

% Sabit Stevens tabloları RK4'ün her ara çağrısında yeniden kurulmaz.
% Ek A'daki özgün lbf düğümleri topluca N'a çevrilir; ara satırlar
% yuvarlatılmış newton sayılarıyla tutulmaz.
persistent h m idle mil maximum
if isempty(h)
    h = [0 3048 6096 9144 12192 15240];
    m = [0 0.2 0.4 0.6 0.8 1];
    lbfToN = 4.4482216152605;
    idle = [1060 670 880 1140 1500 1860;
            635 425 690 1010 1330 1700;
            60 25 345 755 1130 1525;
            -1020 -710 -300 350 910 1360;
            -2700 -1900 -1300 -247 600 1100;
            -3600 -1400 -595 -342 -200 700] * lbfToN;
    mil = [12680 9150 6200 3950 2450 1400;
           12680 9150 6313 4040 2470 1400;
           12610 9312 6610 4290 2600 1560;
           12640 9839 7090 4660 2840 1660;
           12390 10176 7750 5320 3250 1930;
           11680 9848 8050 6100 3800 2310] * lbfToN;
    maximum = [20000 15000 10800 7000 4000 2500;
               21420 15700 11225 7323 4435 2600;
               22700 16860 12250 8154 5000 2835;
               24240 18910 13760 9285 5700 3215;
               26070 21075 15975 11115 6860 3950;
               28886 23319 18300 13484 8642 5057] * lbfToN;
end
% Tablonun dışındaki Mach/irtifada kenar değerini kullan; ekstrapolasyon yok.
hq = min(max(altitude,h(1)),h(end));
mq = min(max(mach,m(1)),m(end));
ti = interp2(h,m,idle,hq,mq);
tm = interp2(h,m,mil,hq,mq);
tx = interp2(h,m,maximum,hq,mq);
% 0..50: rölanti-militer; 50..100: militer-azami itki arası doğrusal geçiş.
if power < 50
    thrust = ti + (tm-ti) * power/50;
else
    thrust = tm + (tx-tm) * (power-50)/50;
end
force = [thrust;0;0];
% Bu kol momenti itki doğrultusunun CG'den sapmasıdır; rotor jiroskopiği
% F16Dynamics içindeki toplam açısal momentum denklemine ayrıca girer.
moment = cross(p.Xeng-p.Xcg,force);
end
