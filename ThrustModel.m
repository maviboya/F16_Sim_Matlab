function [thrust,thrust_moment] = ThrustModel(dt,h,alt,Mach)
persistent P3
if isempty(P3)
    P3 = 20;
end

P1 = ThrottleLever(dt);

if P1 >= 50 && P3 >= 50
    P2 = P1;
elseif P1 >= 50 && P3 < 50
    P2 = 60;
elseif P1 < 50 && P3 >= 50
    P2 = 40;
else
    P2 = P1;
end

tau = ThrottlerRate(P2,P3);
P3dot = tau * (P2 - P3);
P3 = EulerIntegration(h,P3,P3dot);


Mach = saturate(Mach,0.2,1);
alt = saturate(alt,0,15000);
[T1,T2,T3]= StaticThrust(alt,Mach);

if P3 < 50
    T = T1 + (T2 - T1) * P3/50;
else
    T = T2 + (T3 - T2) * (P3-50)/50;
end

thrust = [T;0;0];
thrust_moment = [0;0;0];
% Moment ekle, açısal momentum falan da ekle


end

%%
% gaz kolu statik
function P1 = ThrottleLever(dt)

if dt <= 0.77
    P1 = 64.94*dt;
else
    P1 = 217.38*dt - 117.38;
end

end

% gaz kolu dinamik
function dt = ThrottlerRate(P1,P3)
if P1 - P3 <= 25
    timeConstant = 1;
elseif 25 < P1 - P3 && P1 - P3 < 50
    timeConstant = 0.1;
else
    timeConstant = 1.9 - 0.036*(P1 - P3);
end

if P1 >= 50 && P3 >= 50
    dt = 5;
elseif P1 >= 50 && P3 < 50
    dt = timeConstant;
elseif P1 < 50 && P3 >= 50
    dt = 5;
else
    dt = timeConstant;
end

end
% statik itki
function [T1,T2,T3]= StaticThrust(alt,Mach)
persistent h_tablo Mach_tablo X Y T_mil T_idle T_max
if isempty(h_tablo)

    h_tablo = [0 3048 6096 9144 12192 15240]';
    Mach_tablo = [0.2 0.4 0.6 0.8 1]';

    T_idle = [2824	1890	3069	4492	5916	7562
        267	111	1535	3358	5026	6783
        -4537	-3158	-1334	1557	4048	6049
        -12010	-8451	-5782	-1099	2669	4893
        -16013	-6227	-2647	-1521	-890	3114]';

    T_mil = [56401	40699	28080	17970	10987	6227
        56089	41420	29401	19082	11565	6939
        56223	43764	31536	20728	12632	7384
        55111	45263	34472	23663	14456	8585
        51953	43804	35806	27133	16902	10275]';

    T_max = [95276	69834	49929	32573	19727	11565
        100970	74993	54488	36269	22240	12610
        107820	84112	61204	41300	25354	14300
        115959	93742	71057	49440	30513	17570
        128485	103723	81398	59977	38440	22494]';
    [X,Y] = meshgrid(Mach_tablo,h_tablo);
end
T1 = interp2(X,Y,T_idle,Mach,alt);
T2 = interp2(X,Y,T_mil,Mach,alt);
T3 = interp2(X,Y,T_max,Mach,alt);

end