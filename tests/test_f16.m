function test_f16()
%TEST_F16 Proje kökünden çalıştırılan kısa gerileme kontrolleri.
% Amaç uçuş testi doğrulaması değil; birim, dönüşüm ve kod akışı hatalarını
% küçük örneklerle yakalamaktır.
p = F16Parameters();
f = F16Frames();
assert(p.inertia(1,3) < 0);

% WGS-84 konum gidiş-dönüşü ve dönüş matrisinin ortogonalliği.
lat = 41*pi/180; lon = 29*pi/180; h = 5000;
r = f.geodeticToECEF(lat,lon,h,p);
[lat2,lon2,h2] = f.ecefToGeodetic(r,p);
assert(abs(lat2-lat) < 1e-11 && abs(lon2-lon) < 1e-11 && abs(h2-h) < 1e-5);
C = f.nedToECEF(lat,lon);
assert(norm(C'*C-eye(3),'fro') < 1e-12);
q = f.dcmToQuat(C);
assert(norm(f.quatToDCM(q)-C,'fro') < 1e-12);

[T,P,rho,a] = AtmosphereModel(5000);
% Atmosfer SI birimleri ve yüksek hızdaki yapay doyumun kaldırılması.
assert(abs(T-255.65) < 1e-8 && P > 50000 && P < 60000);
assert(rho > 0 && a > 0);
[~,~,~,~,tas] = AeroModel([250;0;0],[0;0;0],rho, ...
    p.Sref,p.span,p.chord,0,0,0,a,p.Xref,p.Xcg);
assert(abs(tas-250) < 1e-9); % yüksek hava hızı yapay olarak kısılmamalı

% Stevens Ek A.8'in yanal düğümlerini ve işaretlerini sınar.
% Alfa=5°, beta=10° için Cl=-0,024 ve Cn=0,042; ters beta ters momenttir.
speed = 150;
velocity = speed*[cosd(10)*cosd(5);sind(10);cosd(10)*sind(5)];
[~,positiveMoment,~,~,~,dynamicPressure] = AeroModel( ...
    velocity,[0;0;0],rho,p.Sref,p.span,p.chord,0,0,0,a,p.Xref,p.Xcg);
velocity(2) = -velocity(2);
[~,negativeMoment] = AeroModel( ...
    velocity,[0;0;0],rho,p.Sref,p.span,p.chord,0,0,0,a,p.Xref,p.Xcg);
momentScale = dynamicPressure*p.Sref*p.span;
assert(abs(positiveMoment(1)/momentScale+0.024) < 1e-12);
assert(abs(positiveMoment(3)/momentScale-0.042) < 1e-12);
assert(norm(positiveMoment([1 3])+negativeMoment([1 3])) < 1e-9);

% Alfa=5°, beta=0°: 20° kanatçık ve 30° dümen tablo türevlerinin tam
% referans açılarıdır. Beklenen sayılar Wiley Ek A.8'den bağımsız alınır.
velocity = speed*[cosd(5);0;sind(5)];
[forceAileron,momentAileron] = AeroModel(velocity,[0;0;0],rho, ...
    p.Sref,p.span,p.chord,0,20,0,a,p.Xref,p.Xcg);
[forceRudder,momentRudder] = AeroModel(velocity,[0;0;0],rho, ...
    p.Sref,p.span,p.chord,0,0,30,a,p.Xref,p.Xcg);
assert(abs(forceAileron(2)/(dynamicPressure*p.Sref)-0.021) < 1e-12);
assert(abs(momentAileron(1)/momentScale+0.052) < 1e-12);
assert(abs(momentAileron(3)/momentScale-(-0.009)) < 1e-12);
assert(abs(forceRudder(2)/(dynamicPressure*p.Sref)-0.086) < 1e-12);
assert(abs(momentRudder(1)/momentScale-0.014) < 1e-12);
assert(abs(momentRudder(3)/momentScale+0.045) < 1e-12);

[force,~,powerDot] = F16ThrustRK4(0,0,0,0,p);
% Deniz seviyesi/Mach 0 rölanti tablosunun ilk noktası.
assert(abs(force(1)-1060*4.4482216152605) < 1e-8);
assert(abs(powerDot) < 1e-12);

[aeroDeg,surfaceDot] = F16Actuators([25;21.5;30],[0;0;0],p);
% Büyük basamakta üç yüzey kendi hız sınırına ulaşır.
assert(isequal(aeroDeg,[0;0;0]));
assert(isequal(surfaceDot,[60;80;120]));
[aeroDeg,surfaceDot] = F16Actuators([100;100;100],[25;21.5;30],p);
assert(isequal(aeroDeg,[24;21.5;30]) && isequal(surfaceDot,[0;0;0]));
[~,surfaceDot] = F16Actuators([-25;0;0],[25;0;0],p);
assert(surfaceDot(1) == -60); % sınırdaki yüzey ters komutta geri döner

% 1° basamak hız sınırına erişmez; 0,0495 s sonunda %63,2 yanıt beklenir.
smallStep = run_f16_flat(struct('duration',0.0495,'dt',0.00495, ...
    'controls',[1;0;0;0.5],'initialActuatorDeg',[0;0;0], ...
    'stopAtGround',false));
assert(abs(smallStep.actuatorDeg(1,end)-(1-exp(-1))) < 1e-5);

cfg = struct('duration',0.2,'dt',0.02,'stopAtGround',false);
% Her iki Dünya seçimi aynı durum boyutunu ve sonlu değerleri üretmeli.
flat = run_f16_flat(cfg);
ellipsoid = run_f16_rotating(cfg);
assert(numel(flat.time) == 11 && numel(ellipsoid.time) == 11);
assert(all(isfinite(flat.state(:))) && all(isfinite(ellipsoid.state(:))));
assert(max(abs(sqrt(sum(flat.state(10:13,:).^2,1))-1)) < 1e-12);
assert(max(abs(sqrt(sum(ellipsoid.state(10:13,:).^2,1))-1)) < 1e-12);
assert(abs(flat.altitude(1)-5000) < 1e-12);
assert(abs(ellipsoid.altitude(1)-5000) < 1e-5);
assert(norm(flat.velocityBody(:,1)-ellipsoid.velocityBody(:,1)) < 1e-9);
assert(norm(flat.attitudeEulerDeg(:,1)-ellipsoid.attitudeEulerDeg(:,1)) < 1e-8);
assert(size(flat.state,1) == 17 && size(ellipsoid.state,1) == 17);
assert(isequal(flat.actuatorDeg(:,1),[0;0;0]));
assert(isequal(run_f16_flat(struct('duration',0)).state(:,1),flat.state(:,1)));
assert(norm(run_f16_rotating(struct('duration',0)).state(:,1) ...
    -ellipsoid.state(:,1)) < 1e-12);

cfg = struct('duration',0.1,'dt',0.005,'stopAtGround',false, ...
    'controls',[25;21.5;30;0.5],'initialActuatorDeg',[0;0;0]);
% Açıkça sıfır yüzey açısıyla başlatılan büyük komutta geçişi ve üst
% sınırları denetle; servo yanıtı Dünya seçimine bağlı olmamalıdır.
stepped = run_f16_flat(cfg);
assert(all(stepped.actuatorDeg(:,end) > 0));
assert(all(stepped.actuatorDeg(:,end) <= [6;8;12]+1e-8));
assert(all(stepped.actuatorDeg(:) >= 0));
steppedEllipsoid = run_f16_rotating(cfg);
assert(max(abs(stepped.actuatorDeg(:)-steppedEllipsoid.actuatorDeg(:))) < 1e-12);
cfg.duration = 0.02;
cfg.controls = [100;100;100;0.5];
cfg.initialActuatorDeg = [25;21.5;30];
atLimit = run_f16_rotating(cfg);
assert(max(max(abs(atLimit.actuatorDeg-[25;21.5;30]))) < 1e-12);

% Rotor momentumu yalnız açısal ivmenin çapraz çarpım terimini değiştirir.
p.frames = f;
for earthCase = {'flat','ellipsoid'}
    cfg = struct('earth',earthCase{1},'windNED',[0;0;0], ...
        'controls',[0;0;0;0.5]);
    if strcmp(cfg.earth,'flat')
        yi = flat.state(:,1);
    else
        yi = ellipsoid.state(:,1);
    end
    yi(4:6) = [0.12;0.2;-0.3];
    if strcmp(cfg.earth,'flat')
        dynamics = @F16DynamicsFlat;
    else
        dynamics = @F16DynamicsRotating;
    end
    fullDerivative = dynamics(0,yi,cfg,p);
    pWithoutRotor = p;
    pWithoutRotor.engineAngularMomentumBody = [0;0;0];
    noRotorDerivative = dynamics(0,yi,cfg,pWithoutRotor);
    expected = p.inertia\(-cross(yi(4:6),p.engineAngularMomentumBody));
    assert(norm(fullDerivative(4:6)-noRotorDerivative(4:6)-expected) < 1e-10);
end

% Kitabın boylamsal 3 DOF türevleri, yanal durumlar sıfırken düz Dünya
% 6 DOF denklemlerinin u,w,q,kuzey,aşağı,güç,elevatör bileşenleriyle örtüşür.
cfg6 = struct('earth','flat','windNED',[4;0;-2], ...
    'controls',[-2;0;0;0.6]);
cfg3 = struct('windNED',cfg6.windNED,'controls',[-2;0.6]);
state3 = [140;5;0.08;120;-5000;0.1;30;-1];
pitchQuat = f.eulerToQuat(0,state3(6),0);
state6 = [state3(1);0;state3(2);0;state3(3);0; ...
    state3(4);0;state3(5);pitchQuat;state3(7);state3(8);0;0];
derivative3 = F16Dynamics3DOF(0,state3,cfg3,p);
derivative6 = F16DynamicsFlat(0,state6,cfg6,p);
assert(norm(derivative3([1:5 7:8])- ...
    derivative6([1 3 5 7 9 14 15])) < 1e-10);
assert(abs(derivative3(6)-state3(3)) < 1e-12);

longitudinal = run_f16_3dof(struct('duration',0.2,'dt',0.01, ...
    'controls',[-2;0.6],'initialElevatorDeg',0,'stopAtGround',false));
assert(size(longitudinal.state,1) == 8 && numel(longitudinal.time) == 21);
assert(all(isfinite(longitudinal.state(:))));
assert(isequal(longitudinal.positionNED(2,:),zeros(1,21)));
assert(all(longitudinal.actuatorDeg >= -p.actuatorLimitDeg(1)));
assert(all(longitudinal.actuatorDeg <= p.actuatorLimitDeg(1)));

% Stevens 3.6-3 nominal deniz seviyesi trimi: 502 ft/s, Xcg=%35 kord.
nominal = check_f16_trim(502*0.3048,0,0.35);
assert(abs(nominal.angleOfAttackDeg-0.03691*180/pi) < 0.005);
assert(abs(nominal.elevatorDeg-(-0.7588)) < 0.005);
assert(abs(nominal.throttle-0.1385) < 0.001);
assert(norm(nominal.accelerationResidual) < 1e-7);
fprintf('F16 regression checks passed.\n');
end
