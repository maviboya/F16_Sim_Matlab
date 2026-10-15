function [sourceTables,trajectoryCheck]=test_stevens_validation()
%TEST_STEVENS_VALIDATION Kaynak tabloları, üç model ve koordinat dönüşümü.
sourceTables=validate_stevens_tables();
p=F16Parameters();p.frames=F16Frames();
[~,~,powerDot]=F16ThrustRK4(.5,40,0,.4,p);
[~,~,powerDotUp]=F16ThrustRK4(.85,40,0,.4,p);
[~,~,powerDotDown]=F16ThrustRK4(.85,80,0,.4,p);
assert(abs(powerDot-(64.94*.5-40))<1e-12);
assert(abs(powerDotUp-20)<1e-12);
assert(abs(powerDotDown-5*(217.38*.85-117.38-80))<1e-12);
for model={'flat','rotating','3dof'}
    nominal=check_f16_trim(502*.3048,0,.30,model{1});
    assert(nominal.valid,'Nominal trim must be feasible: %s',model{1});
    badAlpha=check_f16_trim(130*.3048,0,.35,model{1});
    assert(~badAlpha.valid,'130 ft/s must not be reported as valid.');
    badThrust=check_f16_trim(600*.3048,50000*.3048,.30,model{1});
    assert(~badThrust.valid,'Insufficient thrust must not be reported as trim.');
    full=linearize_f16_full(nominal);half=linearize_f16_full(nominal,.5);
    assert(max(abs(full.jacobian-half.jacobian),[],'all')<1e-4, ...
        'Jacobian is too sensitive to step size: %s',model{1});
    fast=check_f16_trim(900*.3048,0,.30,model{1});lin=linearize_f16_full(fast);
    poles=lin.eigenvalues(imag(lin.eigenvalues)>1e-5);[~,index]=min(abs(poles));
    assert(abs(2*pi/abs(poles(index))-183.4)<.5,'Full-state phugoid regression.');
end
% Boylamsal hareket: gerçek 3-DOF ve düz Dünya türevleri aynı olmalı.
x=[150;.06;0;0;.07;0;0;.08;0;0;0;5000;40];u=[-2;0;0;.6];
d3=f16_validation_dynamics(x,u,'3dof',p);d6=f16_validation_dynamics(x,u,'flat',p);
assert(norm(d3([1 2 5 8 10 12 13])-d6([1 2 5 8 10 12 13]))<1e-10);
% Ortak koordinat adaptörünü bağımsız üretim ECEF adımıyla doğrula.
x=[150;.06;.015;.15;.07;.2;.03;.08;-.02;100;80;5000;40];
[d,info]=f16_validation_dynamics(x,u,'rotating',p);
step=1e-4;cfg=struct('windNED',zeros(3,1),'controls',u);
y=info.state;
% ECEF ötelemesini bağımsız NED Newton denklemiyle karşılaştır:
% dv_N/dt = F_N/m + g_N - (2*omega_IE^N + omega_NE^N) x v_N.
f=p.frames;[lat,lon,alt]=f.ecefToGeodetic(y(7:9),p);
EB=f.quatToDCM(y(10:13));NB=f.nedToECEF(lat,lon)'*EB;
vb=EB'*y(1:3);vn=NB*vb;earthN=p.omegaEarth*[cos(lat);0;-sin(lat)];
e2=p.flattening*(2-p.flattening);
n=p.aEarth/sqrt(1-e2*sin(lat)^2);m=p.aEarth*(1-e2)/(1-e2*sin(lat)^2)^1.5;
transportN=[vn(2)/(n+alt);-vn(1)/(m+alt);-vn(2)*tan(lat)/(n+alt)];
gravity=9.7803253359*(1+.00193185265241*sin(lat)^2)/sqrt(1-e2*sin(lat)^2)*(p.aEarth/(p.aEarth+alt))^2;
force=F16AircraftLoads(0,y,cfg,p,vb,info.earthRatesBody,zeros(3,1),alt);
expectedN=NB*force/p.mass+[0;0;gravity]-cross(2*earthN+transportN,vn);
actualN=NB*(info.bodyAcceleration+cross(x(7:9),vb));
assert(norm(actualN-expectedN)<1e-10,'ECEF and independent NED acceleration disagree.');
a=F16DynamicsRotating(0,y,cfg,p);b=F16DynamicsRotating(step/2,y+step*a/2,cfg,p);
c=F16DynamicsRotating(step/2,y+step*b/2,cfg,p);e=F16DynamicsRotating(step,y+step*c,cfg,p);
y=y+step*(a+2*b+2*c+e)/6;
actual=(decodeRotating(y,p)-x)/step;
assert(max(abs(actual-d)./max(1,abs(d)))<2e-4,'ECEF/local derivative transform failed.');
% Dönüş trimi NED'e göre açısal ivmeyi sıfırlamalı; ECEF'e göre değil.
turn=check_f16_trim(502*.3048,0,.35,'rotating',.3);
assert(turn.valid,'Rotating turn trim did not converge.');
localDot=f16_validation_dynamics(turn.state,turn.controls,'rotating',p);
assert(norm(localDot(7:9))<1e-10,'NED transport acceleration is missing from turn trim.');
assert(norm(localDot(4:6)-[0;0;.3])<1e-12,'Turn rate must be relative to local NED.');
trajectoryCheck=checkNEDTrajectory(turn,p);
% Dünya dönüşü sıfır ve eğrilik ihmal edilecek kadar küçükken düz modele dön.
limit=p;limit.omegaEarth=0;limit.flattening=0;limit.aEarth=1e12;limit.gFlat=9.7803253359;
x(10:12)=0;
flat=f16_validation_dynamics(x,u,'flat',limit,[0;0]);
rot=f16_validation_dynamics(x,u,'rotating',limit,[0;0]);
assert(max(abs(flat-rot)./max(1,abs(flat)))<1e-7,'Rotating model flat-Earth limit failed.');
fprintf('Source tables, three-model trims, Jacobians and ECEF transform passed.\n');
end

function audit=checkNEDTrajectory(tr,p)
% Ayrı NED durumları ve denklemleriyle 12 s açık çevrim uçuş.
% Üretim ECEF RHS'si veya karşılaştırma adaptörü NED integrasyonunda çağrılmaz.
% Yük tabloları ortaktır; bu kontrol aerodinamik veriyi değil EOM'u sınar.
f=p.frames;cfg=struct('windNED',zeros(3,1),'controls',tr.controls);
[~,initial]=f16_validation_dynamics(tr.state,tr.controls,'rotating',p,tr.locationDeg);
ecef=initial.state;lat=deg2rad(tr.locationDeg(1));lon=deg2rad(tr.locationDeg(2));
EN=f.nedToECEF(lat,lon);q=f.eulerToQuat(tr.state(4),tr.state(5),tr.state(6));
ned=[EN'*ecef(1:3);ecef(4:6);lat;lon;tr.state(12);q;ecef(14:17)];
dt=.01;steps=1200;audit.timeS=(0:steps)'*dt;
audit.positionErrorM=zeros(steps+1,1);audit.velocityErrorMps=zeros(steps+1,1);
audit.altitudeNEDM=zeros(steps+1,1);audit.altitudeECEFM=zeros(steps+1,1);
for k=1:steps
    ecef=advance(@(s)F16DynamicsRotating(0,s,cfg,p),ecef,dt);
    ned=advance(@nedRHS,ned,dt);
    EN=f.nedToECEF(ned(7),ned(8));
    position=f.geodeticToECEF(ned(7),ned(8),ned(9),p);
    audit.positionErrorM(k+1)=norm(position-ecef(7:9));
    audit.velocityErrorMps(k+1)=norm(EN*ned(1:3)-ecef(1:3));
    audit.altitudeNEDM(k+1)=ned(9);
    [~,~,audit.altitudeECEFM(k+1)]=f.ecefToGeodetic(ecef(7:9),p);
end
assert(max(audit.positionErrorM)<1e-4,'Independent NED/ECEF trajectories disagree.');
assert(max(audit.velocityErrorMps)<1e-5,'Independent NED/ECEF velocities disagree.');
fprintf('12 s independent NED/ECEF: max position %.6g m, velocity %.6g m/s; h %.6g m.\n', ...
    max(audit.positionErrorM),max(audit.velocityErrorMps),audit.altitudeNEDM(end));
% Baş açısı değişen nominal yatay dönüş etrafında zorlanmış varyasyon:
% delta_dot = A(t)*delta + [f(x_nom(t))-x_nom_dot(t)], delta(0)=0.
% Kontrolcü değildir; ilk sapmanın nedenini sınayan yerel hata hesabıdır.
linearDt=.05;nominal=tr.state;delta=zeros(13,4);
vb=tr.state(1)*[cos(tr.state(2))*cos(tr.state(3));sin(tr.state(3));sin(tr.state(2))*cos(tr.state(3))];
C0=f.quatToDCM(f.eulerToQuat(tr.state(4),tr.state(5),tr.state(6)));
earth0=C0'*p.omegaEarth*[cos(lat);0;-sin(lat)];
e2=p.flattening*(2-p.flattening);m0=p.aEarth*(1-e2)/(1-e2*sin(lat)^2)^1.5;
audit.linearTimeS=(0:round(12/linearDt))'*linearDt;
audit.linearAltitudeM=zeros(size(audit.linearTimeS));
audit.translationForcingAltitudeM=zeros(size(audit.linearTimeS));
audit.rotationForcingAltitudeM=zeros(size(audit.linearTimeS));
audit.coriolisForcingAltitudeM=zeros(size(audit.linearTimeS));
increments=[1e-3;1e-6*ones(8,1);.1;.1;.01;1e-3];
for k=1:numel(audit.linearTimeS)-1
    nominal(6)=tr.state(6)+.3*(k-.5)*linearDt;
    d=f16_validation_dynamics(nominal,tr.controls,'rotating',p,tr.locationDeg);
    midpoint=nominal;midpoint(10:11)=nominal(10:11)+linearDt*d(10:11)/2;
    d=f16_validation_dynamics(midpoint,tr.controls,'rotating',p,tr.locationDeg);
    desired=zeros(13,1);desired(6)=.3;desired(10:11)=d(10:11);
    A=zeros(13);
    for j=1:13
        xp=midpoint;xm=midpoint;xp(j)=xp(j)+increments(j);xm(j)=xm(j)-increments(j);
        A(:,j)=(f16_validation_dynamics(xp,tr.controls,'rotating',p,tr.locationDeg)- ...
            f16_validation_dynamics(xm,tr.controls,'rotating',p,tr.locationDeg))/(2*increments(j));
    end
    forcing=d-desired;parts=zeros(13,4);parts(:,1)=forcing;
    parts(1:3,2)=forcing(1:3);parts(7:9,3)=forcing(7:9);
    C=f.quatToDCM(f.eulerToQuat(midpoint(4),midpoint(5),midpoint(6)));
    latitude=lat+midpoint(10)/m0;
    coriolis=-2*cross(C'*p.omegaEarth*[cos(latitude);0;-sin(latitude)]-earth0,vb);
    dv=dot(vb,coriolis)/tr.state(1);
    parts(1:3,4)=[dv;(vb(1)*coriolis(3)-vb(3)*coriolis(1))/(vb(1)^2+vb(3)^2); ...
        (coriolis(2)-sin(tr.state(3))*dv)/(tr.state(1)*cos(tr.state(3)))];
    propagated=expm([A,parts;zeros(4,17)]*linearDt)*[delta;eye(4)];
    delta=propagated(1:13,:);nominal(10:11)=nominal(10:11)+linearDt*d(10:11);
    audit.linearAltitudeM(k+1)=delta(12,1);
    audit.translationForcingAltitudeM(k+1)=delta(12,2);
    audit.rotationForcingAltitudeM(k+1)=delta(12,3);
    audit.coriolisForcingAltitudeM(k+1)=delta(12,4);
end
audit.linearFinalRelativeError=abs(audit.linearAltitudeM(end)-audit.altitudeNEDM(end))/abs(audit.altitudeNEDM(end));
assert(audit.linearFinalRelativeError<.1,'Heading-dependent forcing does not explain initial departure.');
fprintf('Heading-forced linear prediction at 12 s: %.6g m; nonlinear mismatch %.3g percent.\n', ...
    audit.linearAltitudeM(end),100*audit.linearFinalRelativeError);
fprintf('Linear altitude contributions: translation %.6g m, rotation %.6g m.\n', ...
    audit.translationForcingAltitudeM(end),audit.rotationForcingAltitudeM(end));
fprintf('Coriolis contribution within translation: %.6g m.\n',audit.coriolisForcingAltitudeM(end));
    function d=nedRHS(s)
        vn=s(1:3);w=s(4:6);latitude=s(7);height=s(9);
        C=f.quatToDCM(s(10:13)/norm(s(10:13)));
        e2=p.flattening*(2-p.flattening);den=1-e2*sin(latitude)^2;
        n=p.aEarth/sqrt(den);m=p.aEarth*(1-e2)/den^1.5;
        earth=p.omegaEarth*[cos(latitude);0;-sin(latitude)];
        transport=[vn(2)/(n+height);-vn(1)/(m+height);-vn(2)*tan(latitude)/(n+height)];
        [force,moment,pd,sd]=F16AircraftLoads(0,s,cfg,p,C'*vn,w-C'*earth,zeros(3,1),height);
        g=9.7803253359*(1+.00193185265241*sin(latitude)^2)/sqrt(den)*(p.aEarth/(p.aEarth+height))^2;
        vd=C*force/p.mass+[0;0;g]-cross(2*earth+transport,vn);
        wd=p.inertia\(moment-cross(w,p.inertia*w+p.engineAngularMomentumBody));
        positionDot=[vn(1)/(m+height);vn(2)/((n+height)*cos(latitude));-vn(3)];
        qd=.5*f.quatMultiply(s(10:13),[0;w-C'*(earth+transport)]);
        d=[vd;wd;positionDot;qd;pd;sd];
    end
end

function y=advance(fun,y,dt)
a=fun(y);b=fun(y+dt*a/2);c=fun(y+dt*b/2);d=fun(y+dt*c);
y=y+dt*(a+2*b+2*c+d)/6;y(10:13)=y(10:13)/norm(y(10:13));
end

function x=decodeRotating(y,p)
f=p.frames;[lat,lon,h]=f.ecefToGeodetic(y(7:9),p);
C=f.quatToDCM(y(10:13));CN=f.nedToECEF(lat,lon)'*C;v=C'*y(1:3);
e2=p.flattening*(2-p.flattening);vn=CN*v;
n=p.aEarth/sqrt(1-e2*sin(lat)^2);m=p.aEarth*(1-e2)/(1-e2*sin(lat)^2)^1.5;
transport=[vn(2)/(n+h);-vn(1)/(m+h);-vn(2)*tan(lat)/(n+h)];
w=y(4:6)-C'*[0;0;p.omegaEarth]-CN'*transport;
lat0=deg2rad(41);lon0=deg2rad(29);e2=p.flattening*(2-p.flattening);
N=p.aEarth/sqrt(1-e2*sin(lat0)^2);M=p.aEarth*(1-e2)/(1-e2*sin(lat0)^2)^1.5;
x=[norm(v);atan2(v(3),v(1));asin(v(2)/norm(v));atan2(CN(3,2),CN(3,3)); ...
    -asin(CN(3,1));atan2(CN(2,1),CN(1,1));w;M*(lat-lat0);N*cos(lat0)*(lon-lon0);h;y(14)];
end
