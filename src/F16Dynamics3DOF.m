function derivative = F16Dynamics3DOF(time,state,cfg,p)
%F16DYNAMICS3DOF Kitaptaki düz Dünya boylamsal 3 DOF denklemleri.
% Durum: [u;w;q;p_N;p_D;theta;motor gücü;elevatör açısı].
% NED'de doğu, gövdede v/p/r ile yatış/baş hareketi sıfır kabul edilir.
idx = p.longitudinalIndex;
u = state(idx.u);
w = state(idx.w);
pitchRate = state(idx.q);
pitch = state(idx.theta);
altitude = -state(idx.down);

% R_NB: NED kuzey-aşağı vektörünü gövde ileri-aşağı eksenine taşır.
C_BN = [cos(pitch),-sin(pitch);sin(pitch),cos(pitch)];
windBody = C_BN*cfg.windNED([1 3]);
airVelocityBody = [u;w]-windBody;

if isa(cfg.controls,'function_handle')
    controls = cfg.controls(time,state);
else
    controls = cfg.controls;
end
controls = controls(:);
if numel(controls) ~= 2 || any(~isfinite(controls))
    error('F16Dynamics3DOF:controls', ...
        'Controls must be [elevator;throttle].');
end
throttle = min(max(controls(2),0),1);
[surfaceForAero,surfaceDot] = F16Actuators( ...
    [controls(1);0;0],[state(idx.elevatorDeg);0;0],p);
[~,~,density,soundSpeed] = AtmosphereModel(max(0,altitude));
[aeroForce,aeroMoment,~,~,~,~,mach] = AeroModel( ...
    [airVelocityBody(1);0;airVelocityBody(2)], ...
    [0;pitchRate;0],density,p.Sref,p.span,p.chord, ...
    surfaceForAero(1),0,0,soundSpeed,p.Xref,p.Xcg);
[thrustForce,thrustMoment,powerDot] = F16ThrustRK4( ...
    throttle,state(idx.enginePower),altitude,mach,p);

% Kitap eqt:otelemeDinamigi3DOF: u_dot=F_x/m-q*w,
% w_dot=F_z/m+q*u. F içine gövdeye çevrilmiş ağırlık dahildir.
gravityBody = C_BN*[0;p.gFlat];
forceBody = aeroForce([1 3])+thrustForce([1 3]) ...
    +p.mass*gravityBody;
uDot = forceBody(1)/p.mass-pitchRate*w;
wDot = forceBody(2)/p.mass+pitchRate*u;
% Kitap eqt:donmeDinamigi3DOF ve eqt:donmeKinematigi3DOF.
pitchRateDot = (aeroMoment(2)+thrustMoment(2))/p.inertia(2,2);
pitchDot = pitchRate;
% Kitap eqt:otelemeKinematigi3DOF: gövde yer hızı NED'e taşınır.
positionDot = C_BN'*[u;w];
derivative = [uDot;wDot;pitchRateDot;positionDot;pitchDot; ...
    powerDot;surfaceDot(1)];
end
