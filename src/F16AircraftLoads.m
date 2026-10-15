function [forceBody,momentBody,powerDot,surfaceDot] = ...
    F16AircraftLoads(time,state,cfg,p,velocityBody,omegaAero,windBody,altitude)
%F16AIRCRAFTLOADS İki 6 DOF Dünya modeli için ortak F-16 alt modelleri.
% Hızlar, açısal hız ve kuvvetler gövde ekseninde; irtifa metredir.
% omegaAero, aerodinamik sönüm için Dünya'ya göre açısal hızdır.
idx = p.stateIndex;
if isa(cfg.controls,'function_handle')
    controls = cfg.controls(time,state);
else
    controls = cfg.controls;
end
controls = controls(:);
if numel(controls) ~= 4 || any(~isfinite(controls))
    error('F16Dynamics:controls', ...
        'Controls must be [elevator;aileron;rudder;throttle].');
end
controls(4) = min(max(controls(4),0),1);

[surfaceForAero,surfaceDot] = F16Actuators( ...
    controls(1:3),state(idx.surfaceDeg),p);
airVelocityBody = velocityBody-windBody;
[~,~,density,soundSpeed] = AtmosphereModel(max(0,altitude));
[aeroForce,aeroMoment,~,~,~,~,mach] = AeroModel( ...
    airVelocityBody,omegaAero,density,p.Sref,p.span,p.chord, ...
    surfaceForAero(1),surfaceForAero(2),surfaceForAero(3), ...
    soundSpeed,p.Xref,p.Xcg);
[thrustForce,thrustMoment,powerDot] = F16ThrustRK4( ...
    controls(4),state(idx.enginePower),altitude,mach,p);
forceBody = aeroForce+thrustForce;
momentBody = aeroMoment+thrustMoment;
end
