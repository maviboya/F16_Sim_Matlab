function derivative = F16DynamicsFlat(time,state,cfg,p)
%F16DYNAMICSFLAT Düz, dönmeyen NED Dünya için 6 DOF denklemleri.
% Kitap: düz Dünya öteleme/dönme ve kuaterniyon kinematiği.
% Durum: [v_g^B(3); omega_BI^B(3); p^N(3); q_NB(4); güç; yüzeyler(3)].
idx = p.stateIndex;
frames = p.frames;
qNB = state(idx.quaternion)/norm(state(idx.quaternion));
C_NB = frames.quatToDCM(qNB);
velocityBody = state(idx.velocity);
omegaBI = state(idx.omegaBI);
altitude = -state(idx.position(3));
windBody = C_NB'*cfg.windNED;

[forceBody,momentBody,powerDot,surfaceDot] = F16AircraftLoads( ...
    time,state,cfg,p,velocityBody,omegaBI,windBody,altitude);

% Newton: d(v_g^B)/dt = F^B/m + C_BN g^N - omega_BI^B × v_g^B.
velocityDot = forceBody/p.mass + C_NB'*[0;0;p.gFlat] ...
    -cross(omegaBI,velocityBody);
% Euler: motor rotorunun sabit açısal momentumu da hesaba katılır.
omegaDot = p.inertia \ (momentBody-cross(omegaBI, ...
    p.inertia*omegaBI+p.engineAngularMomentumBody));
positionDot = C_NB*velocityBody;
qDot = 0.5*frames.quatMultiply(qNB,[0;omegaBI]);
derivative = [velocityDot;omegaDot;positionDot;qDot;powerDot;surfaceDot];
end
