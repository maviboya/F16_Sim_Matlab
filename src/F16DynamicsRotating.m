function derivative = F16DynamicsRotating(time,state,cfg,p)
%F16DYNAMICSROTATING Dönen WGS-84 ECEF Dünya için 6 DOF denklemleri.
% Kitap: ECEF öteleme, gövdede eylemsiz açısal momentum ve q_EB kinematiği.
% Durum: [v_g^E(3); omega_BI^B(3); p^E(3); q_EB(4); güç; yüzeyler(3)].
idx = p.stateIndex;
frames = p.frames;
positionECEF = state(idx.position);
velocityECEF = state(idx.velocity);
omegaBI = state(idx.omegaBI);
qEB = state(idx.quaternion)/norm(state(idx.quaternion));
C_EB = frames.quatToDCM(qEB);
[latitude,longitude,altitude] = frames.ecefToGeodetic(positionECEF,p);
C_EN = frames.nedToECEF(latitude,longitude);
omegaEarthECEF = [0;0;p.omegaEarth];
omegaBE = omegaBI-C_EB'*omegaEarthECEF;
velocityBody = C_EB'*velocityECEF;
windBody = C_EB'*C_EN*cfg.windNED;

[forceBody,momentBody,powerDot,surfaceDot] = F16AircraftLoads( ...
    time,state,cfg,p,velocityBody,omegaBE,windBody,altitude);

% Normal ağırlık, merkezkaç katkısını içerir. ECEF ivmesine yalnız
% Coriolis terimini ayrıca koyarak merkezkaç etkisini iki kez saymayız.
eccentricitySquared = p.flattening*(2-p.flattening);
surfaceGravity = 9.7803253359*(1+0.00193185265241*sin(latitude)^2) / ...
    sqrt(1-eccentricitySquared*sin(latitude)^2);
normalGravityECEF = C_EN*[0;0;surfaceGravity* ...
    (p.aEarth/(p.aEarth+altitude))^2];
velocityDot = C_EB*forceBody/p.mass + normalGravityECEF ...
    -2*cross(omegaEarthECEF,velocityECEF);
% Açısal momentum eylemsiz hıza; q_EB kinematiği Dünya'ya göre hıza bağlıdır.
omegaDot = p.inertia \ (momentBody-cross(omegaBI, ...
    p.inertia*omegaBI+p.engineAngularMomentumBody));
positionDot = velocityECEF;
qDot = 0.5*frames.quatMultiply(qEB,[0;omegaBE]);
derivative = [velocityDot;omegaDot;positionDot;qDot;powerDot;surfaceDot];
end
