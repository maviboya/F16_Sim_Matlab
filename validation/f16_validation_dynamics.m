function [d,info] = f16_validation_dynamics(x,controls,model,p,location)
%F16_VALIDATION_DYNAMICS Üretim denklemlerini ortak Stevens koordinatlarında çağır.
% x=[V(m/s),alpha,beta,phi,theta,psi,p_BN,q_BN,r_BN,N,E,h(m),power].
% Dönen Dünya: N/E başlangıç yarıçaplarıyla ölçekli enlem/boylamdır.
% Ortak açısal hız yerel NED'e göredir. Üretim durumu eylemsiz hızı saklar.
if nargin<5,location=[41;29];end
f=p.frames; V=x(1); a=x(2); b=x(3); phi=x(4); theta=x(5);
v=V*[cos(a)*cos(b);sin(b);sin(a)*cos(b)];
q=f.eulerToQuat(phi,theta,x(6)); C=f.quatToDCM(q);
cfg=struct('windNED',zeros(3,1),'controls',controls);
info.transportBody=zeros(3,1);
omegaBE=x(7:9);
if strcmp(model,'3dof')
    cfg.controls=controls([1 4]);
    y=[v(1);v(3);x(8);x(10);-x(12);theta;x(13);controls(1)];
    dy=F16Dynamics3DOF(0,y,cfg,p);
    vd=[dy(1);0;dy(2)]; wd=[0;dy(3);0];
    ed=[0;dy(6);0]; pos=[dy(4);0;-dy(5)]; pd=dy(7);
elseif strcmp(model,'flat')
    y=[v;x(7:9);x(10:11);-x(12);q;x(13);controls(1:3)];
    dy=F16DynamicsFlat(0,y,cfg,p);
    vd=dy(1:3);wd=dy(4:6);
    ed=eulerRate(x(7:9),phi,theta);pos=[dy(7:8);-dy(9)];pd=dy(14);
elseif strcmp(model,'rotating')
    lat0=deg2rad(location(1));lon0=deg2rad(location(2));
    e2=p.flattening*(2-p.flattening);
    n0=p.aEarth/sqrt(1-e2*sin(lat0)^2);
    m0=p.aEarth*(1-e2)/(1-e2*sin(lat0)^2)^1.5;
    lat=lat0+x(10)/m0;lon=lon0+x(11)/(n0*cos(lat0));h=x(12);
    n=p.aEarth/sqrt(1-e2*sin(lat)^2);
    m=p.aEarth*(1-e2)/(1-e2*sin(lat)^2)^1.5;
    EN=f.nedToECEF(lat,lon); EB=EN*C; vn=C*v;
    latDot=vn(1)/(m+h);lonDot=vn(2)/((n+h)*cos(lat));
    transport=[lonDot*cos(lat);-latDot;-lonDot*sin(lat)];
    info.transportBody=C'*transport;
    omegaBE=x(7:9)+info.transportBody;
    earthBody=EB'*[0;0;p.omegaEarth];
    y=[EB*v;omegaBE+earthBody;f.geodeticToECEF(lat,lon,h,p); ...
        f.dcmToQuat(EB);x(13);controls(1:3)];
    dy=F16DynamicsRotating(0,y,cfg,p);
    vd=EB'*dy(1:3)-cross(omegaBE,v);
    % omega_BN = omega_BI - C_BN*(omega_IE^N + omega_NE^N).
    % NED taşıma hızının büyüklüğü ve yönü uçuş boyunca değişir;
    % yalnız Dünya dönüşünü çıkarmak açısal ivme triminde yeterli değildir.
    vnDot=C*(vd+cross(x(7:9),v));hDot=-vn(3);
    curvatureRate=e2*sin(lat)*cos(lat)/(1-e2*sin(lat)^2)*latDot;
    nDot=n*curvatureRate;mDot=3*m*curvatureRate;
    transportDot=[vnDot(2)/(n+h)-vn(2)*(nDot+hDot)/(n+h)^2; ...
        -vnDot(1)/(m+h)+vn(1)*(mDot+hDot)/(m+h)^2;0];
    transportDot(3)=-transportDot(1)*tan(lat)-transport(1)*latDot/cos(lat)^2;
    earthDot=p.omegaEarth*latDot*[-sin(lat);0;-cos(lat)];
    frameRateBody=earthBody+info.transportBody;
    wd=dy(4:6)+cross(x(7:9),frameRateBody)-C'*(earthDot+transportDot);
    ed=eulerRate(x(7:9),phi,theta);
    pos=[m0*latDot;n0*cos(lat0)*lonDot;-vn(3)];pd=dy(14);
else
    error('f16_validation_dynamics:model','Unknown model: %s',model);
end
vdot=dot(v,vd)/V;
adot=(v(1)*vd(3)-v(3)*vd(1))/(v(1)^2+v(3)^2);
bdot=(vd(2)-sin(b)*vdot)/(V*cos(b));
d=[vdot;adot;bdot;ed;wd;pos;pd];
info.state=y;info.derivative=dy;info.bodyAcceleration=vd;
info.angularAcceleration=wd;
info.earthRatesBody=omegaBE;
[~,~,rho,soundSpeed]=AtmosphereModel(max(0,x(12)));
[force,~]=AeroModel(v,omegaBE,rho,p.Sref,p.span,p.chord, ...
    controls(1),controls(2),controls(3),soundSpeed,p.Xref,p.Xcg);
info.sideAcceleration=force(2)/p.mass;info.mach=V/soundSpeed;
end

function d=eulerRate(w,phi,theta)
d=[w(1)+tan(theta)*(w(2)*sin(phi)+w(3)*cos(phi)); ...
    w(2)*cos(phi)-w(3)*sin(phi); ...
    (w(2)*sin(phi)+w(3)*cos(phi))/cos(theta)];
end
