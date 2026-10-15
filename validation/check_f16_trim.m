function trim=check_f16_trim(V,h,cg,model,turnRate,location)
%CHECK_F16_TRIM Seçilen üretim modeliyle yatay uçuş/dönüş dengesi.
% Dönen Dünya trimi belirtilen konumda anlık dengedir.
if nargin<1,V=150;end
if nargin<2,h=5000;end
if nargin<3,cg=.35;end
if nargin<4,model='flat';end
if nargin<5,turnRate=0;end
if nargin<6,location=[41;29];end
assert(~strcmp(model,'3dof') || turnRate==0,'3 DOF cannot turn.');
p=F16Parameters();p.frames=F16Frames();p.Xcg=[(.35-cg)*p.chord;0;0];
simple=turnRate==0 && ~strcmp(model,'rotating');
if simple
    z=[5*pi/180;-2;.2];
    options=optimset('Display','off','TolX',1e-12,'TolFun',1e-16, ...
        'MaxFunEvals',3000,'MaxIter',2000);
    z=fminsearch(@cost,z,options);
else
    if turnRate==0
        seed=check_f16_trim(V,h,cg,'flat');
        z=[deg2rad(seed.angleOfAttackDeg);0;0;deg2rad(seed.angleOfAttackDeg); ...
            seed.throttle;seed.elevatorDeg;0;0];
    else
        b=stevens_example_362();
        z=[b.stateBook(2:5);b.controlBook];
    end
    steps=[repmat(1e-6,4,1);1e-5;repmat(1e-4,3,1)];
    for iteration=1:25
        r=residual(z);if norm(r)<1e-10,break;end
        J=zeros(8);
        for j=1:8
            plus=z;minus=z;plus(j)=plus(j)+steps(j);minus(j)=minus(j)-steps(j);
            J(:,j)=(residual(plus)-residual(minus))/(2*steps(j));
        end
        delta=-J\r;scale=1;
        while scale>1/4096 && norm(residual(z+scale*delta))>=norm(r)
            scale=scale/2;
        end
        z=z+scale*delta;
    end
end
[r,x,u,info]=residual(z);
trim=struct('model',model,'state',x,'controls',u,'locationDeg',location, ...
    'angleOfAttackDeg',rad2deg(x(2)),'elevatorDeg',u(1),'throttle',u(4), ...
    'enginePower',x(13),'altitudeM',h,'airspeedMps',V,'cgFraction',cg, ...
    'turnRateRadS',turnRate,'accelerationResidual',r,'mach',info.mach);
trim.levelVelocityBody=V*[cos(x(2))*cos(x(3));sin(x(3));sin(x(2))*cos(x(3))];
trim.initialEulerDeg=rad2deg(x(4:6));
trim.initialOmegaBody=info.earthRatesBody; % 6 DOF başlatıcılarının giriş sözleşmesi: Dünya'ya göre
dd=f16_validation_dynamics(x,u,model,p,location);
trim.enginePowerDerivative=dd(13);
trim.valid=norm(r)<1e-7 && abs(dd(13))<1e-7 && ...
    u(4)>=0 && u(4)<=1 && rad2deg(x(2))>=-10 && rad2deg(x(2))<=45 && ...
    abs(rad2deg(x(3)))<=30 && all(abs(u(1:3))<=[24;21.5;30]);

    function value=cost(z)
        r=residual(z);
        penalty=max(0,-10-rad2deg(z(1)))^2+max(0,rad2deg(z(1))-45)^2+ ...
            max(0,abs(z(2))-24)^2+100*max(0,-z(3))^2+100*max(0,z(3)-1)^2;
        value=r'*r+penalty;
    end
    function [r,x,u,info]=residual(z)
        if simple
            x=[V;z(1);0;0;z(1);0;zeros(5,1);h;f16Power(z(3))];
            u=[z(2);0;0;z(3)];
        else
            u=[z(6:8);z(5)];
            x=[V;z(1:4);0;zeros(5,1);h;f16Power(z(5))];
            C=p.frames.quatToDCM(p.frames.eulerToQuat(z(3),z(4),0));
            vb=V*[cos(z(1))*cos(z(2));sin(z(2));sin(z(1))*cos(z(2))];
            vn=C*vb;x(6)=-atan2(vn(2),vn(1));
            x(7:9)=turnRate*[-sin(z(4));sin(z(3))*cos(z(4));cos(z(3))*cos(z(4))];
        end
        [d,info]=f16_validation_dynamics(x,u,model,p,location);
        if simple
            r=[info.bodyAcceleration([1 3]);100*d(8)];
        else
            r=[info.bodyAcceleration;100*d(7:9);d(12);info.sideAcceleration];
        end
    end
end

function power=f16Power(throttle)
if throttle<=.77,power=64.94*throttle;else,power=217.38*throttle-117.38;end
end
