function result=linearize_f16_full(trim,differenceScale)
%LINEARIZE_F16_FULL Üretim modelleri; motor/irtifa dahil, yüzeyler sabit.
% Dönen Dünya için yerel yörünge çevresinde anlık Jacobian.
% Ortak açısal hızlar yerel NED'e göredir (omega_BN), ECEF'e göre değil.
if nargin<2,differenceScale=1;end
p=F16Parameters();p.frames=F16Frames();p.Xcg=[(.35-trim.cgFraction)*p.chord;0;0];
x=trim.state;
fun=@(z)f16_validation_dynamics(z,trim.controls,trim.model,p,trim.locationDeg);
step=differenceScale*[1e-3;repmat(1e-6,8,1);1;1;.01;.001];
A=zeros(13);
for j=1:13
    xp=x;xm=x;xp(j)=xp(j)+step(j);xm(j)=xm(j)-step(j);
    if j==12 && x(12)==0
        A(:,j)=(fun(xp)-fun(x))/step(j);
    elseif j==12 && abs(x(12)-15240)<1e-6
        A(:,j)=(fun(x)-fun(xm))/step(j);
    else
        A(:,j)=(fun(xp)-fun(xm))/(2*step(j));
    end
end
active=1:13;
if strcmp(trim.model,'3dof'),active=[1 2 5 8 10 12 13];end
result.jacobian=A;result.eigenvalues=eig(A(active,active));
result.activeStates=active;result.state=x;result.derivative=fun(x);
end
