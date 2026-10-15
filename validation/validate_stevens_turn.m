function result=validate_stevens_turn()
%VALIDATE_STEVENS_TURN Stevens'in dönüşü / düz ve dönen Dünya üretim modelleri.
% Her model kendi triminden sabit kumandalarla başlar. Geri besleme yoktur.
% Kaynak 180 s; zarf dışına çıkan iz kesilir, kalan noktalar NaN bırakılır.
book=stevens_example_362();result.source=book.groundTrackFt;
result.models={'flat','rotating'};result.labels={'Düz Dünya','Dönen Dünya'};
result.referenceTrim=[book.stateBook;book.controlBook];
result.trimValues=nan(17,2);result.trims=cell(1,2);result.runs=cell(1,2);
result.sampleErrorFt=nan(19,2);result.sourceStateResidual=zeros(6,2);
result.unstablePole=zeros(1,2);result.stepCheckMaxPositionFt=zeros(1,2);
result.stepCheckUntilS=zeros(1,2);
result.fineEndTimeS=zeros(1,2);result.stepConverged=false(1,2);
for m=1:2
    tr=check_f16_trim(502*.3048,0,book.cgFraction,result.models{m},.3);
    assert(tr.valid,'Turn trim failed: %s',result.models{m});result.trims{m}=tr;
    scale=ones(13,1);scale([1 10 11 12])=.3048;
    result.trimValues(:,m)=[tr.state./scale;tr.controls([4 1 2 3])];
    p=F16Parameters();p.frames=F16Frames();p.Xcg=[(.35-book.cgFraction)*p.chord;0;0];
    [~,info]=f16_validation_dynamics(book.stateBook.*scale,book.controls,tr.model,p,tr.locationDeg);
    result.sourceStateResidual(:,m)=[info.bodyAcceleration;info.angularAcceleration];
    lin=linearize_f16_full(tr);
    reduced=[1:5 7:9 12:13]; % konum ve döngüsel baş açısı dışarıda
    result.unstablePole(m)=max(real(eig(lin.jacobian(reduced,reduced))));
    fprintf('Turn: %s, RK4 0.01 / 0.005 s...\n',tr.model);
    result.runs{m}=integrate(tr,p,180,.01);
    fine=integrate(tr,p,180,.005);
    coarse=result.runs{m}.history;
    [common,ic,jc]=intersect(coarse.timeS,fine.history.timeS);
    result.stepCheckMaxPositionFt(m)=max(vecnorm(coarse{ic,2:4}-fine.history{jc,2:4},2,2));
    result.stepCheckUntilS(m)=max(common);
    result.fineEndTimeS(m)=fine.endTimeS;
    result.stepConverged(m)=result.stepCheckMaxPositionFt(m)<.05 && ...
        abs(result.runs{m}.endTimeS-fine.endTimeS)<=.02;
    h=result.runs{m}.history;
    [present,index]=ismember(book.groundTrackFt(:,1),h.timeS);
    result.sampleErrorFt(present,m)=vecnorm(h{index(present),2:3}-book.groundTrackFt(present,2:3),2,2);
end
result.trimError=result.trimValues-result.referenceTrim;
% Baş açısı değişirken aynı anlık trimin artıklarının nasıl değiştiğini ölç.
% Bu bir ek yörünge/model eğrisi değildir: aynı konumda dört RHS sorgusudur.
result.frameAudit=headingAudit(result.trims{2},p);
fprintf('Turn end times [flat rotating]: %.2f %.2f s\n',result.runs{1}.endTimeS,result.runs{2}.endTimeS);
fprintf('RK4 dt/2 maximum position difference: %.6g %.6g ft\n',result.stepCheckMaxPositionFt);
fprintf('Step convergence [flat rotating]: %d %d\n',result.stepConverged);
end

function audit=headingAudit(tr,p)
audit.headingOffsetDeg=[0 90 180 270];
audit.bodyAcceleration=zeros(3,4);noSpin=zeros(3,4);
withoutRotation=p;withoutRotation.omegaEarth=0;
for k=1:4
    x=tr.state;x(6)=x(6)+deg2rad(audit.headingOffsetDeg(k));
    [~,full]=f16_validation_dynamics(x,tr.controls,'rotating',p,tr.locationDeg);
    [~,zero]=f16_validation_dynamics(x,tr.controls,'rotating',withoutRotation,tr.locationDeg);
    audit.bodyAcceleration(:,k)=full.bodyAcceleration;
    noSpin(:,k)=zero.bodyAcceleration;
end
audit.changeFromInitial=audit.bodyAcceleration-audit.bodyAcceleration(:,1);
audit.changeWithoutEarthRotation=noSpin-noSpin(:,1);
audit.earthRotationContribution=audit.changeFromInitial-audit.changeWithoutEarthRotation;
end

function run=integrate(tr,p,duration,dt)
[~,initial]=f16_validation_dynamics(tr.state,tr.controls,tr.model,p,tr.locationDeg);
y=initial.state;cfg=struct('windNED',zeros(3,1),'controls',tr.controls);
if strcmp(tr.model,'flat'),fun=@F16DynamicsFlat;else,fun=@F16DynamicsRotating;end
EN0=p.frames.nedToECEF(deg2rad(tr.locationDeg(1)),deg2rad(tr.locationDeg(2)));
pos0=p.frames.geodeticToECEF(deg2rad(tr.locationDeg(1)),deg2rad(tr.locationDeg(2)),0,p);
samplePeriod=.1;
data=nan(round(duration/samplePeriod)+2,7);count=1;data(count,:)=[0 observe(y)];
reason='Hedef süre tamamlandı';
for k=1:round(duration/dt)
    t=(k-1)*dt;
    a=fun(t,y,cfg,p);b=fun(t+dt/2,y+dt*a/2,cfg,p);
    c=fun(t+dt/2,y+dt*b/2,cfg,p);d=fun(t+dt,y+dt*c,cfg,p);
    y=y+dt*(a+2*b+2*c+d)/6;y(10:13)=y(10:13)/norm(y(10:13));
    assert(all(isfinite(y)),'Non-finite turn state.');
    v=observe(y);time=round(k*dt,8);
    out=v(5)<-10 || v(5)>45 || abs(v(6))>30;
    below=v(3)<-1/.3048; % 1 m tolerans: deniz seviyesi başlangıcındaki sayısal gürültü
    if mod(k,round(samplePeriod/dt))==0 || out || below
        count=count+1;data(count,:)=[time v];
    end
    if out,reason='Aerodinamik tablo sınırı';break;end
    if below,reason='Deniz seviyesinin 1 m altına indi';break;end
end
run.history=array2table(data(1:count,:),'VariableNames', ...
    {'timeS','northFt','eastFt','altitudeFt','speedFps','alphaDeg','betaDeg'});
run.endTimeS=time;run.stopReason=reason;run.completed=time>=duration-1e-9;
run.finalState=y;
    function values=observe(s)
        if strcmp(tr.model,'flat')
            ne=s(7:8);h=-s(9);vb=s(1:3);
        else
            ne3=EN0'*(s(7:9)-pos0);ne=ne3(1:2);
            [~,~,h]=p.frames.ecefToGeodetic(s(7:9),p);
            vb=p.frames.quatToDCM(s(10:13))'*s(1:3);
        end
        speed=norm(vb);alpha=atan2d(vb(3),vb(1));beta=asind(vb(2)/speed);
        values=[ne'/.3048 h/.3048 speed/.3048 alpha beta];
    end
end
