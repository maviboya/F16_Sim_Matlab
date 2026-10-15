function result=compare_stevens_modes()
%COMPARE_STEVENS_MODES Üç model / Stevens 3.8-1/2 ve Tablo 4.2-3.
result.models={'flat','rotating','3dof'};result.labels={'Düz Dünya','Dönen Dünya','3-DOF'};
result.speedFps=[900;600;900;367];result.altitudeFt=[50000;50000;0;0];
% Dutch periyot/sönüm, spiral/roll zaman sabiti, kısa periyot/sönüm, fugoid periyot/sönüm.
result.reference=[2.365 .06480 179.2 1.050 4.507 .2615 102.1 .005453; ...
    2.735 .07722 138.7 2.230 NaN NaN NaN NaN; ...
    1.143 .1272 122.1 .1487 2.372 .8175 183.4 .3242; ...
    2.396 .1470 73.52 .4160 4.023 .5735 56.93 .06240];
result.values=nan(4,8,3);result.valid=false(4,3);result.mach=nan(4,3);
result.throttle=nan(4,3);result.trims=cell(4,3);result.linearizations=cell(4,3);
for m=1:3
    for k=1:4
        tr=check_f16_trim(result.speedFps(k)*.3048,result.altitudeFt(k)*.3048,.30,result.models{m});
        result.trims{k,m}=tr;result.valid(k,m)=tr.valid;
        result.throttle(k,m)=tr.throttle;result.mach(k,m)=tr.mach;
        if ~tr.valid,continue;end
        lin=linearize_f16_full(tr);result.linearizations{k,m}=lin;
        % Tam sistemin modlarını boylamsal katılımıyla ayır. 3-DOF'ta yalnız iki çift var.
        [W,D]=eig(lin.jacobian(lin.activeStates,lin.activeStates));poles=diag(D);
        pair=find(imag(poles)>1e-5);
        if m<3
            % Uçak modlarında beta/phi/p/r baskın olan çift Dutch roll'dur.
            lateral=ismember(lin.activeStates,[3 4 7 9]);
            longitudinal=ismember(lin.activeStates,[2 5 8]);
            ratio=sum(abs(W(lateral,pair)).^2,1)./max(sum(abs(W(longitudinal,pair)).^2,1),eps);
            [~,id]=max(ratio);dutch=poles(pair(id));pair(id)=[];
            % Reel uçak modlarını da tam sistemden seç; motor ve irtifa
            % kutuplarını roll/spiral diye etiketleme.
            realIndex=find(abs(imag(poles))<1e-7 & abs(real(poles))>1e-6);
            realRatio=sum(abs(W(lateral,realIndex)).^2,1)./ ...
                max(sum(abs(W(longitudinal,realIndex)).^2,1),eps);
            [~,realOrder]=sort(realRatio,'descend');
            assert(all(realRatio(realOrder(1:2))>1),'Lateral real modes are ambiguous.');
            realLat=sort(real(poles(realIndex(realOrder(1:2)))));
            result.values(k,1:4,m)=[2*pi/abs(dutch),-real(dutch)/abs(dutch),-1/realLat(end),-1/realLat(1)];
        end
        [~,order]=sort(abs(poles(pair)));pair=pair(order);
        assert(numel(pair)>=2,'Missing longitudinal modes.');
        ph=poles(pair(1));sp=poles(pair(2));
        result.values(k,5:8,m)=[2*pi/abs(sp),-real(sp)/abs(sp),2*pi/abs(ph),-real(ph)/abs(ph)];
    end
end
result.error=result.values-result.reference;
% Örnek 3.8: motor gücü ve irtifa sabitken dört durumlu matrisler.
result.example.longReference=[-.020244 7.8763 -32.170 -.65020; ...
    -.00025372 -1.0190 0 .90484;0 0 0 1;7.9472e-11 -2.4982 0 -1.3861];
result.example.latReference=[-.32200 .064032 .038904 -.99156; ...
    0 0 1 .039385;-30.919 0 -3.6730 .67425;9.4742 0 -.026358 -.49849];
for m=1:3
    tr=check_f16_trim(502*.3048,0,.30,result.models{m});lin=linearize_f16_full(tr);
    S=diag([.3048 1 1 1]);long=lin.jacobian([1 2 5 8],[1 2 5 8]);
    result.example.longModel(:,:,m)=S\(long*S);
    result.example.longEigenvalues(:,m)=eig(result.example.longModel(:,:,m));
    if m<3
        result.example.latModel(:,:,m)=lin.jacobian([3 4 7 9],[3 4 7 9]);
        result.example.latEigenvalues(:,m)=eig(result.example.latModel(:,:,m));
    end
end
% Tablo 3.5-2: yayımlı asimetrik durum sadece 6-DOF modellerine uygulanabilir.
x=[500;.5;-.2;-1;1;-1;.7;-.8;.9;1000;900;10000;90];
u=[20;-15;-20;.9];scale=ones(13,1);scale([1 10 11 12])=.3048;
result.derivatives.reference=[-75.23724;-.8813491;-.4759990;2.505734;.3250820;2.145926; ...
    12.62679;.9649671;.5809759;342.4439;-266.7707;248.1241;-58.68999];
p=F16Parameters();p.frames=F16Frames();p.Xcg=[(.35-.4)*p.chord;0;0];
for m=1:2
    result.derivatives.values(:,m)=f16_validation_dynamics(x.*scale,u,result.models{m},p)./scale;
end
result.derivatives.error=result.derivatives.values-result.derivatives.reference;
fprintf('900 ft/s, sea-level phugoid periods [flat rotating 3dof]: %.6f %.6f %.6f s\n',result.values(3,7,:));
end
