function result=compare_stevens_trim()
%COMPARE_STEVENS_TRIM Üç üretim modeli / Stevens Tablo 3.6-2 ve 3.6-3.
result.models={'flat','rotating','3dof'};
result.labels={'Düz Dünya','Dönen Dünya','3-DOF'};
result.speedFps=[130;140;150;170;200;260;300;350;400;440;500;540;600;640;700;800];
% Sütunlar gaz, alfa (derece), elevatör (derece).
result.reference=[ ...
 .816 45.6 20.1;.736 40.3 -1.36;.619 34.6 .173;.464 27.2 .621; ...
 .287 19.7 .723;.148 11.6 -.090;.122 8.49 -.591;.107 5.87 -.539; ...
 .108 4.16 -.591;.113 3.19 -.671;.137 2.14 -.756;.160 1.63 -.798; ...
 .200 1.04 -.846;.230 .742 -.871;.282 .382 -.900;.378 -.045 -.943];
result.cgFraction=[.35;.30;.38];
result.cgReference=[.1385 .03691*180/pi -.7588; ...
    .1485 .03936*180/pi -1.931;.1325 .03544*180/pi -.05590];
result.values=nan(16,3,3);result.valid=false(16,3);result.cgValues=nan(3,3,3);
result.residual=nan(16,3);result.trims=cell(16,3);result.cgTrims=cell(3,3);
for m=1:3
    for k=1:16
        tr=check_f16_trim(result.speedFps(k)*.3048,0,.35,result.models{m});
        result.trims{k,m}=tr;result.values(k,:,m)=[tr.throttle,tr.angleOfAttackDeg,tr.elevatorDeg];
        result.valid(k,m)=tr.valid;result.residual(k,m)=norm(tr.accelerationResidual);
    end
    for k=1:3
        tr=check_f16_trim(502*.3048,0,result.cgFraction(k),result.models{m});
        assert(tr.valid,'CG trim failed: %s',result.models{m});
        result.cgTrims{k,m}=tr;result.cgValues(k,:,m)=[tr.throttle,tr.angleOfAttackDeg,tr.elevatorDeg];
    end
end
result.error=result.values-result.reference;
result.cgError=result.cgValues-result.cgReference;
maxError=nan(3,3);
for m=1:3,maxError(m,:)=max(abs(result.error(result.valid(:,m),:,m)),[],1);end
result.summary=table(string(result.labels'),sum(result.valid,1)',maxError(:,1),maxError(:,2),maxError(:,3), ...
    'VariableNames',{'model','validRows','maxThrottleError','maxAlphaErrorDeg','maxElevatorErrorDeg'});
disp(result.summary);
end
