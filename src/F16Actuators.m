function [actualForAero, surfaceDot] = F16Actuators(commandDeg,actualDeg,p)
%F16ACTUATORS Stevens Ek A.4 birinci derece kontrol yüzeyi eyleyicileri.
% Sıra: [elevatör; kanatçık; dümen]; açılar derece cinsindendir.
% Komut önce fiziksel açıya kısılır; gerçek açı durumunun türevi
% d(delta)/dt = sınırla((delta_komut-delta)/tau, ±hız_limiti) biçimindedir.
% Stevens verisinde ölü bölge yoktur.
commandDeg = min(max(commandDeg(:),-p.actuatorLimitDeg),p.actuatorLimitDeg);
actualDeg = actualDeg(:);
surfaceDot = min(max((commandDeg-actualDeg)/p.actuatorTauS, ...
    -p.actuatorRateDegS),p.actuatorRateDegS);
% RK4 ara durumunda sınır aşılırsa aerodinamik sorguyu fiziksel aralıkta tut.
actualForAero = min(max(actualDeg,-p.actuatorLimitDeg),p.actuatorLimitDeg);
% Elevatör katsayı tablosu ±24°'de biter; diğer yüzeyler tam açıyla girer.
actualForAero(1) = min(max(actualForAero(1),-24),24);
end
