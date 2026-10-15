function p = F16Parameters()
%F16PARAMETERS F-16 modelinin SI birimlerindeki sabit parametreleri.
% 982 slug-ft^2 ürün ataleti Jxz'dir; atalet tensörüne -Jxz girer.
% Kütle ve atalet sabittir; yakıt tüketimine bağlı ağırlık değişimi yoktur.
% Kitaptaki durum denklemlerini kodda izlemeyi kolaylaştıran dizin haritası.
% Hız/konum seçilen Dünya modeline bağlıdır; diğer bileşenlerin sırası sabit.
p.stateIndex.velocity = 1:3;
p.stateIndex.omegaBI = 4:6;
p.stateIndex.position = 7:9;
p.stateIndex.quaternion = 10:13;
p.stateIndex.enginePower = 14;
p.stateIndex.surfaceDeg = 15:17;
% Kitabın boylamsal 3 DOF modeli için ayrı sekiz bileşenli durum.
p.longitudinalIndex.u = 1;
p.longitudinalIndex.w = 2;
p.longitudinalIndex.q = 3;
p.longitudinalIndex.north = 4;
p.longitudinalIndex.down = 5;
p.longitudinalIndex.theta = 6;
p.longitudinalIndex.enginePower = 7;
p.longitudinalIndex.elevatorDeg = 8;

p.mass = 20500 * 0.45359237;       % lbm -> kg
p.Sref = 27.871;                   % m^2
p.span = 9.144;                    % m
p.chord = 3.450336;               % m
p.inertia = diag([9496 55814 63100] * 1.35581795);
p.inertia(1,3) = -982 * 1.35581795;
p.inertia(3,1) = p.inertia(1,3);

% Katsayı referansı Stevens'ın %35 ortalama aerodinamik kord noktasıdır.
% [0;0;0] uçağın burnu değil bu yerel referans/nominal CG noktasıdır.
p.Xref = [0;0;0];                  % m, gövde eksenindeki katsayı referansı
p.Xcg = [0;0;0];
p.Xeng = [0;0;0];

% Stevens/Lewis/Johnson Ek A.5: rotor momentumu +gövde X yönünde sabit.
p.engineAngularMomentumBody = [160 * 1.35581795;0;0]; % N*m*s

% Sıra [elevatör;kanatçık;dümen]; Stevens Ek A.4 konum/hız/gecikme verisi.
p.actuatorLimitDeg = [25;21.5;30]; % elevatör, kanatçık, dümen
p.actuatorRateDegS = [60;80;120];   % derece/s
p.actuatorTauS = 0.0495;           % s

% WGS-84 geometri/dönüş sabitleri ve düz Dünya için sabit yerçekimi.
p.omegaEarth = 7.2921150e-5;      % rad/s, WGS-84
p.aEarth = 6378137.0;             % m, WGS-84 büyük yarıçap
p.flattening = 1/298.257223563;  % WGS-84
p.gFlat = 9.80665;                % m/s^2

end
