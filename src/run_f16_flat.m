function result = run_f16_flat(options)
%RUN_F16_FLAT Düz NED Dünya için bağımsız 6 DOF simülasyonu.
%   result = run_f16_flat(struct('duration',10,'dt',0.01));
% Durum sırası, varsayımlar ve girişler README.md içindedir.
% options.controls sırası [elevatör; kanatçık; dümen; gaz] olup yüzey
% komutları derece, gaz komutu 0..1 aralığındadır.
if nargin < 1
    options = struct();
end
cfg = defaultOptions();
% Kullanıcının verdiği alanlar varsayılan seçeneklerin üzerine yazılır.
names = fieldnames(options);
for j = 1:numel(names)
    cfg.(names{j}) = options.(names{j});
end
cfg.earth = 'flat'; % Bu başlatıcının Dünya modeli sabittir.
cfg.clock = char(cfg.clock);
if ~ismember(cfg.clock,{'simulation','realtime'})
    error('run_f16_flat:clock','clock must be simulation or realtime.');
end
if ~isscalar(cfg.dt) || ~isfinite(cfg.dt) || cfg.dt <= 0 || ...
        ~isscalar(cfg.duration) || ~isfinite(cfg.duration) || cfg.duration < 0
    error('run_f16_flat:time','dt must be positive and duration nonnegative.');
end
if ~isscalar(cfg.cgFraction) || ~isfinite(cfg.cgFraction) || ...
        cfg.cgFraction < 0 || cfg.cgFraction > 1
    error('run_f16_flat:cg','cgFraction must be between 0 and 1.');
end
stepCount = round(cfg.duration/cfg.dt);
% Sabit adımlı RK4'te son adımı ayrıca küçültmemek için süre tam kat olmalı.
if abs(stepCount*cfg.dt-cfg.duration) > 1e-9*max(1,cfg.duration)
    error('run_f16_flat:time','duration must be an integer multiple of dt.');
end
if numel(cfg.initialVelocityBody) ~= 3 || numel(cfg.initialOmegaBody) ~= 3 || ...
        numel(cfg.initialEulerDeg) ~= 3 || ...
        numel(cfg.windNED) ~= 3 || numel(cfg.initialNED) ~= 3
    error('run_f16_flat:initial','Initial vectors and windNED must each have 3 elements.');
end
cfg.initialVelocityBody = cfg.initialVelocityBody(:);
cfg.initialOmegaBody = cfg.initialOmegaBody(:);
cfg.initialEulerDeg = cfg.initialEulerDeg(:);
cfg.initialNED = cfg.initialNED(:);
cfg.windNED = cfg.windNED(:);
p = F16Parameters();
p.frames = F16Frames();
p.Xcg = [(0.35-cfg.cgFraction)*p.chord;0;0];
f = p.frames;
idx = p.stateIndex;
eul = cfg.initialEulerDeg*pi/180;
qNB = f.eulerToQuat(eul(1),eul(2),eul(3));
% Hız gövdede, konum NED'de ve duruş gövdeden NED'e kuaterniyondur.
q0 = qNB;
pos0 = cfg.initialNED;
velocity0 = cfg.initialVelocityBody;
omega0 = cfg.initialOmegaBody;
initialState = [velocity0;omega0;pos0;q0;cfg.initialEnginePower;zeros(3,1)];
% Başlangıç yüzey açıları belirtilmezse komuta eşit başlatılır. Bu seçim
% sabit komutlu trim koşusunda yapay bir servo geçişi oluşmasını engeller.
if isempty(cfg.initialActuatorDeg)
    if isa(cfg.controls,'function_handle')
        initialControls = cfg.controls(0,initialState);
    else
        initialControls = cfg.controls;
    end
    initialControls = initialControls(:);
    if numel(initialControls) ~= 4 || any(~isfinite(initialControls))
        error('run_f16_flat:controls','Controls must have 4 finite elements.');
    end
    initialSurface = initialControls(1:3);
else
    initialSurface = cfg.initialActuatorDeg(:);
    if numel(initialSurface) ~= 3 || any(~isfinite(initialSurface))
        error('run_f16_flat:actuators','initialActuatorDeg must have 3 finite elements.');
    end
end
initialState(idx.surfaceDeg) = min(max(initialSurface, ...
    -p.actuatorLimitDeg),p.actuatorLimitDeg);
time = (0:stepCount)*cfg.dt;
state = zeros(numel(initialState),stepCount+1);
state(:,1) = initialState;
wall = tic;
lastSample = stepCount+1;
% Motor gücü ve üç servo da hava aracıyla aynı dört RK4 ara aşamasında çözülür.
for step = 1:stepCount
    state(:,step+1) = RK4(@F16DynamicsFlat,time(step),state(:,step),cfg.dt,cfg,p);
    % Kuaterniyonun birim normundan sayısal sapmasını her tam adımda gider.
    state(idx.quaternion,step+1) = state(idx.quaternion,step+1) / ...
        norm(state(idx.quaternion,step+1));
    % RK4 ara aşamalarından sonra gerçek yüzey durumunu fiziksel sınırda tut.
    state(idx.surfaceDeg,step+1) = min(max(state(idx.surfaceDeg,step+1), ...
        -p.actuatorLimitDeg),p.actuatorLimitDeg);
    if ~all(isfinite(state(:,step+1)))
        error('run_f16_flat:nonfinite','Nonfinite state at step %d.',step);
    end
    if cfg.stopAtGround && -state(idx.position(3),step+1) <= 0
        % Yer teması bir sonraki tam adımda saptanır (zaman kayması <= dt).
        lastSample = step+1;
        break
    end
    if strcmp(cfg.clock,'realtime')
        % Gerçek zaman seçimi fiziksel dt'yi değiştirmez; yalnız bekletir.
        pause(max(0,time(step+1)-toc(wall)));
    end
end
time = time(1:lastSample);
state = state(:,1:lastSample);
altitude = zeros(1,lastSample);
latlon = nan(2,lastSample);
velocityBody = zeros(3,lastSample);
attitudeEulerDeg = zeros(3,lastSample);
% Sonuçları kullanıcı için ortak yerel NED/gövde biçimine geri dönüştür.
for k = 1:lastSample
    altitude(k) = -state(idx.position(3),k);
    C_NB = f.quatToDCM(state(idx.quaternion,k));
    velocityBody(:,k) = state(idx.velocity,k);
    attitudeEulerDeg(:,k) = [atan2(C_NB(3,2),C_NB(3,3));
        -asin(min(max(C_NB(3,1),-1),1));
        atan2(C_NB(2,1),C_NB(1,1))]*180/pi;
end
result.time = time;
result.state = state;
result.altitude = altitude;
result.velocityBody = velocityBody;
result.attitudeEulerDeg = attitudeEulerDeg;
result.actuatorDeg = state(idx.surfaceDeg,:);
result.latitudeLongitudeDeg = latlon;
result.earth = cfg.earth;
result.clock = cfg.clock;
result.wallSeconds = toc(wall);
result.options = cfg;
end

function cfg = defaultOptions()
% Varsayılan başlangıç bir trim noktası değildir; örnek bir serbest uçuştur.
cfg.earth = 'flat';
cfg.clock = 'simulation';
cfg.dt = 0.01;
cfg.duration = 10;
cfg.controls = [0;0;0;0.5]; % elevatör/kanatçık/dümen (°), gaz (0..1)
cfg.windNED = [0;0;0];      % m/s
cfg.initialVelocityBody = [150;0;0]; % gövde hız bileşenleri, m/s
cfg.initialOmegaBody = [0;0;0];      % seçilen Dünya'ya göre, rad/s
cfg.initialEulerDeg = [0;0;0];       % NED'e göre yatış/dikilme/baş
cfg.initialNED = [0;0;-5000];        % düz Dünya konumu, m
cfg.initialEnginePower = 20;         % 0..100 motor güç durumu
cfg.initialActuatorDeg = [];          % boşsa yüzeyler ilk komutta başlar
cfg.stopAtGround = true;
cfg.cgFraction = 0.35; % ortalama aerodinamik kord üzerindeki CG konumu
end
