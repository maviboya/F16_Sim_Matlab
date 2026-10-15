function result = run_f16_3dof(options)
%RUN_F16_3DOF Kitaptaki düz Dünya boylamsal modeli sabit adımlı RK4 ile çöz.
%   result = run_f16_3dof();
%   result = run_f16_3dof(struct('controls',[-1;0.5]));
% controls = [elevatör (derece); gaz (0..1)]. Rüzgâr yerel NED'de m/s.
% Durum sırası F16Dynamics3DOF ve README.md içinde açıklanır.
if nargin < 1
    options = struct();
end
cfg = defaultOptions();
names = fieldnames(options);
for k = 1:numel(names)
    cfg.(names{k}) = options.(names{k});
end
cfg.clock = char(cfg.clock);
if ~ismember(cfg.clock,{'simulation','realtime'})
    error('run_f16_3dof:clock','clock must be simulation or realtime.');
end
if ~isscalar(cfg.dt) || ~isfinite(cfg.dt) || cfg.dt <= 0 || ...
        ~isscalar(cfg.duration) || ~isfinite(cfg.duration) || cfg.duration < 0
    error('run_f16_3dof:time','dt must be positive and duration nonnegative.');
end
stepCount = round(cfg.duration/cfg.dt);
if abs(stepCount*cfg.dt-cfg.duration) > 1e-9*max(1,cfg.duration)
    error('run_f16_3dof:time','duration must be an integer multiple of dt.');
end
if numel(cfg.initialVelocityBody) ~= 2 || ...
        any(~isfinite(cfg.initialVelocityBody(:))) || ...
        numel(cfg.initialNED) ~= 2 || any(~isfinite(cfg.initialNED(:))) || ...
        numel(cfg.windNED) ~= 3 || any(~isfinite(cfg.windNED(:))) || ...
        cfg.windNED(2) ~= 0
    error('run_f16_3dof:initial', ...
        'Velocity and position need 2 elements; windNED needs zero east wind.');
end
if ~isscalar(cfg.initialPitchRateRadS) || ...
        ~isfinite(cfg.initialPitchRateRadS) || ...
        ~isscalar(cfg.initialPitchDeg) || ~isfinite(cfg.initialPitchDeg) || ...
        ~isscalar(cfg.initialEnginePower) || ~isfinite(cfg.initialEnginePower)
    error('run_f16_3dof:initial', ...
        'Initial pitch rate, pitch and engine power must be finite scalars.');
end
cfg.initialVelocityBody = cfg.initialVelocityBody(:);
cfg.initialNED = cfg.initialNED(:);
cfg.windNED = cfg.windNED(:);
p = F16Parameters();
idx = p.longitudinalIndex;
initialState = [cfg.initialVelocityBody;cfg.initialPitchRateRadS; ...
    cfg.initialNED;cfg.initialPitchDeg*pi/180;cfg.initialEnginePower;0];
if isempty(cfg.initialElevatorDeg)
    if isa(cfg.controls,'function_handle')
        initialControls = cfg.controls(0,initialState);
    else
        initialControls = cfg.controls;
    end
    initialControls = initialControls(:);
    if numel(initialControls) ~= 2 || any(~isfinite(initialControls))
        error('run_f16_3dof:controls','Controls must have 2 finite elements.');
    end
    initialElevator = initialControls(1);
else
    initialElevator = cfg.initialElevatorDeg;
    if ~isscalar(initialElevator) || ~isfinite(initialElevator)
        error('run_f16_3dof:elevator', ...
            'initialElevatorDeg must be one finite angle.');
    end
end
initialState(idx.elevatorDeg) = min(max(initialElevator, ...
    -p.actuatorLimitDeg(1)),p.actuatorLimitDeg(1));

time = (0:stepCount)*cfg.dt;
state = zeros(numel(initialState),stepCount+1);
state(:,1) = initialState;
lastSample = stepCount+1;
wall = tic;
for step = 1:stepCount
    state(:,step+1) = RK4(@F16Dynamics3DOF,time(step), ...
        state(:,step),cfg.dt,cfg,p);
    state(idx.elevatorDeg,step+1) = min(max( ...
        state(idx.elevatorDeg,step+1),-p.actuatorLimitDeg(1)), ...
        p.actuatorLimitDeg(1));
    if ~all(isfinite(state(:,step+1)))
        error('run_f16_3dof:nonfinite','Nonfinite state at step %d.',step);
    end
    if cfg.stopAtGround && state(idx.down,step+1) >= 0
        lastSample = step+1;
        break
    end
    if strcmp(cfg.clock,'realtime')
        pause(max(0,time(step+1)-toc(wall)));
    end
end
time = time(1:lastSample);
state = state(:,1:lastSample);
sampleCount = numel(time);
result.time = time;
result.state = state;
result.altitude = -state(idx.down,:);
result.positionNED = [state(idx.north,:);zeros(1,sampleCount); ...
    state(idx.down,:)];
result.velocityBody = [state(idx.u,:);zeros(1,sampleCount);state(idx.w,:)];
result.attitudeEulerDeg = [zeros(1,sampleCount); ...
    state(idx.theta,:)*180/pi;zeros(1,sampleCount)];
result.pitchDeg = state(idx.theta,:)*180/pi;
result.pitchRateRadS = state(idx.q,:);
result.actuatorDeg = state(idx.elevatorDeg,:);
result.earth = 'flat';
result.model = 'longitudinal3dof';
result.clock = cfg.clock;
result.wallSeconds = toc(wall);
result.options = cfg;
end

function cfg = defaultOptions()
% İlk durum trimli değildir; 6 DOF örneğiyle aynı hız ve irtifayı kullanır.
cfg.clock = 'simulation';
cfg.dt = 0.01;
cfg.duration = 10;
cfg.controls = [0;0.5];
cfg.windNED = [0;0;0];
cfg.initialVelocityBody = [150;0]; % [u;w], m/s
cfg.initialPitchRateRadS = 0;
cfg.initialNED = [0;-5000];         % [kuzey;aşağı], m
cfg.initialPitchDeg = 0;
cfg.initialEnginePower = 20;
cfg.initialElevatorDeg = [];        % boşsa ilk komuta eşit
cfg.stopAtGround = true;
end
