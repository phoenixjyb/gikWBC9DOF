function primitives = generateMotionPrimitives(params)
%GENERATEMOTIONPRIMITIVES Create motion primitive set for front-diff + passive-rear robot
%   primitives = generateMotionPrimitives(params)
%
%   Generates a set of motion primitives for Hybrid A* planning.
%   Each primitive is a (Vx, Wz, dt) command tuple.
%
%   Platform: Front differential + passive rear omniwheels
%   - Min turning radius: ~0.34 m (passive rear constraint)
%   - NO spin-in-place (Vx=0, Wz≠0 not possible)
%   - Control: (Vx, Wz) via front axle differential
%
%   INPUTS:
%       params - Chassis parameters from getChassisParams()
%                Required fields: .track, .Vwheel_max, .Vx_max, .Wz_max,
%                                 .min_turning_radius, .wheelbase
%
%   OUTPUTS:
%       primitives - Struct array with fields:
%           .Vx  - Forward velocity [m/s]
%           .Wz  - Yaw rate [rad/s]
%           .dt  - Duration [s]
%           .arc_length - Approximate distance traveled [m]
%           .description - String description
%
%   Primitive design strategy:
%       1. Forward arcs (Vx > 0, Wz varied) - primary motion
%       2. Backward arcs (Vx < 0, Wz varied) - parking/reversing
%       3. NO spin-in-place (passive rear prevents Vx=0 rotation)
%       4. Enforce R = |Vx/Wz| >= min_turning_radius
%
%   Primitive count: ~18-20 primitives
%       - 8 forward: straight + left/right arcs (3 speeds × 5 yaw rates - duplicates)
%       - 6 backward: straight + left/right arcs
%       - Coverage: left/straight/right, slow/medium/fast
%
%   See also computeMotionPrimitive, HybridState, getChassisParams

%#codegen

% Extract parameters
track = params.track;
Vwheel_max = params.Vwheel_max;
Vx_max = params.Vx_max;
Wz_max = params.Wz_max;
R_min = params.min_turning_radius;

% Primitive generation settings
dt_options = [0.5, 1.0, 1.5];           % [s] Duration options
Vx_forward = [0.4, 0.6, 0.8];          % [m/s] Forward speeds
Vx_backward = [-0.3, -0.5];            % [m/s] Backward speeds (slower)

% Yaw rate options (enforce min radius constraint)
% For R_min = 0.34 m and Vx = 0.8 m/s: Wz_max_for_R_min = 0.8/0.34 = 2.35 rad/s
Wz_options_base = [-2.0, -1.0, 0.0, 1.0, 2.0];  % [rad/s]

% Preallocate primitive array (estimate ~20 primitives)
max_primitives = 30;
primitives = repmat(struct('Vx', 0.0, 'Wz', 0.0, 'dt', 0.0, ...
                           'arc_length', 0.0, 'description', ''), ...
                    max_primitives, 1);

primitive_count = 0;

%% Generate forward primitives
for i_vx = 1:length(Vx_forward)
    Vx = Vx_forward(i_vx);
    
    for i_wz = 1:length(Wz_options_base)
        Wz = Wz_options_base(i_wz);
        
        % Check constraints
        if ~checkPrimitiveConstraints(Vx, Wz, track, Vwheel_max, Wz_max, R_min)
            continue;  % Skip invalid primitive
        end
        
        % Use medium duration (1.0s) for most primitives
        dt = 1.0;
        
        % Add primitive
        primitive_count = primitive_count + 1;
        primitives(primitive_count).Vx = Vx;
        primitives(primitive_count).Wz = Wz;
        primitives(primitive_count).dt = dt;
        primitives(primitive_count).arc_length = abs(Vx) * dt;
        primitives(primitive_count).description = sprintf('FWD Vx=%.1f Wz=%.1f', Vx, Wz);
    end
end

%% Generate backward primitives (fewer, slower)
for i_vx = 1:length(Vx_backward)
    Vx = Vx_backward(i_vx);
    
    % Fewer yaw options for backward (straight, gentle left/right)
    Wz_backward = [-1.0, 0.0, 1.0];  % [rad/s]
    
    for i_wz = 1:length(Wz_backward)
        Wz = Wz_backward(i_wz);
        
        % Check constraints
        if ~checkPrimitiveConstraints(Vx, Wz, track, Vwheel_max, Wz_max, R_min)
            continue;
        end
        
        dt = 1.0;  % Standard duration
        
        % Add primitive
        primitive_count = primitive_count + 1;
        primitives(primitive_count).Vx = Vx;
        primitives(primitive_count).Wz = Wz;
        primitives(primitive_count).dt = dt;
        primitives(primitive_count).arc_length = abs(Vx) * dt;
        primitives(primitive_count).description = sprintf('BCK Vx=%.1f Wz=%.1f', Vx, Wz);
    end
end

%% Add a few longer-duration primitives for straight motion
Vx_straight = 0.8;   % [m/s] Fast forward
Wz_straight = 0.0;   % [rad/s] No turning
dt_long = 1.5;       % [s] Longer duration

if checkPrimitiveConstraints(Vx_straight, Wz_straight, track, Vwheel_max, Wz_max, R_min)
    primitive_count = primitive_count + 1;
    primitives(primitive_count).Vx = Vx_straight;
    primitives(primitive_count).Wz = Wz_straight;
    primitives(primitive_count).dt = dt_long;
    primitives(primitive_count).arc_length = abs(Vx_straight) * dt_long;
    primitives(primitive_count).description = 'FWD FAST STRAIGHT';
end

% Trim unused slots
primitives = primitives(1:primitive_count);

% Display summary
fprintf('Generated %d motion primitives\n', primitive_count);
fprintf('  - Forward: %d\n', sum([primitives.Vx] > 0));
fprintf('  - Backward: %d\n', sum([primitives.Vx] < 0));
fprintf('  - Min radius enforced: %.3f m\n', R_min);

end

%% Helper function: Check primitive constraints
function is_valid = checkPrimitiveConstraints(Vx, Wz, track, Vwheel_max, Wz_max, R_min)
%CHECKPRIMITIVECONSTRAINTS Validate motion primitive satisfies all constraints
%   Returns true if primitive (Vx, Wz) is feasible

    % 1. Wheel speed limits
    v_L = Vx - (track / 2) * Wz;
    v_R = Vx + (track / 2) * Wz;
    
    if abs(v_L) > Vwheel_max || abs(v_R) > Vwheel_max
        is_valid = false;
        return;
    end
    
    % 2. Yaw rate limit
    if abs(Wz) > Wz_max
        is_valid = false;
        return;
    end
    
    % 3. Minimum turning radius (if turning)
    Wz_threshold = 1e-4;  % [rad/s]
    if abs(Wz) >= Wz_threshold
        R = abs(Vx / Wz);
        if R < R_min
            is_valid = false;
            return;
        end
    end
    
    % All checks passed
    is_valid = true;
end
