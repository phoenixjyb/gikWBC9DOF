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
%       primitives - FIXED-SIZE struct array with fields:
%           .Vx  - Forward velocity [m/s]
%           .Wz  - Yaw rate [rad/s]
%           .dt  - Duration [s]
%           .arc_length - Approximate distance traveled [m]
%           NOTE: Invalid primitives have Vx=0, Wz=0, dt=0
%                 Check dt > 0 to identify valid primitives
%
%   Primitive design strategy:
%       1. Forward arcs (Vx > 0, Wz varied) - primary motion
%       2. Backward arcs (Vx < 0, Wz varied) - parking/reversing
%       3. NO spin-in-place (passive rear prevents Vx=0 rotation)
%       4. Enforce R = |Vx/Wz| >= min_turning_radius
%
%   Primitive count: ~18-20 primitives (fixed array size: 30)
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

% Preallocate primitive array with FIXED SIZE for codegen
max_primitives = 30;
primitives = repmat(struct('Vx', 0.0, 'Wz', 0.0, 'dt', 0.0, ...
                           'arc_length', 0.0), ...
                    max_primitives, 1);

% Use fixed index - iterate through ALL slots
idx = int32(1);

%% Generate forward primitives
for i_vx = 1:length(Vx_forward)
    Vx = Vx_forward(i_vx);
    
    for i_wz = 1:length(Wz_options_base)
        if idx > max_primitives
            break;
        end
        
        Wz = Wz_options_base(i_wz);
        
        % Check constraints
        is_valid = checkPrimitiveConstraints(Vx, Wz, track, Vwheel_max, Wz_max, R_min);
        
        if is_valid
            % Use medium duration (1.0s) for most primitives
            dt = 1.0;
            
            % Add primitive at current index
            primitives(idx).Vx = Vx;
            primitives(idx).Wz = Wz;
            primitives(idx).dt = dt;
            primitives(idx).arc_length = abs(Vx) * dt;
            
            idx = idx + 1;
        end
    end
end

%% Generate backward primitives (fewer, slower)
for i_vx = 1:length(Vx_backward)
    if idx > max_primitives
        break;
    end
    
    Vx = Vx_backward(i_vx);
    
    % Fewer yaw options for backward (straight, gentle left/right)
    Wz_backward = [-1.0, 0.0, 1.0];  % [rad/s]
    
    for i_wz = 1:length(Wz_backward)
        if idx > max_primitives
            break;
        end
        
        Wz = Wz_backward(i_wz);
        
        % Check constraints
        is_valid = checkPrimitiveConstraints(Vx, Wz, track, Vwheel_max, Wz_max, R_min);
        
        if is_valid
            dt = 1.0;  % Standard duration
            
            % Add primitive at current index
            primitives(idx).Vx = Vx;
            primitives(idx).Wz = Wz;
            primitives(idx).dt = dt;
            primitives(idx).arc_length = abs(Vx) * dt;
            
            idx = idx + 1;
        end
    end
end

%% Add a few longer-duration primitives for straight motion
if idx <= max_primitives
    Vx_straight = 0.8;   % [m/s] Fast forward
    Wz_straight = 0.0;   % [rad/s] No turning
    dt_long = 1.5;       % [s] Longer duration
    
    is_valid = checkPrimitiveConstraints(Vx_straight, Wz_straight, track, Vwheel_max, Wz_max, R_min);
    if is_valid
        primitives(idx).Vx = Vx_straight;
        primitives(idx).Wz = Wz_straight;
        primitives(idx).dt = dt_long;
        primitives(idx).arc_length = abs(Vx_straight) * dt_long;
        
        idx = idx + 1;
    end
end

% DO NOT TRIM - Keep fixed-size array for codegen compatibility
% Unused slots have dt=0, which caller can filter out
% primitives = primitives(1:primitive_count);  % REMOVED for codegen

% Display summary
valid_count = int32(0);
for i = 1:max_primitives
    if primitives(i).dt > 0
        valid_count = valid_count + 1;
    end
end
fprintf('Generated %d motion primitives\n', valid_count);
if valid_count > 0
    fwd_count = int32(0);
    bck_count = int32(0);
    for i = 1:max_primitives
        if primitives(i).dt > 0
            if primitives(i).Vx > 0
                fwd_count = fwd_count + 1;
            elseif primitives(i).Vx < 0
                bck_count = bck_count + 1;
            end
        end
    end
    fprintf('  - Forward: %d\n', fwd_count);
    fprintf('  - Backward: %d\n', bck_count);
end
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
