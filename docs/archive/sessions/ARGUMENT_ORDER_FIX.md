# Argument Order Fix for unifiedChassisCtrl

**Issue**: MATLAB `arguments` block requires all required parameters before any optional ones.

## Original (Broken) Signature:
```matlab
function [cmd, state] = unifiedChassisCtrl(mode, ref, estPose, state, params)
arguments
    mode ...
    ref ...
    estPose ...
    state struct = struct()   % OPTIONAL (has default)
    params struct             % REQUIRED ❌ Error!
end
```

**Error**: `params` (required) came after `state` (optional with default).

## Fixed Signature:
```matlab
function [cmd, state] = unifiedChassisCtrl(mode, ref, estPose, params, state)
arguments
    mode ...
    ref ...
    estPose ...
    params struct             % REQUIRED ✅
    state struct = struct()   % OPTIONAL (has default) ✅
end
```

**Fix**: Swapped order so `params` (required) comes before `state` (optional).

## Files Modified:
1. `matlab/+gik9dof/+control/unifiedChassisCtrl.m` - Fixed function signature
2. `matlab/holisticVelocityController.m` - Updated call to match new order

## Testing:
Now you can run:
```matlab
run('matlab/test_velocityController.m')
```

Should work without errors! ✅
