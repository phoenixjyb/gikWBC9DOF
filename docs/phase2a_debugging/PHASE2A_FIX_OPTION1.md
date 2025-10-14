# Phase 2A Fix - Option 1: Reset to Home Config for Stage C

**Date**: October 14, 2025  
**Issue**: Staged pipeline fails in second half (1558mm error) due to accumulated drift from Stage B starting configuration  
**Root Cause**: baseSeedFromEE() sequential warm-starting propagates suboptimal initial config, causing 1.5m base path divergence by waypoint 106

## Fix Strategy

Force Stage C to start from **home configuration** instead of Stage B ending configuration.

### Rationale
- Isolated test (starting from home): 1.2mm error throughout ✓
- Staged run (starting from Stage B end): 1.4mm first half, 1558mm second half ❌
- Base path divergence correlates r=0.985 with EE error
- Different starting configs → different base seed paths → catastrophic drift

### Implementation

Modify `executeStageCPPFirst()` wrapper in `runStagedTrajectory.m` to:
1. Accept Stage B ending config `qStart` as before
2. **Override** it with home configuration for baseSeedFromEE
3. Keep Stage B ending for initial robot state (first waypoint execution)

This gives us:
- **Base seed path generation**: Uses home config → generates optimal path
- **Initial robot state**: Uses Stage B ending → realistic continuation
- **Best of both worlds**: Good planning + continuous execution

## Expected Outcome

If this fixes the issue:
- ✅ Mean error: ~1-3mm (vs 780mm broken)
- ✅ Second half errors: Similar to first half
- ✅ Base path divergence: Minimal (similar to isolated test)
- ✅ **Confirms initialization is the root cause**

If this doesn't fix it:
- Need to investigate other factors (workspace limits, PP controller issues, etc.)

## Rollback Plan

Simple: Keep backup of original code, restore if needed.
