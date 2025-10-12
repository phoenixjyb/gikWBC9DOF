```mermaid
flowchart TD
    subgraph "Input Layer"
        EE_POSES["End-Effector Poses<br/>[4×4×N] SE(3)"]
        BASE_PATH["Base Path<br/>[N×3] (x,y,θ)"]
    end
    
    subgraph "Control Functions"
        RTC["runTrajectoryControl<br/><b>IK Control Loop</b><br/>518 lines"]
        SCC["simulateChassisExecution<br/><b>Chassis Simulation</b><br/>328 lines"]
    end
    
    subgraph "Solvers/Controllers"
        GIK["GIK Solver<br/>generalizedInverseKinematics<br/>(MATLAB Toolbox)"]
        PP["Pure Pursuit Controller<br/>purePursuitFollower<br/>(Custom Implementation)"]
    end
    
    subgraph "Output Layer"
        Q_TRAJ["Joint Trajectory<br/>qTraj [9×N]<br/>(all DOF)"]
        VEL_CMD["Velocity Commands<br/>(Vx, Wz) [N×2]<br/>(base only)"]
    end
    
    %% Connections
    EE_POSES --> RTC
    RTC --> GIK
    GIK --> Q_TRAJ
    
    BASE_PATH --> SCC
    SCC --> PP
    PP --> VEL_CMD
    
    %% Styling
    classDef inputStyle fill:#e1f5ff,stroke:#01579b,stroke-width:2px
    classDef controlStyle fill:#fff3e0,stroke:#e65100,stroke-width:3px
    classDef solverStyle fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef outputStyle fill:#e8f5e9,stroke:#1b5e20,stroke-width:2px
    
    class EE_POSES,BASE_PATH inputStyle
    class RTC,SCC controlStyle
    class GIK,PP solverStyle
    class Q_TRAJ,VEL_CMD outputStyle
```

# Three-Pass Architecture: How They Work Together

```mermaid
flowchart TB
    subgraph "Pass 1: Reference IK"
        P1_INPUT["Input: EE Poses [4×4×N]"]
        P1_RTC["runTrajectoryControl<br/>(GIK solver)"]
        P1_OUTPUT["Output: qTraj [9×N]<br/><i>IDEAL trajectory</i>"]
        P1_EXTRACT["Extract base:<br/>baseRef = qTraj(1:3,:)'<br/>[N×3]"]
        
        P1_INPUT --> P1_RTC
        P1_RTC --> P1_OUTPUT
        P1_OUTPUT --> P1_EXTRACT
    end
    
    subgraph "Pass 2: Chassis Simulation"
        P2_INPUT["Input: baseRef [N×3]"]
        P2_SCC["simulateChassisExecution<br/>(Pure pursuit + dynamics)"]
        P2_OUTPUT["Output: executedBase [N×3]<br/><i>REALISTIC trajectory</i>"]
        P2_CONSTRAINTS["Applies:<br/>• vx_max = 1.5 m/s<br/>• wz_max = 2.0 rad/s<br/>• accel = 0.8 m/s²<br/>• wheel limits"]
        
        P2_INPUT --> P2_SCC
        P2_SCC --> P2_CONSTRAINTS
        P2_CONSTRAINTS --> P2_OUTPUT
    end
    
    subgraph "Pass 3: Final IK with Fixed Base"
        P3_INPUT["Input: EE Poses [4×4×N]<br/>+ executedBase [N×3]"]
        P3_RTC["runTrajectoryControl<br/>(GIK + FixedJointTrajectory)"]
        P3_OUTPUT["Output: qTraj [9×N]<br/><i>ACTUAL trajectory</i>"]
        P3_NOTE["Base locked to executedBase<br/>Arm tracks EE"]
        
        P3_INPUT --> P3_RTC
        P3_RTC --> P3_NOTE
        P3_NOTE --> P3_OUTPUT
    end
    
    P1_EXTRACT --> P2_INPUT
    P2_OUTPUT --> P3_INPUT
    
    P3_OUTPUT --> FINAL["Final Log:<br/>• qTraj [9×N]<br/>• eePositions [3×N]<br/>• purePursuit data<br/>• chassis commands"]
    
    %% Styling
    classDef pass1Style fill:#ffebee,stroke:#c62828,stroke-width:2px
    classDef pass2Style fill:#fff3e0,stroke:#ef6c00,stroke-width:2px
    classDef pass3Style fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px
    classDef finalStyle fill:#e3f2fd,stroke:#1565c0,stroke-width:3px
    
    class P1_INPUT,P1_RTC,P1_OUTPUT,P1_EXTRACT pass1Style
    class P2_INPUT,P2_SCC,P2_OUTPUT,P2_CONSTRAINTS pass2Style
    class P3_INPUT,P3_RTC,P3_OUTPUT,P3_NOTE pass3Style
    class FINAL finalStyle
```

# Key Differences Summary

| Aspect | runTrajectoryControl | simulateChassisExecution |
|--------|---------------------|--------------------------|
| **Problem Type** | Inverse Kinematics | Controller Simulation |
| **Input** | EE poses (SE(3)) | Base waypoints (x,y,θ) |
| **Output** | Joint angles qTraj | Velocity commands (Vx,Wz) |
| **Solver** | generalizedInverseKinematics | Pure pursuit follower |
| **DOF** | All 9 (3 base + 6 arm) | Base only (3 DOF) |
| **Constraints** | IK constraints (pose, distance) | Chassis dynamics (velocity, accel) |
| **Integration** | None (step-by-step IK) | Yes (integrates Vx,Wz → x,y,θ) |
| **Purpose** | Find joint config for EE target | Simulate how robot follows path |
| **Used In** | Pass 1, Pass 3, Stage A, Stage B (gikInLoop) | Pass 2, Stage B (pureHyb) |

**Conclusion:** They are complementary, not duplicated! ✅
