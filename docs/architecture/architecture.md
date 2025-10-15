# Project Architecture Overview

Below are block diagrams (Mermaid) showing modules, data flow, and layout.

## Module Interaction

```mermaid
flowchart LR
    CY[Config YAML\nagent_control_pkg/config/*.yaml] --> CR[Config Reader\nconfig_reader.{hpp,cpp}]
    CR --> RUN[Simulation Runner\nmulti_drone_pid_test_main.cpp]

    FPY[Fuzzy Params YAML\nfuzzy_params.yaml] --> FL[ Fuzzy Params Loader\nfuzzy_params_loader.{hpp,cpp}]
    FL --> FLS[GT2‑FLS\ngt2_fuzzy_logic_system.{hpp,cpp}]

    RUN --> PID[PID Logic\npid_controller.{hpp,cpp}]
    RUN --> FLS
    RUN --> PHY[FakeDrone Physics\n(dynamics in runner)]
    PHY --> RUN

    RUN --> CSV[CSV/Metrics Outputs\nresults/simulation_outputs]
    CSV --> ANA[Analysis Scripts\nanalysis/*.py]

    T1[PID Unit Tests\n test/test_pid_controller.cpp] --> PID
    T2[FLS Unit Tests\n test/test_gt2_fls.cpp] --> FLS

    classDef dim fill:#eef,stroke:#88a,color:#111;
    classDef io fill:#efe,stroke:#6a6,color:#111;
    classDef data fill:#ffe,stroke:#aa6,color:#111;
    class CY,FPY,CSV data;
    class CR,FL,RUN,PID,FLS,PHY,ANA,T1,T2 dim;
```

## Folder Layout (key parts)

```mermaid
flowchart TB
    subgraph A[agent_control_pkg]
      subgraph AI[include/agent_control_pkg]
        H1[pid_controller.hpp]
        H2[gt2_fuzzy_logic_system.hpp]
        H3[config_reader.hpp]
        H4[fuzzy_params_loader.hpp]
        H5[agent_controller_node.hpp (placeholder)]
      end
      subgraph AS[src]
        C1[pid_controller.cpp]
        C2[gt2_fuzzy_logic_system.cpp]
        C3[config_reader.cpp]
        C4[fuzzy_params_loader.cpp]
        C5[multi_drone_pid_test_main.cpp]
      end
      subgraph AT[test]
        T1T[test_pid_controller.cpp]
        T2T[test_gt2_fls.cpp]
      end
      AC[config/*.yaml]
    end

    subgraph B[other_packages]
      subgraph FCP[formation_coordinator_pkg (placeholders)]
        FC1[include/formation_coordinator_node.hpp]
        FC2[include/particle_swarm_optimizer.hpp]
        FC3[src/formation_coordinator_node.cpp]
        FC4[src/particle_swarm_optimizer.cpp]
        FC5[src/formation_coordinator_main.cpp]
      end
    end

    D[docs/]
    R[results/]
    Y[analysis/*.py]

    A --> D
    A --> R
    A --> Y
    B --> D
```

## ROS 2 Migration Plan (high‑level)

```mermaid
flowchart LR
    subgraph Runtime
      ACN[agent_control_pkg\nAgent Controller Node(s)]
      FCN[formation_coordinator_pkg\nFormation Coordinator Node]
      SIM[(Webots/AirSim/Standalone Physics)]
    end

    PCORE[AgentControllerCore\n(reuse PID + FLS)] --> ACN
    ACN <--> SIM
    ACN <--> FCN
    FCN -->|targets/params| ACN

    subgraph Config
      RY[ROS2 YAML params\nros__parameters]
    end

    RY --> ACN
    RY --> FCN
```

Notes
- The Simulation Runner is currently the most complete executable for experiments.
- For ROS 2, extract an `AgentControllerCore` from the runner and reuse in C++ nodes.
- `agent_control_pkg/src/ros/agent_controller_node.cpp:1` now wraps the PID/Fuzzy adapters inside a ROS 2 node, while `formation_coordinator_pkg/src/formation_coordinator_node.cpp:1` drives multi-agent setpoints and exposes a `/set_formation` service for shape changes.
