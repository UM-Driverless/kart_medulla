# TODO

- Design mode selection (REMOTE / AUTONOMOUS / MANUAL) and state handling; consider using a button + steering angle to choose modes when no extra ADCs are available, with a future option to move to a touchscreen UI.

- **Merge with C refactor branch**: The other branch has a C-style periodic task wrapper (`createPeriodicTask`) using `vTaskDelayUntil` for deterministic timing. Merge requires:
  - Switch from C++ to C (rename `.cpp` to `.c`, adjust syntax)
  - Bring over from this branch:
    - Pin configuration constants
    - Safety task logic and analog sensor functions
    - State machine (`KartMode` enum and `runMode()`)
    - Motor control integration
  - Bring over README.md notes and links (hardware docs, PCB design)
