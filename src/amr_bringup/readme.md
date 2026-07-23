                  bringup.launch.py
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
 robot_state_publisher  ros2_control_node   Controller Spawners
        │                  │                  │
        │                  ▼                  ▼
        │          Loads DraxHardware   joint_state_broadcaster
        │                                 diff_drive_controller
        ▼
 publishes TF