drax_hardware/
├── include/
│   └── drax_hardware/
│       ├── socket_can.hpp
│       ├── canopen_node.hpp
│       ├── motor.hpp
│       └── drax_hardware.hpp
│
├── src/
│   ├── socket_can.cpp
│   ├── canopen_node.cpp
│   ├── motor.cpp
│   └── drax_hardware.cpp




diff_drive_controller
          │
          ▼
     DraxHardware
          │
          ▼
        Motor
          │
          ▼
     CanOpenNode
          │
          ▼
     SocketCanBus
          │
          ▼
        CAN Bus
          │
          ▼
       Hub Motor