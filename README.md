# semester5-os-traffic-intersection-sim

Traffic intersection simulator modeling two junctions (F10/F11) with vehicles, signals, parking, and emergency corridors to demonstrate operating systems concepts (processes, pthreads, semaphores, pipes, and signals).

## Tech stack
- C++ (POSIX APIs, pthreads, semaphores)
- Designed for Linux/Unix builds (uses `unistd.h`, `sys/types.h`, `select`, `fork`/pipes)

## Build
1. Ensure a C++17 toolchain with pthread support (e.g., `g++`).
2. From the project root:
   - `g++ -std=c++17 -pthread -o traffic_sim main.cpp`
3. Run the simulator:
   - `./traffic_sim`

## Project layout
- `main.cpp`: Entry point orchestrating vehicle threads, controllers, IPC, and logging.
- `controller.h`, `display.h`, `intersection.h`, `parkinglot.h`, `simulation.h`, `vehicle.h`: Core domain types and helpers.

## Notes
- The simulation uses POSIX primitives and is not portable to Windows without compatibility layers.
- Adjust timing constants in the headers if you need different traffic or parking behaviors.
