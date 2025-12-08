\# Architectural Discussion



We need to create a research and development environment to build data acquisition and sensor pipelines to feed Embodied AIs that create intelligent and autonomous behavior in a robot, specifically drones, quadrupeds, bipeds, and wheeled robots. We do NOT use ROS or ROS-2, instead we are going to use raw Linux and Zenoh. The development environment needs to be able to interface with both IAMAI Project AirSim, and NVIDIA Isaac. Initially, we want to create easy to understand demonstration programs that demonstrate way-point route following, object tracking, and obstacle avoidance. However, the environment needs to be able to expand into more sophisticated autonomy, with visual SLAM, visual way point navigation, and swarm coordination and learning. This document describes how the R\&D repo needs to be structured, what the progression is going to be of different demonstrations and libraries, and how the cmake build system is going to be organized to govern development and extensions. We also explore the virtualization of the simulation environment so that we can use Project AirSim or NVIDIA Isaac for visualization and possibly training set development.



The choice of \*\*Zenoh\*\* over ROS/ROS 2 is well-suited for a high-performance, resource-efficient, and decentralized data fabric, which is critical for complex embodied AI applications, especially with constrained robot platforms. Zenoh's publish/subscribe, storage, and query primitives will be the backbone of the system.



Here is an implementation plan detailing the repository structure, progression, build system, and virtualization solution.



-----



\## Repository Structure



The repository should be structured to cleanly separate the core communication layer, the simulation interfaces, the robot-specific logic, and the high-level autonomous behaviors.



```

embodied\_ai/

├── cmake/                      # Reusable CMake modules/functions

├── libs/                       # Core Zenoh and utility libraries (Zenoh::lib, Sensor::lib, etc.)

│   ├── zenoh\_interface/        # Zenoh publishers/subscribers for standard data types (e.g., Image, Odometry, Control)

│   ├── data\_types/             # Zenoh-agnostic C++ data structures and serialization utilities (e.g., custom Zenoh Encoding)

│   └── control\_algorithms/     # Low-level control logic (e.g., PID, trajectory generation)

├── platforms/                  # Robot-specific integration layers (where Embodied AI meets hardware/simulator)

│   ├── drone\_platform/         # Drone-specific setup (AirSim/Isaac vehicle interface)

│   ├── quadruped\_platform/     # Quadruped-specific setup (Isaac platform interface)

│   └── common\_hardware/        # Code for common sensors (e.g., generic Linux sensor drivers)

├── sim\_interfaces/             # Libraries for connecting to simulation environments

│   ├── airsim\_client/          # Library for connecting to AirSim API and publishing data via Zenoh

│   └── isaac\_client/           # Library for connecting to NVIDIA Isaac Sim/Lab and publishing data via Zenoh

├── demonstrations/             # Standalone executables for initial milestones

│   ├── 01\_waypoint\_following/  

│   ├── 02\_object\_tracking/

│   └── 03\_obstacle\_avoidance/

├── autonomy\_stack/             # High-level intelligent behaviors and future sophisticated modules

│   ├── visual\_slam/

│   ├── visual\_nav/

│   └── swarm\_coordinator/

├── scripts/                    # Utility scripts (setup, run, data logging, visualization)

├── third\_party/                # External dependencies not managed by system package manager (e.g., Zenoh-C/C++ headers)

└── CMakeLists.txt              # Top-level CMake file

```



-----



\## CMake Build System Organization



The CMake system should be modular and leverage the power of modern CMake features (targets, imported libraries) for clarity and dependency management.



1\.  \*\*Top-Level `CMakeLists.txt`:\*\*



&nbsp;     \* Defines the project, required C++ standard (C++17 or newer is recommended for Zenoh-C++), and includes all subdirectories.

&nbsp;     \* Uses `add\_subdirectory()` for `libs/`, `sim\_interfaces/`, `platforms/`, `demonstrations/`, and `autonomy\_stack/`.



2\.  \*\*External Dependencies (Zenoh):\*\*



&nbsp;     \* The Zenoh-C or \*\*Zenoh-C++\*\* library should be managed as a `third\_party/` dependency or integrated using CMake's `find\_package()` after manually building/installing Zenoh's prerequisites.

&nbsp;     \* The top-level or `third\_party/CMakeLists.txt` will find/define the \*\*Zenoh library targets\*\* (e.g., `zenohcxx::zenohc`).



3\.  \*\*Library-Level (`libs/`, `sim\_interfaces/`) `CMakeLists.txt`:\*\*



&nbsp;     \* Use `add\_library(zenoh\_interface STATIC ...)` to create a library target.

&nbsp;     \* Use `target\_link\_libraries(zenoh\_interface PUBLIC zenohcxx::zenohc)` to specify dependencies.

&nbsp;     \* Use `target\_include\_directories()` to manage headers.



4\.  \*\*Executable-Level (`demonstrations/`) `CMakeLists.txt`:\*\*



&nbsp;     \* Use `add\_executable(waypoint\_follower ...)` for each demonstration.

&nbsp;     \* Link to necessary libraries: `target\_link\_libraries(waypoint\_follower PRIVATE zenoh\_interface control\_algorithms drone\_platform)`.



-----



\## Progression of Demonstrations and Libraries



The plan is phased, starting with basic movement and Zenoh communication, and expanding into sophisticated autonomy.



| Phase | Libraries/Components Developed | Demonstration Programs | Zenoh Key Expression Examples (Pub/Sub) |

| :--- | :--- | :--- | :--- |

| \*\*Phase 1: Communication Core\*\* | `libs/zenoh\_interface`, `libs/data\_types`, `sim\_interfaces/\*\_client` (basic sensor pub/sub) | \*\*Sensor Data Streaming:\*\* `airsim\_client` publishes camera/IMU data to a Linux subscriber/viewer. | `robot/drone/sensor/camera/rgb` `robot/drone/sensor/imu` |

| \*\*Phase 2: Basic Autonomy\*\* | `libs/control\_algorithms` (PID, basic state machine), `platforms/\*\_platform` (teleop/command interface) | \*\*01: Way-Point Following:\*\* A Linux node publishes waypoints, which the robot platform subscribes to and executes using control algorithms. | `robot/drone/cmd/velocity` `autonomy/waypoint\_topic` |

| \*\*Phase 3: Perceptive Autonomy\*\* | Initial `autonomy\_stack/object\_tracker` (simple bounding box pub/sub), \*\*03: Obstacle Avoidance\*\* (basic sensor fusion/reactive behavior) | \*\*02: Object Tracking:\*\* A perception node processes images and publishes object location; a control node subscribes and directs the robot. \*\*03: Obstacle Avoidance:\*\* Sensor data feeds a reactive module publishing corrective velocity commands. | `robot/\*/perception/objects` `robot/\*/cmd/reactive\_vel` |

| \*\*Phase 4: Advanced Autonomy\*\* | Full `autonomy\_stack/visual\_slam`, `autonomy\_stack/visual\_nav` | \*\*Visual Waypoint Navigation:\*\* Integrate SLAM for localization and map building, feeding into a path planner for visual navigation. | `robot/\*/state/pose\_slam` `robot/\*/map/occupancy` |

| \*\*Phase 5: Multi-Agent/Learning\*\* | `autonomy\_stack/swarm\_coordinator`, Deep Learning inference nodes | \*\*Swarm Coordination:\*\* Multiple simulated robots communicate via Zenoh's routing and query/storage features to coordinate a task (e.g., distributed mapping). | `swarm/coordinator/task\_assignment` `swarm/\*/state/pose` |



-----



\## Virtualization Solution for Simulation



Both Unreal Engine (AirSim) and NVIDIA Isaac Sim are \*\*resource-intensive\*\* and leverage GPUs. The best solutions for virtualization/remote access are:



1\.  \*\*GPU-Enabled Cloud Instances:\*\*



&nbsp;     \* Utilize \*\*NVIDIA Isaac Sim Development Workstation VMI\*\* on platforms like \*\*Microsoft Azure\*\* (as mentioned in search results) or similar GPU-enabled cloud services (AWS, GCP).

&nbsp;     \* This provides a dedicated, high-end virtual machine with an NVIDIA GPU and drivers, perfect for running the simulators and generating synthetic data.

&nbsp;     \* \*\*Zenoh\*\* runs natively on the Linux guest OS, handling the low-latency data fabric between the simulator and your autonomy logic.



2\.  \*\*Local Containerization with GPU Pass-through:\*\*



&nbsp;     \* Use \*\*Docker\*\* or \*\*Singularity\*\* with NVIDIA Container Toolkit to pass the host GPU directly to a container.

&nbsp;     \* Containerize the \*\*Zenoh-based R\\\&D environment\*\* and run the autonomy code in the container, while the simulation environment (AirSim/Isaac Sim) runs on the host OS or in another GPU-enabled container. This is a robust way to ensure a consistent, reproducible build environment.



The Zenoh-based data pipeline allows for a clean split between the autonomy pipeline and virtual world simulation:



&nbsp; \* \*\*Simulator (AirSim/Isaac):\*\* Runs on the GPU-enabled machine (virtual or physical). Publishes virtual sensor data to Zenoh.

&nbsp; \* \*\*Autonomy Code:\*\* Runs either on the same machine (for high-speed local loop) or remotely (e.g., a development laptop) by connecting to the Zenoh network.



The video below discusses Zenoh's role in creating a data fabric for edge computing, which is the exact decentralized paradigm you are implementing.



\[Zenoh: The Edge Computing Data Fabric](https://www.youtube.com/watch?v=\_wAdFHrESY0)





