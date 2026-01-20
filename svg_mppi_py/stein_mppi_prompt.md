Project Path: proj-svg_mppi-main

Source Tree:

```txt
proj-svg_mppi-main
├── Dockerfile
├── LICENSE
├── Makefile
├── README.md
├── data
│   ├── map
│   │   ├── berlin
│   │   │   └── map.yaml
│   │   └── spielberg
│   │       └── map.yaml
│   ├── reference_path
│   │   ├── berlin
│   │   └── spielberg
│   └── rviz
│       └── rviz.rviz
├── docs
│   └── assets
├── launch
│   ├── publish_initial_pose.launch
│   └── simulation_launcher.launch
├── script
│   ├── eval.sh
│   ├── eval_all.sh
│   ├── launch_controllers.sh
│   ├── launch_simulator.sh
│   └── reset_env.sh
└── src
    ├── custom_msgs
    │   ├── mppi_metrics_msgs
    │   │   ├── CMakeLists.txt
    │   │   ├── msg
    │   │   │   └── MPPIMetrics.msg
    │   │   └── package.xml
    │   └── waypoint_msg
    │       ├── CMakeLists.txt
    │       ├── msg
    │       │   └── Waypoint.msg
    │       └── package.xml
    ├── eval
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── eval.launch
    │   ├── log
    │   │   ├── ablation_study
    │   │   │   ├── wo_cov_adaptation
    │   │   │   │   ├── obstacle_avoidance
    │   │   │   │   │   ├── ablation_obstacle_avoidance.pdf
    │   │   │   │   │   └── result.txt
    │   │   │   │   └── path_tracking
    │   │   │   │       ├── ablation_path_tracking.pdf
    │   │   │   │       └── result.txt
    │   │   │   └── wo_nominal_solution
    │   │   │       ├── obstacle_avoidance
    │   │   │       │   ├── cost_box.pdf
    │   │   │       │   └── result.txt
    │   │   │       └── path_tracking
    │   │   │           ├── cost_box.pdf
    │   │   │           └── result.txt
    │   │   ├── comparison_with_baselines.pdf
    │   │   ├── obstacle_avoidance
    │   │   │   ├── comparison_obstacle_avoidance.pdf
    │   │   │   └── result.txt
    │   │   └── path_tracking
    │   │       ├── comparison_path_tracking.pdf
    │   │       └── result.txt
    │   ├── msg
    │   │   └── RaceInfo.msg
    │   ├── package.xml
    │   ├── script
    │   │   └── plot.py
    │   ├── src
    │   │   └── eval.py
    │   └── tmp
    ├── local_costmap_generator
    │   ├── CMakeLists.txt
    │   ├── README.md
    │   ├── config
    │   │   ├── local_costmap_generator.yaml
    │   │   └── local_costmap_generator_simulation.yaml
    │   ├── include
    │   │   └── local_costmap_generator
    │   │       └── local_costmap_generator.hpp
    │   ├── launch
    │   │   └── local_costmap_generator.launch
    │   ├── package.xml
    │   └── src
    │       ├── local_costmap_generator.cpp
    │       └── local_costmap_generator_node.cpp
    ├── mppi_controller
    │   ├── CMakeLists.txt
    │   ├── README.md
    │   ├── config
    │   │   └── mppi_controller.yaml
    │   ├── include
    │   │   └── mppi_controller
    │   │       ├── StopWatch.hpp
    │   │       ├── common.hpp
    │   │       ├── forward_mppi.hpp
    │   │       ├── mpc_base.hpp
    │   │       ├── mpc_template.hpp
    │   │       ├── mppi_controller_ros.hpp
    │   │       ├── prior_samples_with_costs.hpp
    │   │       ├── reverse_mppi.hpp
    │   │       ├── stein_variational_guided_mppi.hpp
    │   │       └── stein_variational_mpc.hpp
    │   ├── launch
    │   │   └── mppi_controller.launch
    │   ├── package.xml
    │   └── src
    │       ├── forward_mppi.cpp
    │       ├── mpc_base.cpp
    │       ├── mppi_controller_node.cpp
    │       ├── mppi_controller_ros.cpp
    │       ├── prior_samples_with_costs.cpp
    │       ├── reverse_mppi.cpp
    │       ├── stein_variational_guided_mppi.cpp
    │       └── stein_variational_mpc.cpp
    ├── racecar_model
    │   ├── CMakeLists.txt
    │   ├── ego_racecar.xacro
    │   ├── launch
    │   │   └── racecar_model.launch
    │   ├── opp_racecar.xacro
    │   └── package.xml
    ├── reference_sdf_generator
    │   ├── CMakeLists.txt
    │   ├── README.md
    │   ├── config
    │   │   └── reference_sdf_generator.yaml
    │   ├── include
    │   │   └── reference_sdf_generator
    │   │       └── reference_sdf_generator.hpp
    │   ├── launch
    │   │   └── reference_sdf_generator.launch
    │   ├── package.xml
    │   └── src
    │       ├── reference_sdf_generator.cpp
    │       └── reference_sdf_generator_node.cpp
    ├── reference_waypoint_loader
    │   ├── CMakeLists.txt
    │   ├── README.md
    │   ├── include
    │   │   └── reference_waypoint_loader
    │   │       ├── rapidcsv.h
    │   │       ├── reference_waypoint_loader.hpp
    │   │       └── tinycolormap.hpp
    │   ├── launch
    │   │   └── reference_waypoint_loader.launch
    │   ├── package.xml
    │   └── src
    │       ├── reference_waypoint_loader.cpp
    │       └── reference_waypoint_loader_node.cpp
    └── simulator
        ├── CMakeLists.txt
        ├── Dockerfile
        ├── LICENSE
        ├── README.md
        ├── build_docker.sh
        ├── docker.sh
        ├── ego_racecar.xacro
        ├── f1tenth_gym
        │   ├── Dockerfile
        │   ├── LICENSE
        │   ├── README.md
        │   ├── examples
        │   │   ├── config_example_map.yaml
        │   │   ├── example_map.yaml
        │   │   └── waypoint_follow.py
        │   ├── gym
        │   │   └── f110_gym
        │   │       ├── __init__.py
        │   │       ├── envs
        │   │       │   ├── __init__.py
        │   │       │   ├── base_classes.py
        │   │       │   ├── collision_models.py
        │   │       │   ├── dynamic_models.py
        │   │       │   ├── f110_env.py
        │   │       │   ├── f110_env_backup.py
        │   │       │   ├── laser_models.py
        │   │       │   ├── maps
        │   │       │   │   ├── berlin.yaml
        │   │       │   │   ├── levine.pgm
        │   │       │   │   ├── levine.yaml
        │   │       │   │   ├── skirk.yaml
        │   │       │   │   ├── stata_basement.yaml
        │   │       │   │   └── vegas.yaml
        │   │       │   └── rendering.py
        │   │       └── unittest
        │   │           ├── __init__.py
        │   │           ├── collision_checks.py
        │   │           ├── dynamics_test.py
        │   │           ├── legacy_scan.npz
        │   │           ├── legacy_scan_gen.py
        │   │           ├── pyglet_test.py
        │   │           ├── pyglet_test_camera.py
        │   │           ├── random_trackgen.py
        │   │           └── scan_sim.py
        │   └── setup.py
        ├── launch
        │   ├── agent_template.launch
        │   ├── gym_bridge.launch
        │   ├── gym_bridge.rviz
        │   ├── gym_bridge_host.launch
        │   └── racecar_model.launch
        ├── maps
        │   ├── backups
        │   │   ├── Spielberg.yaml
        │   │   ├── skirk.yaml
        │   │   └── vegas.yaml
        │   └── map.yaml
        ├── msg
        │   └── RaceInfo.msg
        ├── opp_racecar.xacro
        ├── package.xml
        ├── params.yaml
        ├── scripts
        │   ├── agent_utils.py
        │   ├── agents.py
        │   ├── dummy_agent_node.py
        │   └── gym_bridge.py
        └── start.sh

```

`Dockerfile`:

```
   1 | FROM ros:noetic-ros-base-focal
   2 | 
   3 | # Set environment variables
   4 | ENV DEBIAN_FRONTEND=noninteractive
   5 | ENV ROS_DISTRO=noetic
   6 | ENV DISABLE_ROS1_EOL_WARNINGS=true
   7 | 
   8 | # Update and install basic dependencies
   9 | RUN apt-get update && apt-get install -y \
  10 |     build-essential \
  11 |     cmake \
  12 |     git \
  13 |     python3-dev \
  14 |     python3-pip \
  15 |     python3-catkin-tools \
  16 |     python-is-python3 \
  17 |     wget \
  18 |     curl \
  19 |     sudo \
  20 |     vim \
  21 |     && rm -rf /var/lib/apt/lists/*
  22 | 
  23 | # Install ROS dependencies
  24 | RUN apt-get update && apt-get install -y \
  25 |     ros-${ROS_DISTRO}-ackermann-msgs \
  26 |     ros-${ROS_DISTRO}-geometry-msgs \
  27 |     ros-${ROS_DISTRO}-nav-msgs \
  28 |     ros-${ROS_DISTRO}-std-msgs \
  29 |     ros-${ROS_DISTRO}-tf2-geometry-msgs \
  30 |     ros-${ROS_DISTRO}-tf2-ros \
  31 |     ros-${ROS_DISTRO}-grid-map \
  32 |     ros-${ROS_DISTRO}-grid-map-ros \
  33 |     ros-${ROS_DISTRO}-grid-map-msgs \
  34 |     ros-${ROS_DISTRO}-grid-map-rviz-plugin \
  35 |     ros-${ROS_DISTRO}-grid-map-visualization \
  36 |     ros-${ROS_DISTRO}-grid-map-pcl \
  37 |     ros-${ROS_DISTRO}-costmap-2d \
  38 |     ros-${ROS_DISTRO}-map-server \
  39 |     ros-${ROS_DISTRO}-xacro \
  40 |     ros-${ROS_DISTRO}-robot-state-publisher \
  41 |     ros-${ROS_DISTRO}-joint-state-publisher \
  42 |     ros-${ROS_DISTRO}-rviz \
  43 |     ros-${ROS_DISTRO}-jsk-rviz-plugins \
  44 |     && rm -rf /var/lib/apt/lists/*
  45 | 
  46 | # Install additional dependencies from Makefile
  47 | RUN apt-get update && apt-get install -y \
  48 |     libomp-dev \
  49 |     mpv \
  50 |     xdotool \
  51 |     && rm -rf /var/lib/apt/lists/*
  52 | 
  53 | # Install Python dependencies
  54 | RUN pip3 install --upgrade pip && \
  55 |     pip3 install \
  56 |     numpy \
  57 |     matplotlib \
  58 |     pandas \
  59 |     PyYAML
  60 | 
  61 | # Setup workspace
  62 | WORKDIR /workspace
  63 | 
  64 | # Initialize rosdep
  65 | RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
  66 |     rosdep update"
  67 | 
  68 | # Create workspace directory
  69 | RUN mkdir -p /workspace
  70 | 
  71 | # Setup entrypoint
  72 | RUN echo "#!/bin/bash\n\
  73 | source /opt/ros/${ROS_DISTRO}/setup.bash\n\
  74 | if [ -f /workspace/devel/setup.bash ]; then\n\
  75 |     source /workspace/devel/setup.bash\n\
  76 | fi\n\
  77 | exec \"\$@\"" > /ros_entrypoint.sh && \
  78 |     chmod +x /ros_entrypoint.sh
  79 | 
  80 | # Enable GUI support
  81 | ENV QT_X11_NO_MITSHM=1
  82 | ENV DISPLAY=:0
  83 | 
  84 | # Create a non-root user that can be overridden at runtime
  85 | ARG USER_ID=1000
  86 | ARG GROUP_ID=1000
  87 | RUN groupadd -g ${GROUP_ID} developer && \
  88 |     useradd -m -u ${USER_ID} -g developer -s /bin/bash developer && \
  89 |     echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
  90 | 
  91 | # Make entrypoint executable by all users
  92 | RUN chmod 755 /ros_entrypoint.sh
  93 | 
  94 | # Set working directory permissions
  95 | RUN chmod -R 755 /workspace
  96 | 
  97 | ENTRYPOINT ["/ros_entrypoint.sh"]
  98 | CMD ["bash"]

```

`LICENSE`:

```
   1 | MIT License
   2 | 
   3 | Copyright (c) 2023 kohonda
   4 | 
   5 | Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | of this software and associated documentation files (the "Software"), to deal
   7 | in the Software without restriction, including without limitation the rights
   8 | to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | copies of the Software, and to permit persons to whom the Software is
  10 | furnished to do so, subject to the following conditions:
  11 | 
  12 | The above copyright notice and this permission notice shall be included in all
  13 | copies or substantial portions of the Software.
  14 | 
  15 | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | SOFTWARE.

```

`Makefile`:

```
   1 | # Usage: make [command]
   2 | SHELL := /bin/bash
   3 | setup:
   4 | 	# install dependencies for ROS 	
   5 | 	rosdep install -i --from-path src --rosdistro noetic -y
   6 | 	sudo apt update
   7 | 	sudo apt install -y ros-noetic-map-server python3-catkin-tools libomp-dev ros-noetic-jsk-rviz-plugins mpv
   8 | 
   9 | # build without simulator with ROS
  10 | .PHONY: build
  11 | build:
  12 | 	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_GPU=OFF
  13 | 
  14 | clean:
  15 | 	# clean build files
  16 | 	rm -rf build devel logs .catkin_tools install
  17 | 
  18 | # Docker commands
  19 | docker_build:
  20 | 	docker build \
  21 | 		--build-arg USER_ID=$(shell id -u) \
  22 | 		--build-arg GROUP_ID=$(shell id -g) \
  23 | 		-t svg-mppi:latest .
  24 | 
  25 | .PHONY: bash
  26 | bash:
  27 | 	xhost +local:docker
  28 | 	docker run -it --rm \
  29 | 		--network host \
  30 | 		--privileged \
  31 | 		--user developer \
  32 | 		-e DISPLAY=${DISPLAY} \
  33 | 		-e QT_X11_NO_MITSHM=1 \
  34 | 		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  35 | 		-v ${HOME}/.Xauthority:/home/developer/.Xauthority:rw \
  36 | 		-v ${PWD}:/workspace \
  37 | 		-w /workspace \
  38 | 		svg-mppi:latest \
  39 | 		bash
  40 | 

```

`README.md`:

```md
   1 | 
   2 | # Stein Variational Guided Model Predictive Path Integral Control (SVG-MPPI)
   3 | 
   4 | ### [**Paper**](https://arxiv.org/abs/2309.11040) | [**Video**](https://www.youtube.com/watch?v=ML_aOYQIDL0) 
   5 | 
   6 | This package includes ROS implementation of [Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles](https://arxiv.org/abs/2309.11040) presented in ICRA 2024.
   7 | 
   8 | 
   9 | https://github.com/user-attachments/assets/25bd67c5-3e9c-4c4e-9a79-cacf066f4af5
  10 | 
  11 | 
  12 | ![Overview](docs/assets/overview_svg_mppi.png)
  13 | 
  14 | ## Tested Environment
  15 | - Ubuntu 20.04, 22.04, and 24.04
  16 | - Docker
  17 | 
  18 | <details>
  19 | <summary>Docker Installation</summary>
  20 | 
  21 | [Installation guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
  22 | 
  23 | ```bash
  24 | # Install from get.docker.com
  25 | curl -fsSL https://get.docker.com -o get-docker.sh
  26 | sudo sh get-docker.sh
  27 | sudo groupadd docker
  28 | sudo usermod -aG docker $USER
  29 | ```
  30 | 
  31 | </details>
  32 | 
  33 | ## Quick Start with Docker
  34 | 
  35 | ### 1. Build Docker Image
  36 | ```bash
  37 | make docker_build
  38 | ```
  39 | 
  40 | ### 2. Launch Simulator and Controllers
  41 | 
  42 | Launch simulator in a new terminal
  43 | ```bash
  44 | cd proj-svg_mppi
  45 | ./script/launch_simulator.sh
  46 | ```
  47 | 
  48 | Launch controllers in another terminal
  49 | ```bash
  50 | cd proj-svg_mppi
  51 | # Enter Docker container with GUI support
  52 | make bash
  53 | # Inside container, launch controllers
  54 | ./script/launch_controllers.sh
  55 | ```
  56 | You can change the all MPPI parameters and settings in [the yaml file](./src/mppi_controller/config/mppi_controller.yaml)
  57 | 
  58 | 
  59 | 
  60 | https://github.com/user-attachments/assets/06b4fd91-6a2c-4f6e-9f44-94393595ece2
  61 | 
  62 | 
  63 | 
  64 | ## Run on Native System
  65 | 
  66 | ### Requirements
  67 | 
  68 | - Ubuntu 20.04
  69 | - ROS Noetic
  70 | 
  71 | ### 1. Install Dependencies
  72 | 
  73 | ```bash
  74 | cd proj-svg_mppi
  75 | make setup
  76 | ```
  77 | 
  78 | ### 2. Build the Project
  79 | 
  80 | ```bash
  81 | cd proj-svg_mppi
  82 | make build
  83 | ```
  84 | 
  85 | ### 3. Launch Simulator and Controllers
  86 | 
  87 | Launch simulator in the Docker container
  88 | ```bash
  89 | cd proj-svg_mppi
  90 | ./script/launch_simulator.sh
  91 | ```
  92 | 
  93 | Launch controllers in another terminal
  94 | ```bash
  95 | cd proj-svg_mppi
  96 | ./script/launch_controllers.sh 
  97 | ```
  98 | 
  99 | ## Evaluation
 100 | 
 101 | You can reproduce all simulation results in the paper by one command: 
 102 | ```bash
 103 | cd proj-svg_mppi/script
 104 | ./eval_all.sh
 105 | ```
 106 | 
 107 | Or, You can evaluate a fixed parameters by this command:
 108 | ```bash
 109 | cd proj-svg_mppi/script
 110 | ./eval.sh
 111 | ```
 112 | 
 113 | **Note**: The evaluation is used asynchronous simulation using ROS. So, the results can be slightly changed even if all seeds are fixed.
 114 | 
 115 | 
 116 | ## Citation
 117 | 
 118 | ```bibtex
 119 | @inproceedings{honda2024stein,
 120 |   title={Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles},
 121 |   author={Honda, Kohei and Akai, Naoki and Suzuki, Kosuke and Aoki, Mizuho and Hosogaya, Hirotaka and Okuda, Hiroyuki and Suzuki, Tatsuya},
 122 |   booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
 123 |   pages={7020--7026},
 124 |   year={2024},
 125 |   organization={IEEE}
 126 | }
 127 | ```

```

`data\map\berlin\map.yaml`:

```yaml
   1 | image: map.png
   2 | resolution: 0.050000
   3 | origin: [-11.606540, -26.520793, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196

```

`data\map\spielberg\map.yaml`:

```yaml
   1 | image: map.png
   2 | resolution: 0.05796
   3 | origin: [-84.85359914210505, -36.30299725862132, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.45
   6 | free_thresh: 0.196

```

`data\rviz\rviz.rviz`:

```rviz
   1 | Panels:
   2 |   - Class: rviz/Displays
   3 |     Help Height: 138
   4 |     Name: Displays
   5 |     Property Tree Widget:
   6 |       Expanded:
   7 |         - /best_path1/Namespaces1
   8 |       Splitter Ratio: 0.5
   9 |     Tree Height: 429
  10 |   - Class: rviz/Selection
  11 |     Name: Selection
  12 |   - Class: rviz/Tool Properties
  13 |     Expanded:
  14 |       - /2D Pose Estimate1
  15 |       - /2D Nav Goal1
  16 |       - /Publish Point1
  17 |     Name: Tool Properties
  18 |     Splitter Ratio: 0.5886790156364441
  19 |   - Class: rviz/Views
  20 |     Expanded:
  21 |       - /Current View1
  22 |     Name: Views
  23 |     Splitter Ratio: 0.5
  24 |   - Class: rviz/Time
  25 |     Name: Time
  26 |     SyncMode: 0
  27 |     SyncSource: LaserScan
  28 |   - Class: jsk_rviz_plugin/RecordAction
  29 |     Name: RecordAction
  30 |   - Class: jsk_rviz_plugin/PublishTopic
  31 |     Name: PublishTopic
  32 |     Topic: mppi/stop
  33 |   - Class: jsk_rviz_plugin/PublishTopic
  34 |     Name: PublishTopic
  35 |     Topic: mppi/start
  36 | Preferences:
  37 |   PromptSaveOnExit: true
  38 | Toolbars:
  39 |   toolButtonStyle: 2
  40 | Visualization Manager:
  41 |   Class: ""
  42 |   Displays:
  43 |     - Alpha: 0.5
  44 |       Cell Size: 1
  45 |       Class: rviz/Grid
  46 |       Color: 160; 160; 164
  47 |       Enabled: false
  48 |       Line Style:
  49 |         Line Width: 0.029999999329447746
  50 |         Value: Lines
  51 |       Name: Grid
  52 |       Normal Cell Count: 0
  53 |       Offset:
  54 |         X: 0
  55 |         Y: 0
  56 |         Z: 0
  57 |       Plane: XY
  58 |       Plane Cell Count: 10
  59 |       Reference Frame: <Fixed Frame>
  60 |       Value: false
  61 |     - Alpha: 0.699999988079071
  62 |       Class: rviz/Map
  63 |       Color Scheme: map
  64 |       Draw Behind: false
  65 |       Enabled: true
  66 |       Name: Map
  67 |       Topic: /map
  68 |       Unreliable: false
  69 |       Use Timestamp: false
  70 |       Value: true
  71 |     - Alpha: 1
  72 |       Buffer Length: 1
  73 |       Class: rviz/Path
  74 |       Color: 92; 53; 102
  75 |       Enabled: true
  76 |       Head Diameter: 0.30000001192092896
  77 |       Head Length: 0.20000000298023224
  78 |       Length: 0.30000001192092896
  79 |       Line Style: Lines
  80 |       Line Width: 0.029999999329447746
  81 |       Name: Path
  82 |       Offset:
  83 |         X: 0
  84 |         Y: 0
  85 |         Z: 0
  86 |       Pose Color: 255; 85; 255
  87 |       Pose Style: None
  88 |       Queue Size: 10
  89 |       Radius: 0.029999999329447746
  90 |       Shaft Diameter: 0.10000000149011612
  91 |       Shaft Length: 0.10000000149011612
  92 |       Topic: /reference_path
  93 |       Unreliable: false
  94 |       Value: true
  95 |     - Alpha: 1
  96 |       Autocompute Intensity Bounds: true
  97 |       Autocompute Value Bounds:
  98 |         Max Value: 0
  99 |         Min Value: 0
 100 |         Value: true
 101 |       Axis: Z
 102 |       Channel Name: intensity
 103 |       Class: rviz/LaserScan
 104 |       Color: 255; 255; 255
 105 |       Color Transformer: AxisColor
 106 |       Decay Time: 0
 107 |       Enabled: true
 108 |       Invert Rainbow: false
 109 |       Max Color: 252; 175; 62
 110 |       Min Color: 245; 121; 0
 111 |       Name: LaserScan
 112 |       Position Transformer: XYZ
 113 |       Queue Size: 10
 114 |       Selectable: true
 115 |       Size (Pixels): 4
 116 |       Size (m): 0.009999999776482582
 117 |       Style: Points
 118 |       Topic: /scan
 119 |       Unreliable: false
 120 |       Use Fixed Frame: true
 121 |       Use rainbow: true
 122 |       Value: true
 123 |     - Alpha: 1
 124 |       Class: rviz/Axes
 125 |       Enabled: false
 126 |       Length: 0.4000000059604645
 127 |       Name: Axes
 128 |       Radius: 0.05000000074505806
 129 |       Reference Frame: ego_racecar/base_link
 130 |       Show Trail: false
 131 |       Value: false
 132 |     - Alpha: 1
 133 |       Class: rviz/PointStamped
 134 |       Color: 204; 41; 204
 135 |       Enabled: true
 136 |       History Length: 1
 137 |       Name: PointStamped
 138 |       Queue Size: 10
 139 |       Radius: 0.20000000298023224
 140 |       Topic: /target_point
 141 |       Unreliable: false
 142 |       Value: true
 143 |     - Alpha: 0.699999988079071
 144 |       Class: rviz/Map
 145 |       Color Scheme: costmap
 146 |       Draw Behind: false
 147 |       Enabled: false
 148 |       Name: costmap
 149 |       Topic: /f1_costmap_2d/f1_costmap/costmap
 150 |       Unreliable: false
 151 |       Use Timestamp: false
 152 |       Value: false
 153 |     - Alpha: 1
 154 |       Class: rviz/Polygon
 155 |       Color: 25; 255; 0
 156 |       Enabled: true
 157 |       Name: costmap_footprint
 158 |       Queue Size: 10
 159 |       Topic: /f1_costmap_2d/f1_costmap/footprint
 160 |       Unreliable: false
 161 |       Value: true
 162 |     - Alpha: 1
 163 |       Class: rviz/RobotModel
 164 |       Collision Enabled: false
 165 |       Enabled: true
 166 |       Links:
 167 |         All Links Enabled: true
 168 |         Expand Joint Details: false
 169 |         Expand Link Details: false
 170 |         Expand Tree: false
 171 |         Link Tree Style: Links in Alphabetic Order
 172 |         back_left_wheel:
 173 |           Alpha: 1
 174 |           Show Axes: false
 175 |           Show Trail: false
 176 |           Value: true
 177 |         back_right_wheel:
 178 |           Alpha: 1
 179 |           Show Axes: false
 180 |           Show Trail: false
 181 |           Value: true
 182 |         base_link:
 183 |           Alpha: 1
 184 |           Show Axes: false
 185 |           Show Trail: false
 186 |           Value: true
 187 |         front_left_hinge:
 188 |           Alpha: 1
 189 |           Show Axes: false
 190 |           Show Trail: false
 191 |         front_left_wheel:
 192 |           Alpha: 1
 193 |           Show Axes: false
 194 |           Show Trail: false
 195 |           Value: true
 196 |         front_right_hinge:
 197 |           Alpha: 1
 198 |           Show Axes: false
 199 |           Show Trail: false
 200 |         front_right_wheel:
 201 |           Alpha: 1
 202 |           Show Axes: false
 203 |           Show Trail: false
 204 |           Value: true
 205 |         laser_model:
 206 |           Alpha: 1
 207 |           Show Axes: false
 208 |           Show Trail: false
 209 |           Value: true
 210 |       Name: EgoCarModel
 211 |       Robot Description: ego_racecar/robot_description
 212 |       TF Prefix: ego_racecar
 213 |       Update Interval: 0
 214 |       Value: true
 215 |       Visual Enabled: true
 216 |     - Alpha: 1
 217 |       Class: rviz/RobotModel
 218 |       Collision Enabled: false
 219 |       Enabled: true
 220 |       Links:
 221 |         All Links Enabled: true
 222 |         Expand Joint Details: false
 223 |         Expand Link Details: false
 224 |         Expand Tree: false
 225 |         Link Tree Style: Links in Alphabetic Order
 226 |         back_left_wheel:
 227 |           Alpha: 1
 228 |           Show Axes: false
 229 |           Show Trail: false
 230 |           Value: true
 231 |         back_right_wheel:
 232 |           Alpha: 1
 233 |           Show Axes: false
 234 |           Show Trail: false
 235 |           Value: true
 236 |         base_link:
 237 |           Alpha: 1
 238 |           Show Axes: false
 239 |           Show Trail: false
 240 |           Value: true
 241 |         front_left_hinge:
 242 |           Alpha: 1
 243 |           Show Axes: false
 244 |           Show Trail: false
 245 |         front_left_wheel:
 246 |           Alpha: 1
 247 |           Show Axes: false
 248 |           Show Trail: false
 249 |           Value: true
 250 |         front_right_hinge:
 251 |           Alpha: 1
 252 |           Show Axes: false
 253 |           Show Trail: false
 254 |         front_right_wheel:
 255 |           Alpha: 1
 256 |           Show Axes: false
 257 |           Show Trail: false
 258 |           Value: true
 259 |         laser_model:
 260 |           Alpha: 1
 261 |           Show Axes: false
 262 |           Show Trail: false
 263 |           Value: true
 264 |       Name: OppCarModel
 265 |       Robot Description: opp_racecar/robot_description
 266 |       TF Prefix: opp_racecar
 267 |       Update Interval: 0
 268 |       Value: true
 269 |       Visual Enabled: true
 270 |     - Class: rviz/MarkerArray
 271 |       Enabled: true
 272 |       Marker Topic: /mppi/best_path
 273 |       Name: best_path
 274 |       Namespaces:
 275 |         best_path: true
 276 |       Queue Size: 100
 277 |       Value: true
 278 |     - Buffer length: 100
 279 |       Class: jsk_rviz_plugin/Plotter2D
 280 |       Enabled: true
 281 |       Name: mppi_calculation_time[ms]
 282 |       Show Value: true
 283 |       Topic: /mppi/calculation_time
 284 |       Value: true
 285 |       auto color change: false
 286 |       auto scale: true
 287 |       background color: 0; 0; 0
 288 |       backround alpha: 0
 289 |       border: true
 290 |       foreground alpha: 0.699999988079071
 291 |       foreground color: 25; 255; 240
 292 |       height: 128
 293 |       left: 128
 294 |       linewidth: 1
 295 |       max color: 255; 0; 0
 296 |       max value: 1
 297 |       min value: -1
 298 |       show caption: true
 299 |       text size: 12
 300 |       top: 128
 301 |       update interval: 0.03999999910593033
 302 |       width: 128
 303 |     - Alpha: 0.5
 304 |       Autocompute Intensity Bounds: true
 305 |       Class: grid_map_rviz_plugin/GridMap
 306 |       Color: 200; 200; 200
 307 |       Color Layer: distance_field
 308 |       Color Transformer: IntensityLayer
 309 |       Enabled: false
 310 |       Height Layer: distance_field
 311 |       Height Transformer: Flat
 312 |       History Length: 1
 313 |       Invert Rainbow: false
 314 |       Max Color: 255; 255; 255
 315 |       Max Intensity: 10
 316 |       Min Color: 0; 0; 0
 317 |       Min Intensity: 0
 318 |       Name: ReferenceSDF
 319 |       Show Grid Lines: false
 320 |       Topic: /reference_sdf
 321 |       Unreliable: false
 322 |       Use Rainbow: true
 323 |       Value: false
 324 |     - Alpha: 0.5
 325 |       Autocompute Intensity Bounds: true
 326 |       Class: grid_map_rviz_plugin/GridMap
 327 |       Color: 200; 200; 200
 328 |       Color Layer: angle_field
 329 |       Color Transformer: IntensityLayer
 330 |       Enabled: false
 331 |       Height Layer: distance_field
 332 |       Height Transformer: Flat
 333 |       History Length: 1
 334 |       Invert Rainbow: false
 335 |       Max Color: 255; 255; 255
 336 |       Max Intensity: 10
 337 |       Min Color: 0; 0; 0
 338 |       Min Intensity: 0
 339 |       Name: angleSDF
 340 |       Show Grid Lines: false
 341 |       Topic: /reference_sdf
 342 |       Unreliable: false
 343 |       Use Rainbow: true
 344 |       Value: false
 345 |     - Alpha: 0.5
 346 |       Autocompute Intensity Bounds: true
 347 |       Class: grid_map_rviz_plugin/GridMap
 348 |       Color: 200; 200; 200
 349 |       Color Layer: speed_field
 350 |       Color Transformer: IntensityLayer
 351 |       Enabled: false
 352 |       Height Layer: distance_field
 353 |       Height Transformer: Flat
 354 |       History Length: 1
 355 |       Invert Rainbow: false
 356 |       Max Color: 255; 255; 255
 357 |       Max Intensity: 10
 358 |       Min Color: 0; 0; 0
 359 |       Min Intensity: 0
 360 |       Name: speedSDF
 361 |       Show Grid Lines: false
 362 |       Topic: /reference_sdf
 363 |       Unreliable: false
 364 |       Use Rainbow: true
 365 |       Value: false
 366 |     - Class: rviz/MarkerArray
 367 |       Enabled: false
 368 |       Marker Topic: /reference_sdf/waypoints
 369 |       Name: reference_sdf_waypoints
 370 |       Namespaces:
 371 |         {}
 372 |       Queue Size: 100
 373 |       Value: false
 374 |     - Buffer length: 100
 375 |       Class: jsk_rviz_plugin/Plotter2D
 376 |       Enabled: true
 377 |       Name: speed
 378 |       Show Value: true
 379 |       Topic: /mppi/speed
 380 |       Value: true
 381 |       auto color change: false
 382 |       auto scale: true
 383 |       background color: 0; 0; 0
 384 |       backround alpha: 0
 385 |       border: true
 386 |       foreground alpha: 0.699999988079071
 387 |       foreground color: 25; 255; 240
 388 |       height: 128
 389 |       left: 300
 390 |       linewidth: 1
 391 |       max color: 255; 0; 0
 392 |       max value: 1
 393 |       min value: -1
 394 |       show caption: true
 395 |       text size: 12
 396 |       top: 128
 397 |       update interval: 0.03999999910593033
 398 |       width: 128
 399 |     - Alpha: 0.10000000149011612
 400 |       Autocompute Intensity Bounds: true
 401 |       Class: grid_map_rviz_plugin/GridMap
 402 |       Color: 200; 200; 200
 403 |       Color Layer: collision_layer
 404 |       Color Transformer: ColorLayer
 405 |       Enabled: true
 406 |       Height Layer: collision_layer
 407 |       Height Transformer: Flat
 408 |       History Length: 1
 409 |       Invert Rainbow: false
 410 |       Max Color: 255; 255; 255
 411 |       Max Intensity: 10
 412 |       Min Color: 0; 0; 0
 413 |       Min Intensity: 0
 414 |       Name: local costmap
 415 |       Show Grid Lines: false
 416 |       Topic: /local_costmap
 417 |       Unreliable: false
 418 |       Use Rainbow: true
 419 |       Value: true
 420 |     - Class: jsk_rviz_plugin/PieChart
 421 |       Enabled: true
 422 |       Name: collision rate
 423 |       Topic: /mppi/collision_rate
 424 |       Value: true
 425 |       auto color change: true
 426 |       background color: 0; 0; 0
 427 |       backround alpha: 0
 428 |       clockwise rotate direction: false
 429 |       foreground alpha: 0.699999988079071
 430 |       foreground alpha 2: 0.4000000059604645
 431 |       foreground color: 25; 255; 240
 432 |       left: 300
 433 |       max color: 255; 0; 0
 434 |       max color change threthold: 0
 435 |       max value: 1
 436 |       med color: 255; 0; 0
 437 |       med color change threthold: 0
 438 |       min value: 0
 439 |       show caption: true
 440 |       size: 128
 441 |       text size: 14
 442 |       top: 300
 443 |     - Alpha: 1
 444 |       Class: rviz/PointStamped
 445 |       Color: 204; 41; 204
 446 |       Enabled: false
 447 |       History Length: 1
 448 |       Name: PointStamped
 449 |       Queue Size: 10
 450 |       Radius: 0.10000000149011612
 451 |       Topic: /backward_point
 452 |       Unreliable: false
 453 |       Value: false
 454 |     - Class: rviz/MarkerArray
 455 |       Enabled: true
 456 |       Marker Topic: /static_obstacles
 457 |       Name: static_obs
 458 |       Namespaces:
 459 |         static_obstacles: true
 460 |       Queue Size: 100
 461 |       Value: true
 462 |     - Class: rviz/Marker
 463 |       Enabled: true
 464 |       Marker Topic: /collision_check
 465 |       Name: collision_check
 466 |       Namespaces:
 467 |         collision_check: true
 468 |       Queue Size: 100
 469 |       Value: true
 470 |     - Class: jsk_rviz_plugin/VideoCapture
 471 |       Enabled: true
 472 |       Name: VideoCapture
 473 |       Value: true
 474 |       filename: output.avi
 475 |       fps: 20
 476 |       height: 1080
 477 |       start capture: false
 478 |       use 3D viewer size: true
 479 |       width: 1920
 480 |     - Class: jsk_rviz_plugin/PieChart
 481 |       Enabled: true
 482 |       Name: cost
 483 |       Topic: /mppi/cost
 484 |       Value: true
 485 |       auto color change: true
 486 |       background color: 0; 0; 0
 487 |       backround alpha: 0
 488 |       clockwise rotate direction: false
 489 |       foreground alpha: 0.699999988079071
 490 |       foreground alpha 2: 0.4000000059604645
 491 |       foreground color: 25; 255; 240
 492 |       left: 128
 493 |       max color: 255; 0; 0
 494 |       max color change threthold: 0
 495 |       max value: 200
 496 |       med color: 255; 0; 0
 497 |       med color change threthold: 0
 498 |       min value: 0
 499 |       show caption: true
 500 |       size: 128
 501 |       text size: 14
 502 |       top: 300
 503 |     - Class: rviz/MarkerArray
 504 |       Enabled: false
 505 |       Marker Topic: /mppi/proposal_state_distributions
 506 |       Name: proposal_state_dists
 507 |       Namespaces:
 508 |         {}
 509 |       Queue Size: 100
 510 |       Value: false
 511 |     - Class: rviz/MarkerArray
 512 |       Enabled: true
 513 |       Marker Topic: /mppi/candidate_paths
 514 |       Name: candidates path
 515 |       Namespaces:
 516 |         candidate_path_line: true
 517 |         candidate_path_nodes: true
 518 |       Queue Size: 100
 519 |       Value: true
 520 |     - Class: rviz/MarkerArray
 521 |       Enabled: true
 522 |       Marker Topic: /mppi/control_covariances
 523 |       Name: steer_covariances
 524 |       Namespaces:
 525 |         steer_covariance: true
 526 |       Queue Size: 100
 527 |       Value: true
 528 |     - Class: rviz/MarkerArray
 529 |       Enabled: true
 530 |       Marker Topic: /mppi/nominal_path
 531 |       Name: nominal_path
 532 |       Namespaces:
 533 |         nominal_path_line: true
 534 |         nominal_path_nodes: true
 535 |       Queue Size: 100
 536 |       Value: true
 537 |   Enabled: true
 538 |   Global Options:
 539 |     Background Color: 48; 48; 48
 540 |     Default Light: true
 541 |     Fixed Frame: map
 542 |     Frame Rate: 30
 543 |   Name: root
 544 |   Tools:
 545 |     - Class: rviz/Interact
 546 |       Hide Inactive Objects: true
 547 |     - Class: rviz/MoveCamera
 548 |     - Class: rviz/Select
 549 |     - Class: rviz/FocusCamera
 550 |     - Class: rviz/Measure
 551 |     - Class: rviz/SetInitialPose
 552 |       Theta std deviation: 0.2617993950843811
 553 |       Topic: /initialpose
 554 |       X std deviation: 0.5
 555 |       Y std deviation: 0.5
 556 |     - Class: rviz/SetGoal
 557 |       Topic: /move_base_simple/goal
 558 |     - Class: rviz/PublishPoint
 559 |       Single click: true
 560 |       Topic: /clicked_point
 561 |   Value: true
 562 |   Views:
 563 |     Current:
 564 |       Class: rviz/ThirdPersonFollower
 565 |       Distance: 3.6161093711853027
 566 |       Enable Stereo Rendering:
 567 |         Stereo Eye Separation: 0.05999999865889549
 568 |         Stereo Focal Distance: 1
 569 |         Swap Stereo Eyes: false
 570 |         Value: false
 571 |       Field of View: 0.7853981852531433
 572 |       Focal Point:
 573 |         X: -1.007444143295288
 574 |         Y: 0.03970237076282501
 575 |         Z: -2.384185791015625e-07
 576 |       Focal Shape Fixed Size: false
 577 |       Focal Shape Size: 0.05000000074505806
 578 |       Invert Z Axis: false
 579 |       Name: Current View
 580 |       Near Clip Distance: 0.009999999776482582
 581 |       Pitch: 0.5353983044624329
 582 |       Target Frame: ego_racecar/base_link
 583 |       Yaw: 3.1453986167907715
 584 |     Saved: ~
 585 | Window Geometry:
 586 |   Displays:
 587 |     collapsed: false
 588 |   Height: 1022
 589 |   Hide Left Dock: false
 590 |   Hide Right Dock: false
 591 |   PublishTopic:
 592 |     collapsed: false
 593 |   QMainWindow State: 000000ff00000000fd0000000400000000000001fa00000340fc020000000bfb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000b0fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb00000018005000750062006c0069007300680054006f007000690063010000003d000000600000006000fffffffb00000018005000750062006c0069007300680054006f00700069006301000000a3000000600000006000fffffffb000000100044006900730070006c006100790073010000010900000274000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb00000018005200650063006f007200640041006300740069006f006e000000049e000000410000004100ffffff000000010000010f00000340fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d00000340000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000009eb0000005efc0100000002fb0000000800540069006d00650100000000000009eb000003bc00fffffffb0000000800540069006d00650100000000000004500000000000000000000006d60000034000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
 594 |   RecordAction:
 595 |     collapsed: false
 596 |   Selection:
 597 |     collapsed: false
 598 |   Time:
 599 |     collapsed: false
 600 |   Tool Properties:
 601 |     collapsed: false
 602 |   Views:
 603 |     collapsed: false
 604 |   Width: 2539
 605 |   X: 168
 606 |   Y: 27

```

`launch\publish_initial_pose.launch`:

```launch
   1 | <?xml version="1.0"?>
   2 | <launch>
   3 | 
   4 | <node name="initialpose_publisher" pkg="rostopic" type="rostopic"
   5 |     args="pub /initialpose geometry_msgs/PoseWithCovarianceStamped &quot;
   6 | {
   7 | 'header': {
   8 |   'seq': 0,
   9 |   'stamp':{
  10 |     'secs': 0,
  11 |     'nsecs': 0,
  12 |   },
  13 |   'frame_id': 'map',
  14 | },
  15 | 'pose': {
  16 |   'pose': {
  17 |     'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
  18 |     'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
  19 |   },
  20 |   'covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  21 |     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  22 |     0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  23 |   }
  24 | }&quot;" />
  25 | 
  26 | </launch>

```

`launch\simulation_launcher.launch`:

```launch
   1 | <launch>
   2 |     <arg name="workspace" default="$(env HOME)/suzlab_ws"/>
   3 |     <arg name="map_name" default="spielberg"/>
   4 |     <arg name="mppi_param_path" default="$(find mppi_controller)/config/mppi_controller.yaml"/>
   5 |     <arg name="use_rviz" default="true"/>
   6 |     <param name="/use_sim_time" value="false"/>
   7 | 
   8 |     <!-- Load racecar models -->
   9 |     <include file="$(find racecar_model)/launch/racecar_model.launch"/>
  10 | 
  11 |     <!-- Rviz -->
  12 |     <node pkg="rviz" type="rviz" name="rviz" args="-d $(arg workspace)/data/rviz/rviz.rviz" if="$(arg use_rviz)"/>
  13 | 
  14 |     <!-- Map server : disabled because the same node is provided by f1tenth_gym_ros. -->
  15 |     <!-- <node pkg="map_server" type="map_server" name="map_server" args="$(arg workspace)/data/map/map.yaml" /> -->
  16 | 
  17 |     <!-- Cost map -->
  18 |     <!-- <node pkg="f1_costmap_2d" type="f1_costmap_2d_node" name="f1_costmap_2d" output="screen">
  19 |         <rosparam command="load" file="$(find f1_costmap_2d)/params/costmap2d_simulation.yaml" />
  20 |     </node> -->
  21 | 
  22 |     <!-- Local cost map -->
  23 |     <node pkg="local_costmap_generator" type="local_costmap_generator_node" name="local_costmap_generator" output="screen">
  24 |         <rosparam command="load" file="$(find local_costmap_generator)/config/local_costmap_generator_simulation.yaml" />
  25 |     </node>
  26 | 
  27 |     <!-- reference path loader --> 
  28 |     <!-- <node pkg="reference_path_loader" type="reference_path_loader_node" name="reference_path_loader" output="screen">
  29 |         <param name="reference_path_csv" value="$(arg workspace)/data/reference_path/ego_ref_path.csv" />
  30 |         <param name="reference_path_topic" value="/reference_path" />
  31 |         <param name="map_frame" value="map" />
  32 |     </node> -->
  33 | 
  34 |     <!-- reference waypoint(= ref_path & ref_vel) loader --> 
  35 |     <node pkg="reference_waypoint_loader" type="reference_waypoint_loader_node" name="reference_waypoint_loader" output="screen">
  36 |         <param name="reference_waypoint_csv" value="$(arg workspace)/data/reference_path/$(arg map_name)/ego_ref_waypoint.csv" />
  37 |         <param name="reference_waypoint_topic" value="/reference_waypoint" />
  38 |         <param name="reference_path_topic" value="/reference_path" />
  39 |         <param name="reference_rviz_marker_topic" value="/rviz_reference_marker" />
  40 |         <param name="reference_waypoint_v_column_label" value="ref_v" />
  41 |         <param name="map_frame" value="map" />
  42 | 
  43 |         <!-- publish optimized path (for aggressive driving) -->
  44 |         <param name="reference_waypoint_x_column_label" value="opt_x" />
  45 |         <param name="reference_waypoint_y_column_label" value="opt_y" />
  46 | 
  47 |         <!-- publish centerline (for safe driving) -->
  48 |         <!-- <param name="reference_waypoint_x_column_label" value="center_x" />
  49 |         <param name="reference_waypoint_y_column_label" value="center_y" /> -->
  50 |     </node>
  51 | 
  52 |     <!-- MPC tracker with C/GMRES -->
  53 |     <!-- <include file="$(find mpc_tracker)/launch/mpc_tracker.launch"/> -->
  54 | 
  55 |     <!-- pure pursuit -->
  56 |     <!-- <include file="$(find pure_pursuit)/launch/pure_pursuit.launch">
  57 |         <arg name="is_simulation" value="true"/>
  58 |     </include> -->
  59 | 
  60 |     <!-- Reference SDF generator -->
  61 |     <include file="$(find reference_sdf_generator)/launch/reference_sdf_generator.launch"/>
  62 | 
  63 |     <!-- DWA -->
  64 |     <!-- <include file="$(find dwa)/launch/dwa.launch">
  65 |         <arg name="is_simulation" value="true" />
  66 |     </include> -->
  67 | 
  68 |     <!-- MPPI -->
  69 |     <include file="$(find mppi_controller)/launch/mppi_controller.launch">
  70 |         <arg name="is_simulation" value="true"/>
  71 |         <arg name="is_localize_less_mode" value="false"/>
  72 |         <arg name="mppi_param_path" value="$(arg mppi_param_path)"/>
  73 |     </include>
  74 | 
  75 | </launch>

```

`script\eval.sh`:

```sh
   1 | #!/bin/bash
   2 | 
   3 | # check installed xdotool
   4 | if ! type "xdotool" > /dev/null 2>&1; then
   5 |     echo "[ERROR] xdotool is not installed. Please install xdotool."
   6 |     exit 1
   7 | fi
   8 | 
   9 | MAP_NAME="berlin"
  10 | EVAL_NAME=$1
  11 | MPPI_PARAM_PATH=$2
  12 | NUM_TRIALS=$3
  13 | NUM_STATIC_OBSTACLES=$4
  14 | IS_VISUALIZE=$5
  15 | 
  16 | if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  17 |     echo "Usage: $0 <eval_name> <mppi_param_path> <num_trials> <num_static_obstacles> <is_visualize>"
  18 |     echo "<eval_name> is optional. default is 'default'"
  19 |     echo "<mppi_param_path> is optional. default is src/mppi_controller/config/mppi_controller.yaml"
  20 |     echo "<num_trials> is optional. default is 100"
  21 |     echo "<num_static_obstacles> is optional. default is 5"
  22 |     echo "<is_visualize> is optional. default is true"
  23 |     exit 0
  24 | fi
  25 | 
  26 | if [ -z "$EVAL_NAME" ]; then
  27 |     EVAL_NAME="default"
  28 | fi
  29 | 
  30 | if [ -z "$MPPI_PARAM_PATH" ]; then
  31 |     MPPI_PARAM_PATH="default"
  32 | fi
  33 | 
  34 | if [ -z "$NUM_TRIALS" ]; then
  35 |     NUM_TRIALS=100
  36 | fi
  37 | 
  38 | if [ -z "$NUM_STATIC_OBSTACLES" ]; then
  39 |     NUM_STATIC_OBSTACLES=5
  40 | fi
  41 | 
  42 | if [ -z "$IS_VISUALIZE" ]; then
  43 |     IS_VISUALIZE=false
  44 | fi
  45 | 
  46 | 
  47 | # run simulator
  48 | gnome-terminal --title="ros_gym_simulator" -- bash -c "./launch_simulator.sh $MAP_NAME $NUM_STATIC_OBSTACLES";
  49 | # wait for simulator to be ready: check xdotool
  50 | while [ -z "$(xdotool search --name "/catkin_ws/src/f1tenth_gym_ros/launch/gym_bridge.launch http://localhost:11311")" ]; do
  51 |     echo "[INFO] waiting for simulator to be ready..."
  52 |     sleep 3s;
  53 | done
  54 | 
  55 | # run controllers
  56 | gnome-terminal --title="controllers" -- bash -c "./launch_controllers.sh $MAP_NAME $MPPI_PARAM_PATH $IS_VISUALIZE" \
  57 |     && sleep 3s;
  58 | 
  59 | # run evaluation node
  60 | ROOT_WS=$(cd $(dirname $0) && cd .. && pwd)
  61 | source /opt/ros/noetic/setup.bash;
  62 | source $ROOT_WS/devel/setup.bash;
  63 | roslaunch eval_local_planner eval.launch trial_num:=$NUM_TRIALS eval_name:=$EVAL_NAME
  64 | pid_eval=$!
  65 | echo "[INFO] evaluation pid: $pid_eval"
  66 | wait $pid_eval
  67 | echo "[INFO] evaluation finished"
  68 | 
  69 | # kill simulator of another terminal
  70 | docker rm -f f1tenth_gym_container
  71 | 
  72 | # kill controllers
  73 | xdotool windowkill $(xdotool search --name "launch/simulation_launcher.launch http://localhost:11311");
  74 | 

```

`script\eval_all.sh`:

```sh
   1 | #!/bin/bash
   2 | 
   3 | # check yq installed
   4 | if ! type "yq" > /dev/null 2>&1; then
   5 |     echo "[ERROR] yq is not installed. Please install yq."
   6 |     exit 1
   7 | fi
   8 | 
   9 | MAP_NAME="berlin"
  10 | IS_VISUALIZE=false
  11 | 
  12 | CURRENT_DIR=$(cd $(dirname $0) && pwd)
  13 | SUZ_WS=$(cd $(dirname $0) && cd .. && pwd)
  14 | 
  15 | # parse template yaml
  16 | default_yaml=$SUZ_WS/src/mppi_controller/config/mppi_controller.yaml
  17 | tmp_yaml=$SUZ_WS/src/eval/tmp/tmp.yaml
  18 | 
  19 | ###########################
  20 | # Compare path tracking (pt) performance with baseline methos 
  21 | ###########################
  22 | NUM_STATIC_OBSTACLES=0 # No obstacle avoidance
  23 | NUM_TRIALS=30
  24 | ###### svg_mppi (Proposed method) ######
  25 | echo "[INFO] Evaluating svg_mppi..."
  26 | cp $default_yaml $tmp_yaml
  27 | yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
  28 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  29 | ./eval.sh "pt_svg_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  30 | echo "[INFO] Finished evaluating svg_mppi"
  31 | 
  32 | ###### sv_mpc ######
  33 | echo "[INFO] Evaluating sv_mpc..."
  34 | cp $default_yaml $tmp_yaml
  35 | yq eval '.mpc_mode = "sv_mpc"' -i $tmp_yaml
  36 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  37 | ./eval.sh "pt_sv_mpc" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  38 | echo "[INFO] Finished evaluating sv_mpc"
  39 | 
  40 | ###### reverse_mppi ######
  41 | echo "[INFO] Evaluating reverse_mppi..."
  42 | cp $default_yaml $tmp_yaml
  43 | yq eval '.mpc_mode = "reverse_mppi"' -i $tmp_yaml
  44 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  45 | ./eval.sh "pt_reverse_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  46 | echo "[INFO] Finished evaluating reverse_mppi"
  47 | 
  48 | ###### forward_mppi ######
  49 | STEER_COVS=(0.1 0.075 0.05 0.025)
  50 | for steer_cov in ${STEER_COVS[@]}; do
  51 |     echo "[INFO] Evaluating forward_mppi (steer_cov: $steer_cov)..."
  52 |     cp $default_yaml $tmp_yaml
  53 |     yq eval '.mpc_mode = "forward_mppi"' -i $tmp_yaml
  54 |     yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  55 |     yq eval ".forward_mppi.steer_cov = $steer_cov" -i $tmp_yaml
  56 |     ./eval.sh "pt_forward_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  57 |     echo "[INFO] Finished evaluating forward_mppi (steer_cov: $steer_cov)"
  58 | done
  59 | 
  60 | ###########################
  61 | # Compare obstacle avoidance (oa) performance with baseline methos 
  62 | ###########################
  63 | 
  64 | NUM_STATIC_OBSTACLES=5 # With obstacle avoidance
  65 | NUM_TRIALS=100
  66 | ###### svg_mppi (Proposed method) ######
  67 | echo "[INFO] Evaluating svg_mppi..."
  68 | cp $default_yaml $tmp_yaml
  69 | yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
  70 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  71 | ./eval.sh "oa_svg_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  72 | echo "[INFO] Finished evaluating svg_mppi"
  73 | 
  74 | ###### sv_mpc ######
  75 | echo "[INFO] Evaluating sv_mpc..."
  76 | cp $default_yaml $tmp_yaml
  77 | yq eval '.mpc_mode = "sv_mpc"' -i $tmp_yaml
  78 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  79 | ./eval.sh "oa_sv_mpc" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  80 | echo "[INFO] Finished evaluating sv_mpc"
  81 | 
  82 | ###### reverse_mppi ######
  83 | echo "[INFO] Evaluating reverse_mppi..."
  84 | cp $default_yaml $tmp_yaml
  85 | yq eval '.mpc_mode = "reverse_mppi"' -i $tmp_yaml
  86 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  87 | ./eval.sh "oa_reverse_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  88 | echo "[INFO] Finished evaluating reverse_mppi"
  89 | 
  90 | ###### forward_mppi ######
  91 | STEER_COVS=(0.1 0.075 0.05 0.025)
  92 | for steer_cov in ${STEER_COVS[@]}; do
  93 |     echo "[INFO] Evaluating forward_mppi (steer_cov: $steer_cov)..."
  94 |     cp $default_yaml $tmp_yaml
  95 |     yq eval '.mpc_mode = "forward_mppi"' -i $tmp_yaml
  96 |     yq eval '.is_visualize_mppi = false' -i $tmp_yaml
  97 |     yq eval ".forward_mppi.steer_cov = $steer_cov" -i $tmp_yaml
  98 |     ./eval.sh "oa_forward_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
  99 |     echo "[INFO] Finished evaluating forward_mppi (steer_cov: $steer_cov)"
 100 | done
 101 | 
 102 | ###########################
 103 | # Abalation study: without covariance adaptation
 104 | ###########################
 105 | 
 106 | STEER_COVS=(0.1 0.075 0.05 0.025)
 107 | 
 108 | ###### Path tracking ######
 109 | NUM_STATIC_OBSTACLES=0 # No obstacle avoidance
 110 | NUM_TRIALS=30
 111 | for steer_cov in ${STEER_COVS[@]}; do
 112 |     echo "[INFO] Evaluating svg_mppi (steer_cov: $steer_cov)..."
 113 |     cp $default_yaml $tmp_yaml
 114 |     yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
 115 |     yq eval '.is_visualize_mppi = false' -i $tmp_yaml
 116 |     yq eval ".svg_mppi.steer_cov = $steer_cov" -i $tmp_yaml
 117 |     yq eval ".svg_mppi.is_covariance_adaptation = false" -i $tmp_yaml
 118 |     ./eval.sh "pt_svg_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
 119 |     echo "[INFO] Finished evaluating svg_mppi (steer_cov: $steer_cov)"
 120 | done
 121 | 
 122 | ###### Obstacle avoidance ######
 123 | 
 124 | NUM_STATIC_OBSTACLES=5 # With obstacle avoidance
 125 | NUM_TRIALS=100
 126 | for steer_cov in ${STEER_COVS[@]}; do
 127 |     echo "[INFO] Evaluating svg_mppi (steer_cov: $steer_cov)..."
 128 |     cp $default_yaml $tmp_yaml
 129 |     yq eval '.is_visualize_mppi = false' -i $tmp_yaml
 130 |     yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
 131 |     yq eval ".svg_mppi.steer_cov = $steer_cov" -i $tmp_yaml
 132 |     yq eval ".svg_mppi.is_covariance_adaptation = false" -i $tmp_yaml
 133 |     ./eval.sh "oa_svg_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
 134 |     echo "[INFO] Finished evaluating svg_mppi (steer_cov: $steer_cov)"
 135 | done
 136 | 
 137 | ###########################
 138 | # Abalation study: without nominal solution
 139 | ###########################
 140 | 
 141 | ###### Path tracking ######
 142 | NUM_STATIC_OBSTACLES=0 # No obstacle avoidance
 143 | NUM_TRIALS=30
 144 | echo "[INFO] Evaluating svg_mppi without nominal solution..."
 145 | cp $default_yaml $tmp_yaml
 146 | yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
 147 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
 148 | yq eval ".svg_mppi.is_use_nominal_solution = false" -i $tmp_yaml
 149 | ./eval.sh "pt_svg_mppi_wo_nominal" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
 150 | echo "[INFO] Finished evaluating svg_mppi without nominal solution"
 151 | 
 152 | ###### Obstacle avoidance ######
 153 | NUM_STATIC_OBSTACLES=5 # With obstacle avoidance
 154 | NUM_TRIALS=100
 155 | echo "[INFO] Evaluating svg_mppi without nominal solution..."
 156 | cp $default_yaml $tmp_yaml
 157 | yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
 158 | yq eval '.is_visualize_mppi = false' -i $tmp_yaml
 159 | yq eval ".svg_mppi.is_use_nominal_solution = false" -i $tmp_yaml
 160 | ./eval.sh "oa_svg_mppi_wo_nominal" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
 161 | echo "[INFO] Finished evaluating svg_mppi without nominal solution"

```

`script\launch_controllers.sh`:

```sh
   1 | #!/bin/bash
   2 | 
   3 | MAP_NAME=$1
   4 | MPPI_PARAM_PATH=$2
   5 | IS_VISUALIZE=$3
   6 | 
   7 | 
   8 | if [[ "$1" == "-h" || "$1" == "--help" ]]; then
   9 |     echo "Usage: $0 <map_name> <mppi_param_path> <is_visualize>"
  10 |     echo "<map_name> is optional. Default is 'berlin'."
  11 |     echo "<mppi_param_path> is optional. default is src/mppi_controller/config/mppi_controller.yaml"
  12 |     echo "<is_visualize> is optional. default is true"
  13 |     exit 1
  14 | fi
  15 | 
  16 | if [ -z $MAP_NAME ]; then
  17 |     MAP_NAME="berlin"
  18 | fi
  19 | 
  20 | if [ -z "$MPPI_PARAM_PATH" ]; then
  21 |     MPPI_PARAM_PATH="default"
  22 | fi
  23 | 
  24 | if [ -z "$IS_VISUALIZE" ]; then
  25 |     IS_VISUALIZE=true
  26 | fi
  27 | 
  28 | # get location of root workspace
  29 | ROOT_WS=$(cd $(dirname $0) && cd .. && pwd) # recommended to use "~/suzlab_ws"
  30 | 
  31 | # source ros & root project
  32 | source /opt/ros/noetic/setup.bash;
  33 | source $ROOT_WS/devel/setup.bash;
  34 | 
  35 | # reset env
  36 | $ROOT_WS/script/reset_env.sh&
  37 | 
  38 | # launch nodes
  39 | if [ "$MPPI_PARAM_PATH" == "default" ]; then
  40 |     cd $ROOT_WS && roslaunch launch/simulation_launcher.launch workspace:=$ROOT_WS map_name:=$MAP_NAME use_rviz:=$IS_VISUALIZE
  41 | else
  42 |     cd $ROOT_WS && roslaunch launch/simulation_launcher.launch workspace:=$ROOT_WS map_name:=$MAP_NAME mppi_param_path:=$MPPI_PARAM_PATH use_rviz:=$IS_VISUALIZE
  43 | fi

```

`script\launch_simulator.sh`:

```sh
   1 | #!/bin/bash
   2 | 
   3 | MAP_NAME=$1
   4 | NUM_STATIC_OBSTACLES=$2
   5 | 
   6 | if [[ "$1" == "-h" || "$1" == "--help" ]]; then
   7 |     echo "Usage: $0 <map_name> <num_static_obstacles>"
   8 |     echo "<map_name> is optional. Default is 'berlin'."
   9 |     echo "<num_static_obstacles> is optional. Default is 5."
  10 |     exit 1
  11 | fi
  12 | 
  13 | if [ -z $MAP_NAME ]; then
  14 |     MAP_NAME="berlin"
  15 | fi
  16 | 
  17 | if [ -z "$NUM_STATIC_OBSTACLES" ]; then
  18 |     NUM_STATIC_OBSTACLES=5
  19 | fi
  20 | 
  21 | # get location of root workspace
  22 | ROOT_WS=$(cd $(dirname $0) && cd .. && pwd) 
  23 | 
  24 | # copy map data from root to ROOT_WS
  25 | map_dir=$ROOT_WS/data/map/$MAP_NAME
  26 | if [ ! -d $map_dir ]; then
  27 |     echo "[ERROR] map data does not exist: $map_dir"
  28 |     exit 1
  29 | fi
  30 | cp $ROOT_WS/data/map/$MAP_NAME/map.png  $ROOT_WS/src/simulator/maps/map.png;
  31 | cp $ROOT_WS/data/map/$MAP_NAME/map.yaml $ROOT_WS/src/simulator/maps/map.yaml;
  32 | cp $ROOT_WS/data/reference_path/$MAP_NAME/opp_ref_path.csv $ROOT_WS/src/simulator/maps/opp_ref_path.csv
  33 | 
  34 | # launch simulator nodes 
  35 | echo "[INFO] launch gym simulator";
  36 | gnome-terminal --title="gym_ros1_wrapper" -- bash -c "source /opt/ros/noetic/setup.bash; source $ROOT_WS/devel/setup.sh; roslaunch f1tenth_gym_ros agent_template.launch" \
  37 |     && sleep 1s;
  38 | 
  39 | # cleaning up in advance
  40 | DOCKER_STATUS=`docker inspect --format='{{.State.Status}}' f1tenth_gym_container`;
  41 | if [ $DOCKER_STATUS == "running" ]; then
  42 |     echo "[INFO] launch gym environment on a docker container.";
  43 |     docker rm -f f1tenth_gym_container;
  44 | fi
  45 | 
  46 | # setup gym environment
  47 | echo "[INFO] build and launch gym environment on a docker container.";
  48 | cd $ROOT_WS/src/simulator && $ROOT_WS/src/simulator/build_docker.sh&& $ROOT_WS/src/simulator/docker.sh $NUM_STATIC_OBSTACLES;

```

`script\reset_env.sh`:

```sh
   1 | #!/bin/bash
   2 | 
   3 | # set ego states
   4 | # rosparam set /ego_initial_x 0.7
   5 | # rosparam set /ego_initial_y 0.0
   6 | # rosparam set /ego_initial_theta 0.1
   7 | rosparam set /ego_initial_x 0.0
   8 | rosparam set /ego_initial_y -1.0
   9 | rosparam set /ego_initial_theta 3.14
  10 | 
  11 | # set opp states
  12 | rosparam set /opp_initial_x 100.0
  13 | rosparam set /opp_initial_y 0.0
  14 | rosparam set /opp_initial_theta 1.57
  15 | 
  16 | # set seed to rosparam
  17 | rosparam set /seed 0
  18 | 
  19 | # reset gym environment
  20 | rostopic pub -1 /reset_gym_env std_msgs/Bool "data: True"

```

`src\custom_msgs\mppi_metrics_msgs\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.0.2)
   2 | project(mppi_metrics_msgs)
   3 | 
   4 | find_package(catkin REQUIRED COMPONENTS
   5 |   message_generation
   6 |   std_msgs
   7 | )
   8 | 
   9 | ## Generate messages in the 'msg' folder
  10 | add_message_files(
  11 |   FILES
  12 |   MPPIMetrics.msg
  13 | )
  14 | 
  15 | 
  16 | ## Generate added messages and services with any dependencies listed here
  17 | generate_messages(
  18 |   DEPENDENCIES
  19 |   std_msgs
  20 | )
  21 | 
  22 | catkin_package(
  23 | #  INCLUDE_DIRS include
  24 | #  LIBRARIES mppi_metrics_msgs
  25 |  CATKIN_DEPENDS message_generation std_msgs message_runtime
  26 | #  DEPENDS system_lib
  27 | )
  28 | 
  29 | ###########
  30 | ## Build ##
  31 | ###########
  32 | 
  33 | ## Specify additional locations of header files
  34 | ## Your package locations should be listed before other locations
  35 | include_directories(
  36 | # include
  37 |   ${catkin_INCLUDE_DIRS}
  38 | )
  39 | 
  40 | ## Declare a C++ library
  41 | # add_library(${PROJECT_NAME}
  42 | #   src/${PROJECT_NAME}/mppi_metrics_msgs.cpp
  43 | # )
  44 | 
  45 | ## Add cmake target dependencies of the library
  46 | ## as an example, code may need to be generated before libraries
  47 | ## either from message generation or dynamic reconfigure
  48 | # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  49 | 
  50 | ## Declare a C++ executable
  51 | ## With catkin_make all packages are built within a single CMake context
  52 | ## The recommended prefix ensures that target names across packages don't collide
  53 | # add_executable(${PROJECT_NAME}_node src/mppi_metrics_msgs_node.cpp)
  54 | 
  55 | ## Rename C++ executable without prefix
  56 | ## The above recommended prefix causes long target names, the following renames the
  57 | ## target back to the shorter version for ease of user use
  58 | ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
  59 | # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
  60 | 
  61 | ## Add cmake target dependencies of the executable
  62 | ## same as for the library above
  63 | # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  64 | 
  65 | ## Specify libraries to link a library or executable target against
  66 | # target_link_libraries(${PROJECT_NAME}_node
  67 | #   ${catkin_LIBRARIES}
  68 | # )
  69 | 
  70 | #############
  71 | ## Install ##
  72 | #############
  73 | 
  74 | # all install targets should use catkin DESTINATION variables
  75 | # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
  76 | 
  77 | ## Mark executable scripts (Python etc.) for installation
  78 | ## in contrast to setup.py, you can choose the destination
  79 | # catkin_install_python(PROGRAMS
  80 | #   scripts/my_python_script
  81 | #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  82 | # )
  83 | 
  84 | ## Mark executables for installation
  85 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
  86 | # install(TARGETS ${PROJECT_NAME}_node
  87 | #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  88 | # )
  89 | 
  90 | ## Mark libraries for installation
  91 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
  92 | # install(TARGETS ${PROJECT_NAME}
  93 | #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  94 | #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  95 | #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  96 | # )
  97 | 
  98 | ## Mark cpp header files for installation
  99 | # install(DIRECTORY include/${PROJECT_NAME}/
 100 | #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 101 | #   FILES_MATCHING PATTERN "*.h"
 102 | #   PATTERN ".svn" EXCLUDE
 103 | # )
 104 | 
 105 | ## Mark other files for installation (e.g. launch and bag files, etc.)
 106 | # install(FILES
 107 | #   # myfile1
 108 | #   # myfile2
 109 | #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 110 | # )
 111 | 
 112 | #############
 113 | ## Testing ##
 114 | #############
 115 | 
 116 | ## Add gtest based cpp test target and link libraries
 117 | # catkin_add_gtest(${PROJECT_NAME}-test test/test_mppi_metrics_msgs.cpp)
 118 | # if(TARGET ${PROJECT_NAME}-test)
 119 | #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 120 | # endif()
 121 | 
 122 | ## Add folders to be run by python nosetests
 123 | # catkin_add_nosetests(test)

```

`src\custom_msgs\mppi_metrics_msgs\msg\MPPIMetrics.msg`:

```msg
   1 | std_msgs/Header header
   2 | float32 state_cost
   3 | float32 collision_cost
   4 | float32 input_error
   5 | float32 calculation_time

```

`src\custom_msgs\mppi_metrics_msgs\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>mppi_metrics_msgs</name>
   4 |   <version>0.0.0</version>
   5 |   <description>The mppi_result package</description>
   6 | 
   7 |   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8 |   <!-- Example:  -->
   9 |   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 |   <maintainer email="honda@todo.todo">Kohei Honda</maintainer>
  11 | 
  12 | 
  13 |   <!-- One license tag required, multiple allowed, one license per tag -->
  14 |   <!-- Commonly used license strings: -->
  15 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  16 |   <license>TODO</license>
  17 | 
  18 | 
  19 |   <!-- Url tags are optional, but multiple are allowed, one per tag -->
  20 |   <!-- Optional attribute type can be: website, bugtracker, or repository -->
  21 |   <!-- Example: -->
  22 |   <!-- <url type="website">http://wiki.ros.org/mppi_result</url> -->
  23 | 
  24 | 
  25 |   <!-- Author tags are optional, multiple are allowed, one per tag -->
  26 |   <!-- Authors do not have to be maintainers, but could be -->
  27 |   <!-- Example: -->
  28 |   <!-- <author email="jane.doe@example.com">Jane Doe</author> -->
  29 | 
  30 | 
  31 |   <!-- The *depend tags are used to specify dependencies -->
  32 |   <!-- Dependencies can be catkin packages or system dependencies -->
  33 |   <!-- Examples: -->
  34 |   <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  35 |   <!--   <depend>roscpp</depend> -->
  36 |   <!--   Note that this is equivalent to the following: -->
  37 |   <!--   <build_depend>roscpp</build_depend> -->
  38 |   <!--   <exec_depend>roscpp</exec_depend> -->
  39 |   <!-- Use build_depend for packages you need at compile time: -->
  40 |   <!--   <build_depend>message_generation</build_depend> -->
  41 |   <!-- Use build_export_depend for packages you need in order to build against this package: -->
  42 |   <!--   <build_export_depend>message_generation</build_export_depend> -->
  43 |   <!-- Use buildtool_depend for build tool packages: -->
  44 |   <!--   <buildtool_depend>catkin</buildtool_depend> -->
  45 |   <!-- Use exec_depend for packages you need at runtime: -->
  46 |   <!--   <exec_depend>message_runtime</exec_depend> -->
  47 |   <!-- Use test_depend for packages you need only for testing: -->
  48 |   <!--   <test_depend>gtest</test_depend> -->
  49 |   <!-- Use doc_depend for packages you need only for building documentation: -->
  50 |   <!--   <doc_depend>doxygen</doc_depend> -->
  51 |   <buildtool_depend>catkin</buildtool_depend>
  52 |   <build_depend>message_generation</build_depend>
  53 |   <build_export_depend>message_generation</build_export_depend>
  54 |   <build_depend>std_msgs</build_depend>
  55 |   <build_export_depend>std_msgs</build_export_depend>
  56 |   <exec_depend>std_msgs</exec_depend>
  57 |   <exec_depend>message_runtime</exec_depend>
  58 | 
  59 | 
  60 |   <!-- The export tag contains other, unspecified, tags -->
  61 |   <export>
  62 |     <!-- Other tools can request additional information be placed here -->
  63 | 
  64 |   </export>
  65 | </package>

```

`src\custom_msgs\waypoint_msg\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.0.2)
   2 | project(waypoint_msgs)
   3 | 
   4 | ## Compile as C++11, supported in ROS Kinetic and newer
   5 | # add_compile_options(-std=c++11)
   6 | 
   7 | ## Find catkin macros and libraries
   8 | ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
   9 | ## is used, also find other catkin packages
  10 | find_package(catkin REQUIRED COMPONENTS
  11 |   geometry_msgs
  12 |   message_generation
  13 |   std_msgs
  14 | )
  15 | 
  16 | ## System dependencies are found with CMake's conventions
  17 | # find_package(Boost REQUIRED COMPONENTS system)
  18 | 
  19 | 
  20 | ## Uncomment this if the package has a setup.py. This macro ensures
  21 | ## modules and global scripts declared therein get installed
  22 | ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
  23 | # catkin_python_setup()
  24 | 
  25 | ################################################
  26 | ## Declare ROS messages, services and actions ##
  27 | ################################################
  28 | 
  29 | ## To declare and build messages, services or actions from within this
  30 | ## package, follow these steps:
  31 | ## * Let MSG_DEP_SET be the set of packages whose message types you use in
  32 | ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
  33 | ## * In the file package.xml:
  34 | ##   * add a build_depend tag for "message_generation"
  35 | ##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
  36 | ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
  37 | ##     but can be declared for certainty nonetheless:
  38 | ##     * add a exec_depend tag for "message_runtime"
  39 | ## * In this file (CMakeLists.txt):
  40 | ##   * add "message_generation" and every package in MSG_DEP_SET to
  41 | ##     find_package(catkin REQUIRED COMPONENTS ...)
  42 | ##   * add "message_runtime" and every package in MSG_DEP_SET to
  43 | ##     catkin_package(CATKIN_DEPENDS ...)
  44 | ##   * uncomment the add_*_files sections below as needed
  45 | ##     and list every .msg/.srv/.action file to be processed
  46 | ##   * uncomment the generate_messages entry below
  47 | ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
  48 | 
  49 | ## Generate messages in the 'msg' folder
  50 | add_message_files(
  51 |   FILES
  52 |   Waypoint.msg
  53 | )
  54 | 
  55 | ## Generate services in the 'srv' folder
  56 | # add_service_files(
  57 | #   FILES
  58 | #   Service1.srv
  59 | #   Service2.srv
  60 | # )
  61 | 
  62 | ## Generate actions in the 'action' folder
  63 | # add_action_files(
  64 | #   FILES
  65 | #   Action1.action
  66 | #   Action2.action
  67 | # )
  68 | 
  69 | ## Generate added messages and services with any dependencies listed here
  70 | generate_messages(
  71 |   DEPENDENCIES
  72 |   geometry_msgs #   std_msgs
  73 | )
  74 | 
  75 | ################################################
  76 | ## Declare ROS dynamic reconfigure parameters ##
  77 | ################################################
  78 | 
  79 | ## To declare and build dynamic reconfigure parameters within this
  80 | ## package, follow these steps:
  81 | ## * In the file package.xml:
  82 | ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
  83 | ## * In this file (CMakeLists.txt):
  84 | ##   * add "dynamic_reconfigure" to
  85 | ##     find_package(catkin REQUIRED COMPONENTS ...)
  86 | ##   * uncomment the "generate_dynamic_reconfigure_options" section below
  87 | ##     and list every .cfg file to be processed
  88 | 
  89 | ## Generate dynamic reconfigure parameters in the 'cfg' folder
  90 | # generate_dynamic_reconfigure_options(
  91 | #   cfg/DynReconf1.cfg
  92 | #   cfg/DynReconf2.cfg
  93 | # )
  94 | 
  95 | ###################################
  96 | ## catkin specific configuration ##
  97 | ###################################
  98 | ## The catkin_package macro generates cmake config files for your package
  99 | ## Declare things to be passed to dependent projects
 100 | ## INCLUDE_DIRS: uncomment this if your package contains header files
 101 | ## LIBRARIES: libraries you create in this project that dependent projects also need
 102 | ## CATKIN_DEPENDS: catkin_packages dependent projects also need
 103 | ## DEPENDS: system dependencies of this project that dependent projects also need
 104 | catkin_package(
 105 | #  INCLUDE_DIRS include
 106 | #  LIBRARIES waypoint_msgs
 107 |  CATKIN_DEPENDS geometry_msgs message_generation std_msgs message_runtime
 108 | #  DEPENDS system_lib
 109 | )
 110 | 
 111 | ###########
 112 | ## Build ##
 113 | ###########
 114 | 
 115 | ## Specify additional locations of header files
 116 | ## Your package locations should be listed before other locations
 117 | include_directories(
 118 | # include
 119 |   ${catkin_INCLUDE_DIRS}
 120 | )
 121 | 
 122 | ## Declare a C++ library
 123 | # add_library(${PROJECT_NAME}
 124 | #   src/${PROJECT_NAME}/waypoint_msgs.cpp
 125 | # )
 126 | 
 127 | ## Add cmake target dependencies of the library
 128 | ## as an example, code may need to be generated before libraries
 129 | ## either from message generation or dynamic reconfigure
 130 | # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 131 | 
 132 | ## Declare a C++ executable
 133 | ## With catkin_make all packages are built within a single CMake context
 134 | ## The recommended prefix ensures that target names across packages don't collide
 135 | # add_executable(${PROJECT_NAME}_node src/waypoint_msgs_node.cpp)
 136 | 
 137 | ## Rename C++ executable without prefix
 138 | ## The above recommended prefix causes long target names, the following renames the
 139 | ## target back to the shorter version for ease of user use
 140 | ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
 141 | # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
 142 | 
 143 | ## Add cmake target dependencies of the executable
 144 | ## same as for the library above
 145 | # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 146 | 
 147 | ## Specify libraries to link a library or executable target against
 148 | # target_link_libraries(${PROJECT_NAME}_node
 149 | #   ${catkin_LIBRARIES}
 150 | # )
 151 | 
 152 | #############
 153 | ## Install ##
 154 | #############
 155 | 
 156 | # all install targets should use catkin DESTINATION variables
 157 | # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
 158 | 
 159 | ## Mark executable scripts (Python etc.) for installation
 160 | ## in contrast to setup.py, you can choose the destination
 161 | # catkin_install_python(PROGRAMS
 162 | #   scripts/my_python_script
 163 | #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 164 | # )
 165 | 
 166 | ## Mark executables for installation
 167 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 168 | # install(TARGETS ${PROJECT_NAME}_node
 169 | #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 170 | # )
 171 | 
 172 | ## Mark libraries for installation
 173 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 174 | # install(TARGETS ${PROJECT_NAME}
 175 | #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 176 | #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 177 | #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 178 | # )
 179 | 
 180 | ## Mark cpp header files for installation
 181 | # install(DIRECTORY include/${PROJECT_NAME}/
 182 | #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 183 | #   FILES_MATCHING PATTERN "*.h"
 184 | #   PATTERN ".svn" EXCLUDE
 185 | # )
 186 | 
 187 | ## Mark other files for installation (e.g. launch and bag files, etc.)
 188 | # install(FILES
 189 | #   # myfile1
 190 | #   # myfile2
 191 | #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 192 | # )
 193 | 
 194 | #############
 195 | ## Testing ##
 196 | #############
 197 | 
 198 | ## Add gtest based cpp test target and link libraries
 199 | # catkin_add_gtest(${PROJECT_NAME}-test test/test_waypoint_msgs.cpp)
 200 | # if(TARGET ${PROJECT_NAME}-test)
 201 | #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 202 | # endif()
 203 | 
 204 | ## Add folders to be run by python nosetests
 205 | # catkin_add_nosetests(test)

```

`src\custom_msgs\waypoint_msg\msg\Waypoint.msg`:

```msg
   1 | std_msgs/Header header
   2 | geometry_msgs/PoseStamped[] poses
   3 | geometry_msgs/TwistStamped[] twists

```

`src\custom_msgs\waypoint_msg\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>waypoint_msgs</name>
   4 |   <version>0.0.0</version>
   5 |   <description>The waypoint_msgs package</description>
   6 | 
   7 |   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8 |   <!-- Example:  -->
   9 |   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 |   <maintainer email="mizuho@todo.todo">mizuho</maintainer>
  11 | 
  12 | 
  13 |   <!-- One license tag required, multiple allowed, one license per tag -->
  14 |   <!-- Commonly used license strings: -->
  15 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  16 |   <license>TODO</license>
  17 | 
  18 | 
  19 |   <!-- Url tags are optional, but multiple are allowed, one per tag -->
  20 |   <!-- Optional attribute type can be: website, bugtracker, or repository -->
  21 |   <!-- Example: -->
  22 |   <!-- <url type="website">http://wiki.ros.org/waypoint_msgs</url> -->
  23 | 
  24 | 
  25 |   <!-- Author tags are optional, multiple are allowed, one per tag -->
  26 |   <!-- Authors do not have to be maintainers, but could be -->
  27 |   <!-- Example: -->
  28 |   <!-- <author email="jane.doe@example.com">Jane Doe</author> -->
  29 | 
  30 | 
  31 |   <!-- The *depend tags are used to specify dependencies -->
  32 |   <!-- Dependencies can be catkin packages or system dependencies -->
  33 |   <!-- Examples: -->
  34 |   <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  35 |   <!--   <depend>roscpp</depend> -->
  36 |   <!--   Note that this is equivalent to the following: -->
  37 |   <!--   <build_depend>roscpp</build_depend> -->
  38 |   <!--   <exec_depend>roscpp</exec_depend> -->
  39 |   <!-- Use build_depend for packages you need at compile time: -->
  40 |   <!--   <build_depend>message_generation</build_depend> -->
  41 |   <!-- Use build_export_depend for packages you need in order to build against this package: -->
  42 |   <!--   <build_export_depend>message_generation</build_export_depend> -->
  43 |   <!-- Use buildtool_depend for build tool packages: -->
  44 |   <!--   <buildtool_depend>catkin</buildtool_depend> -->
  45 |   <!-- Use exec_depend for packages you need at runtime: -->
  46 |   <!--   <exec_depend>message_runtime</exec_depend> -->
  47 |   <!-- Use test_depend for packages you need only for testing: -->
  48 |   <!--   <test_depend>gtest</test_depend> -->
  49 |   <!-- Use doc_depend for packages you need only for building documentation: -->
  50 |   <!--   <doc_depend>doxygen</doc_depend> -->
  51 |   <buildtool_depend>catkin</buildtool_depend>
  52 |   <build_depend>geometry_msgs</build_depend>
  53 |   <build_depend>message_generation</build_depend>
  54 |   <build_export_depend>message_generation</build_export_depend>
  55 |   <build_depend>std_msgs</build_depend>
  56 |   <build_export_depend>geometry_msgs</build_export_depend>
  57 |   <build_export_depend>std_msgs</build_export_depend>
  58 |   <exec_depend>geometry_msgs</exec_depend>
  59 |   <exec_depend>std_msgs</exec_depend>
  60 |   <exec_depend>message_runtime</exec_depend>
  61 | 
  62 | 
  63 |   <!-- The export tag contains other, unspecified, tags -->
  64 |   <export>
  65 |     <!-- Other tools can request additional information be placed here -->
  66 | 
  67 |   </export>
  68 | </package>

```

`src\eval\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.0.2)
   2 | project(eval_local_planner)
   3 | 
   4 | find_package(catkin REQUIRED COMPONENTS 
   5 |     message_generation 
   6 |     rospy 
   7 |     std_msgs
   8 |     mppi_metrics_msgs
   9 |     )
  10 | 
  11 | ## Generate messages in the 'msg' folder
  12 | add_message_files(
  13 |   FILES
  14 |   RaceInfo.msg
  15 | )
  16 | 
  17 | generate_messages(
  18 |   DEPENDENCIES
  19 |   std_msgs
  20 | )
  21 | 
  22 | catkin_package(
  23 |   CATKIN_DEPENDS message_runtime rospy std_msgs
  24 | )
  25 | 
  26 | catkin_install_python(PROGRAMS 
  27 |     src/eval.py 
  28 |     DESTINATION 
  29 |     ${CATKIN_PACKAGE_BIN_DESTINATION})
  30 | 
  31 | install(DIRECTORY launch
  32 |     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  33 | )

```

`src\eval\launch\eval.launch`:

```launch
   1 | <launch>
   2 |   <arg name="trial_num" default="100" />
   3 |   <arg name="log_dir" default="$(find eval_local_planner)/log" />
   4 |   <arg name="eval_name" default="unkonwn" />
   5 |   
   6 |   <node pkg="eval_local_planner" type="eval.py" name="eval">
   7 |     <param name="trial_num" value="$(arg trial_num)" />
   8 |     <param name="log_dir" value="$(arg log_dir)" />
   9 |     <param name="eval_name" value="$(arg eval_name)" />
  10 |   </node>
  11 | </launch>

```

`src\eval\log\ablation_study\wo_cov_adaptation\obstacle_avoidance\result.txt`:

```txt
   1 | ===== collision rate (%) =====
   2 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive  cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
   3 |       7.2        0.8       6.2        1.4           4.0      2.4       6.8       18.6        6.8        9.8       3.0
   4 | 
   5 | ===== success rate (%) =====
   6 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive  cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
   7 |      76.0       98.0      80.0       94.0          84.0     88.0      77.0       21.0       76.0       65.0      89.0
   8 | 
   9 | ===== mean calculation time (ms) =====
  10 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive   cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
  11 | 10.948384  10.969925 10.953152  11.202828     10.996517 11.545736 11.000287  10.419295  10.939156  10.852389 11.100192
  12 | 
  13 | ===== max calculation time (ms) =====
  14 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive   cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
  15 | 56.966064 112.460083  92.96257  66.893768     39.654819 70.466316 38.429596  40.471554  78.337593  36.310429 80.593575
  16 | 
  17 | ===== min calculation time (ms) =====
  18 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive   cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
  19 | 10.059898  10.042743 10.081059  10.224265     10.003488 10.353969 10.238001   9.565791  10.134777   9.970858 10.231398
  20 | 

```

`src\eval\log\ablation_study\wo_cov_adaptation\path_tracking\result.txt`:

```txt
   1 | ===== collision rate (%) =====
   2 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive  cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
   3 |       0.0        0.0       0.0        0.0           0.0      0.0       0.0       16.0        0.0        0.0       0.0
   4 | 
   5 | ===== success rate (%) =====
   6 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive  cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
   7 |     100.0      100.0     100.0      100.0         100.0    100.0     100.0       20.0      100.0      100.0     100.0
   8 | 
   9 | ===== mean calculation time (ms) =====
  10 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive   cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
  11 | 10.713777  10.514701 10.759649  10.976992     11.011889 10.829051 10.858876  10.756008  11.142179  10.758827 11.254325
  12 | 
  13 | ===== max calculation time (ms) =====
  14 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive   cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
  15 | 37.627979  43.152454 37.966225  72.998894     38.327957 18.752865 36.869007  54.289776  58.169537  26.665203 85.898865
  16 | 
  17 | ===== min calculation time (ms) =====
  18 |  cov:0.02  cov:0.005  cov:0.01  cov:0.075  cov:adaptive   cov:0.1  cov:0.03  cov:0.001  cov:0.025  cov:0.015  cov:0.05
  19 | 10.173324     9.8232 10.069316  10.234397     10.309262 10.248606 10.263186   9.753757   10.14784  10.107996  10.23154
  20 | 

```

`src\eval\log\ablation_study\wo_nominal_solution\obstacle_avoidance\result.txt`:

```txt
   1 | ===== collision rate (%) =====
   2 |  w_nominal_solution  wo_nominal_solution
   3 |                 4.0                  0.0
   4 | 
   5 | ===== success rate (%) =====
   6 |  w_nominal_solution  wo_nominal_solution
   7 |                84.0                100.0
   8 | 
   9 | ===== mean calculation time (ms) =====
  10 |  w_nominal_solution  wo_nominal_solution
  11 |           10.996517            11.124281
  12 | 
  13 | ===== max calculation time (ms) =====
  14 |  w_nominal_solution  wo_nominal_solution
  15 |           39.654819            80.511703
  16 | 
  17 | ===== min calculation time (ms) =====
  18 |  w_nominal_solution  wo_nominal_solution
  19 |           10.003488             10.25699
  20 | 

```

`src\eval\log\ablation_study\wo_nominal_solution\path_tracking\result.txt`:

```txt
   1 | ===== collision rate (%) =====
   2 |  w_nominal_solution  wo_nominal_solution
   3 |                 0.0                  9.4
   4 | 
   5 | ===== success rate (%) =====
   6 |  w_nominal_solution  wo_nominal_solution
   7 |               100.0                 62.0
   8 | 
   9 | ===== mean calculation time (ms) =====
  10 |  w_nominal_solution  wo_nominal_solution
  11 |           11.011889            11.555154
  12 | 
  13 | ===== max calculation time (ms) =====
  14 |  w_nominal_solution  wo_nominal_solution
  15 |           38.327957           101.669357
  16 | 
  17 | ===== min calculation time (ms) =====
  18 |  w_nominal_solution  wo_nominal_solution
  19 |           10.309262            10.381723
  20 | 

```

`src\eval\log\obstacle_avoidance\result.txt`:

```txt
   1 | ===== collision rate (%) =====
   2 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)  sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
   3 |                    25.0                     21.0    12.4                   24.4                     13.6          24.8       4.0
   4 | 
   5 | ===== success rate (%) =====
   6 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)  sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
   7 |                    25.0                     32.0    57.0                   27.0                     46.0          25.0      84.0
   8 | 
   9 | ===== mean calculation time (ms) =====
  10 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)   sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
  11 |               12.228044                12.142908 29.08274              12.192884                11.963975     25.991078 10.996517
  12 | 
  13 | ===== max calculation time (ms) =====
  14 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)    sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
  15 |               43.069771                34.249859 50.148918               32.81353                32.079838     55.886452 39.654819
  16 | 
  17 | ===== min calculation time (ms) =====
  18 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)  sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
  19 |               11.383617                11.367415 21.6952              11.400786                11.127115     23.685493 10.003488
  20 | 

```

`src\eval\log\path_tracking\result.txt`:

```txt
   1 | ===== collision rate (%) =====
   2 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)  sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
   3 |                     0.0                      0.0     0.0                    0.0                      0.0           0.0       0.0
   4 | 
   5 | ===== success rate (%) =====
   6 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)  sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
   7 |                   100.0                    100.0   100.0                  100.0                    100.0         100.0     100.0
   8 | 
   9 | ===== mean calculation time (ms) =====
  10 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)    sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
  11 |               12.169051                12.017425 29.181171              11.974373                 11.86973     26.197173 11.011889
  12 | 
  13 | ===== max calculation time (ms) =====
  14 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)    sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
  15 |               28.589848                33.888325 60.481472              29.984585                36.459423     57.253567 38.327957
  16 | 
  17 | ===== min calculation time (ms) =====
  18 |  forward_mppi(cov:0.05)  forward_mppi(cov:0.075)    sv_mpc  forward_mppi(cov:0.1)  forward_mppi(cov:0.025)  reverse_mppi  svg_mppi
  19 |               11.376301                11.285757 22.001921              11.280871                 11.12582     24.063183 10.309262
  20 | 

```

`src\eval\msg\RaceInfo.msg`:

```msg
   1 | Header header
   2 | int32 ego_lap_count
   3 | float32 ego_elapsed_time
   4 | bool ego_collision
   5 | int32 num_obstacles
   6 | bool[] static_obstacle_collisions

```

`src\eval\package.xml`:

```xml
   1 | <package format="2">
   2 |   <name>eval_local_planner</name>
   3 |   <version>0.0.1</version>
   4 |   <description>
   5 |     Evaluation node for local planner
   6 |   </description>
   7 |   <author>Kohei Honda</author>
   8 |   <maintainer email="0905honda@gmail.com">Kohei Honda</maintainer>
   9 |   <license>MIT</license>
  10 | 
  11 |   <buildtool_depend>catkin</buildtool_depend>
  12 |   <build_depend>message_filters</build_depend>
  13 |   <build_depend>rospy</build_depend>
  14 |   <build_depend>std_msgs</build_depend>
  15 |   <build_depend>message_generation</build_depend>
  16 |   <build_depend>mppi_metrics_msgs</build_depend>
  17 | 
  18 |   <build_export_depend>message_filters</build_export_depend>
  19 |   <build_export_depend>rospy</build_export_depend>
  20 |   <build_export_depend>std_msgs</build_export_depend>
  21 |   <build_export_depend>mppi_metrics_msgs</build_export_depend>
  22 |   
  23 |   <exec_depend>message_filters</exec_depend>
  24 |   <exec_depend>rospy</exec_depend>
  25 |   <exec_depend>std_msgs</exec_depend>
  26 |   <exec_depend>message_runtime</exec_depend>
  27 |   <exec_depend>mppi_metrics_msgs</exec_depend>
  28 | </package>

```

`src\eval\script\plot.py`:

```py
   1 | from typing import List, Dict
   2 | import os
   3 | import fire
   4 | import pandas as pd
   5 | from matplotlib import pyplot as plt
   6 | import seaborn as sns
   7 | 
   8 | # to avoid type3 font
   9 | plt.rcParams["pdf.fonttype"] = 42
  10 | plt.rcParams["ps.fonttype"] = 42
  11 | 
  12 | 
  13 | def read_files(result_dir: str) -> Dict[str, pd.DataFrame]:
  14 |     # get all csv path and name in result_dir
  15 |     files = os.listdir(result_dir)
  16 |     csv_files = [file for file in files if file.endswith(".csv")]
  17 | 
  18 |     csv_paths = [os.path.join(result_dir, csv_path) for csv_path in csv_files]
  19 |     csv_names = [os.path.splitext(csv_path)[0] for csv_path in csv_files]
  20 | 
  21 |     data_dict = {}
  22 |     for csv_path, csv_name in zip(csv_paths, csv_names):
  23 |         data_dict[csv_name] = pd.read_csv(csv_path)
  24 | 
  25 |     return data_dict
  26 | 
  27 | 
  28 | def output_result(result_dir: str) -> None:
  29 |     data_dict = read_files(result_dir)
  30 |     OBSTACLE_NUM_PER_TRIAL = 5
  31 |     data_collision = pd.DataFrame()
  32 |     data_success = pd.DataFrame()
  33 |     data_mean_calc_time = pd.DataFrame()
  34 |     data_max_calc_time = pd.DataFrame()
  35 |     data_min_calc_time = pd.DataFrame()
  36 |     for key in data_dict.keys():
  37 |         collision_num = sum(data_dict[key]["collision"])
  38 |         sum_obs_num = len(data_dict[key]["collision"]) * OBSTACLE_NUM_PER_TRIAL
  39 |         collision_rate = collision_num / sum_obs_num * 100
  40 |         data_collision = pd.concat(
  41 |             [data_collision, pd.DataFrame([collision_rate], columns=[key])], axis=1
  42 |         )
  43 | 
  44 |         success = sum(data_dict[key]["success"])
  45 |         success_rate = success / len(data_dict[key]["success"]) * 100
  46 |         data_success = pd.concat(
  47 |             [data_success, pd.DataFrame([success_rate], columns=[key])], axis=1
  48 |         )
  49 | 
  50 |         mean_calc_time = data_dict[key]["mean_calculation_time"].mean()
  51 |         data_mean_calc_time = pd.concat(
  52 |             [data_mean_calc_time, pd.DataFrame([mean_calc_time], columns=[key])], axis=1
  53 |         )
  54 | 
  55 |         max_calc_time = data_dict[key]["max_calculation_time"].max()
  56 |         data_max_calc_time = pd.concat(
  57 |             [data_max_calc_time, pd.DataFrame([max_calc_time], columns=[key])], axis=1
  58 |         )
  59 | 
  60 |         min_calc_time = data_dict[key]["min_calculation_time"].min()
  61 |         data_min_calc_time = pd.concat(
  62 |             [data_min_calc_time, pd.DataFrame([min_calc_time], columns=[key])], axis=1
  63 |         )
  64 | 
  65 |     # save result as txt
  66 |     with open(os.path.join(result_dir, "result.txt"), "w") as f:
  67 |         f.write("===== collision rate (%) =====\n")
  68 |         f.write(data_collision.to_string(index=False))
  69 |         f.write("\n\n")
  70 |         f.write("===== success rate (%) =====\n")
  71 |         f.write(data_success.to_string(index=False))
  72 |         f.write("\n\n")
  73 |         f.write("===== mean calculation time (ms) =====\n")
  74 |         f.write(data_mean_calc_time.to_string(index=False))
  75 |         f.write("\n\n")
  76 |         f.write("===== max calculation time (ms) =====\n")
  77 |         f.write(data_max_calc_time.to_string(index=False))
  78 |         f.write("\n\n")
  79 |         f.write("===== min calculation time (ms) =====\n")
  80 |         f.write(data_min_calc_time.to_string(index=False))
  81 |         f.write("\n\n")
  82 | 
  83 | 
  84 | def single_cost_box(result_dir: str, ablation_study: str = None) -> None:
  85 |     data_dict = read_files(result_dir)
  86 |     data = pd.DataFrame()
  87 | 
  88 |     # print result
  89 |     output_result(result_dir)
  90 | 
  91 |     # plot cost
  92 |     for key in data_dict.keys():
  93 |         data = pd.concat(
  94 |             [
  95 |                 data,
  96 |                 data_dict[key]["state_cost"] + data_dict[key]["collision_cost"],
  97 |             ],
  98 |             axis=1,
  99 |         )
 100 |     data.columns = [key for key in data_dict.keys()]
 101 | 
 102 |     # IQR removal
 103 |     # Sometimes the simulator glitches on some trials and you end up dizzy in the wall.
 104 |     # Q1 = data.quantile(0.25)
 105 |     # Q3 = data.quantile(0.75)
 106 |     # IQR = Q3 - Q1
 107 |     # data = data[~((data < (Q1 - 1.5 * IQR)) | (data > (Q3 + 1.5 * IQR))).any(axis=1)]
 108 | 
 109 |     if ablation_study is None:
 110 |         # reordering
 111 |         # For comparison with baselines
 112 |         align_data = pd.DataFrame(
 113 |             {
 114 |                 "MPPI\n(cov:0.025)": data["forward_mppi(cov:0.025)"],
 115 |                 # "forward_mppi\n(cov:0.05)": data["forward_mppi(cov:0.05)"],
 116 |                 "MPPI\n(cov:0.075)": data["forward_mppi(cov:0.075)"],
 117 |                 "MPPI\n(cov:0.1)": data["forward_mppi(cov:0.1)"],
 118 |                 "Reverse-MPPI": data["reverse_mppi"],
 119 |                 "SV-MPC": data["sv_mpc"],
 120 |                 "SVG-MPPI\n(Ours)": data["svg_mppi"],
 121 |             }
 122 |         )
 123 |     elif ablation_study == "cov":
 124 |         # For ablation study
 125 |         align_data = pd.DataFrame(
 126 |             {
 127 |                 # "cov:0.001": data["cov:0.001"],
 128 |                 "0.01": data["cov:0.01"],
 129 |                 "0.015": data["cov:0.015"],
 130 |                 "0.02": data["cov:0.02"],
 131 |                 "0.025": data["cov:0.025"],
 132 |                 "0.03": data["cov:0.03"],
 133 |                 "0.05": data["cov:0.05"],
 134 |                 "0.075": data["cov:0.075"],
 135 |                 "0.1": data["cov:0.1"],
 136 |                 "adaptive\n(SVG-MPPI)": data["cov:adaptive"],
 137 |             }
 138 |         )
 139 |     elif ablation_study == "nom":
 140 |         align_data = pd.DataFrame(
 141 |             {
 142 |                 "wo_nominal_solution": data["wo_nominal_solution"],
 143 |                 "w_nominal_solution": data["w_nominal_solution"],
 144 |             }
 145 |         )
 146 | 
 147 |     means = align_data.mean()
 148 | 
 149 |     plt.figure(figsize=(20, 12))
 150 | 
 151 |     sns.set_theme(style="whitegrid")
 152 |     sns.set_context("poster")
 153 |     sns.set_style("ticks")
 154 |     sns.set_palette("Set2")
 155 |     sns.boxplot(
 156 |         data=align_data,
 157 |         showmeans=True,
 158 |         meanprops={
 159 |             "marker": "o",
 160 |             "markerfacecolor": "black",
 161 |             "markeredgecolor": "black",
 162 |             "markersize": 10,
 163 |         },
 164 |         showfliers=False,
 165 |     )
 166 | 
 167 |     # plot mean
 168 |     for i, mean in enumerate(means):
 169 |         # plt.scatter(i, mean, marker="o", color="black", s=50)
 170 |         plt.text(
 171 |             i,
 172 |             mean + 0.01,
 173 |             # mean + 0.15,
 174 |             "{:.2f}".format(mean),
 175 |             ha="right",
 176 |             va="center",
 177 |             fontsize=25,
 178 |         )
 179 | 
 180 |     # plt.yscale("log")
 181 |     if ablation_study == "cov":
 182 |         plt.xlabel("Covariance of steering angle [rad]", fontsize=30)
 183 |     else:
 184 |         plt.xlabel("Algorithm", fontsize=30)
 185 |     plt.ylabel("Mean trajectory state cost per trial", fontsize=30)
 186 | 
 187 |     # save as pdf
 188 |     path = os.path.join(result_dir, "cost_box.pdf")
 189 |     plt.savefig(path, bbox_inches="tight")
 190 | 
 191 |     plt.show()
 192 | 
 193 | 
 194 | def double_cost_box(
 195 |     path_tracking_result_dir: str, obstacle_avoidance_result_dir: str, save_dir: str
 196 | ) -> None:
 197 |     pt_data_dict = read_files(path_tracking_result_dir)
 198 |     oa_data_dict = read_files(obstacle_avoidance_result_dir)
 199 | 
 200 |     # print result
 201 |     output_result(path_tracking_result_dir)
 202 |     output_result(obstacle_avoidance_result_dir)
 203 | 
 204 |     # plot cost
 205 |     pt_data = pd.DataFrame()
 206 |     for key in pt_data_dict.keys():
 207 |         pt_data = pd.concat(
 208 |             [
 209 |                 pt_data,
 210 |                 pt_data_dict[key]["state_cost"] + pt_data_dict[key]["collision_cost"],
 211 |             ],
 212 |             axis=1,
 213 |         )
 214 |     pt_data.columns = [key for key in pt_data_dict.keys()]
 215 | 
 216 |     oa_data = pd.DataFrame()
 217 |     for key in oa_data_dict.keys():
 218 |         oa_data = pd.concat(
 219 |             [
 220 |                 oa_data,
 221 |                 oa_data_dict[key]["state_cost"] + oa_data_dict[key]["collision_cost"],
 222 |             ],
 223 |             axis=1,
 224 |         )
 225 |     oa_data.columns = [key for key in oa_data_dict.keys()]
 226 | 
 227 |     # reordering
 228 |     # For comparison with baselines
 229 |     pt_align_data = pd.DataFrame(
 230 |         {
 231 |             "MPPI\n(cov:0.025)": pt_data["forward_mppi(cov:0.025)"],
 232 |             "MPPI\n(cov:0.075)": pt_data["forward_mppi(cov:0.075)"],
 233 |             "MPPI\n(cov:0.1)": pt_data["forward_mppi(cov:0.1)"],
 234 |             "Reverse-MPPI": pt_data["reverse_mppi"],
 235 |             "SV-MPC": pt_data["sv_mpc"],
 236 |             "SVG-MPPI\n(Ours)": pt_data["svg_mppi"],
 237 |         }
 238 |     )
 239 |     pt_means = pt_align_data.mean()
 240 | 
 241 |     oa_align_data = pd.DataFrame(
 242 |         {
 243 |             "MPPI\n(cov:0.025)": oa_data["forward_mppi(cov:0.025)"],
 244 |             "MPPI\n(cov:0.075)": oa_data["forward_mppi(cov:0.075)"],
 245 |             "MPPI\n(cov:0.1)": oa_data["forward_mppi(cov:0.1)"],
 246 |             "Reverse-MPPI": oa_data["reverse_mppi"],
 247 |             "SV-MPC": oa_data["sv_mpc"],
 248 |             "SVG-MPPI\n(Ours)": oa_data["svg_mppi"],
 249 |         }
 250 |     )
 251 |     oa_means = oa_align_data.mean()
 252 | 
 253 |     # align like seaborn dataset tips format
 254 |     pt_align_data = pd.melt(pt_align_data, var_name="Algorithm", value_name="Cost (PT)")
 255 |     oa_align_data = pd.melt(oa_align_data, var_name="Algorithm", value_name="Cost (OA)")
 256 | 
 257 |     # pt/oaの列を追加
 258 |     align_data = pd.concat([pt_align_data, oa_align_data], axis=0)
 259 |     align_data["Scenario"] = ["PT"] * len(pt_align_data) + ["OA"] * len(oa_align_data)
 260 | 
 261 |     fig, ax1 = plt.subplots(figsize=(30, 12))
 262 | 
 263 |     sns.set_theme(style="whitegrid")
 264 |     sns.set_context("poster")
 265 |     sns.set_style("ticks")
 266 |     sns.set_palette("Set2")
 267 | 
 268 |     sns.boxplot(
 269 |         x="Algorithm",
 270 |         y="Cost (PT)",
 271 |         hue="Scenario",
 272 |         data=align_data,
 273 |         showmeans=True,
 274 |         meanprops={
 275 |             "marker": "o",
 276 |             "markerfacecolor": "black",
 277 |             "markeredgecolor": "black",
 278 |             "markersize": 10,
 279 |         },
 280 |         showfliers=False,
 281 |         ax=ax1,
 282 |     )
 283 | 
 284 |     ax2 = ax1.twinx()
 285 |     sns.boxplot(
 286 |         x="Algorithm",
 287 |         y="Cost (OA)",
 288 |         hue="Scenario",
 289 |         data=align_data,
 290 |         showmeans=True,
 291 |         meanprops={
 292 |             "marker": "o",
 293 |             "markerfacecolor": "black",
 294 |             "markeredgecolor": "black",
 295 |             "markersize": 10,
 296 |         },
 297 |         showfliers=False,
 298 |         ax=ax2,
 299 |     )
 300 | 
 301 |     # delete x label
 302 |     ax1.set_xlabel("")
 303 |     ax2.set_xlabel("")
 304 | 
 305 |     # set legend size
 306 |     handles, labels = ax1.get_legend_handles_labels()
 307 |     ax1.legend(handles=handles[0:6], labels=labels[0:6], fontsize=40, loc="upper right")
 308 |     ax2.legend(
 309 |         handles=handles[6:12], labels=labels[6:12], fontsize=40, loc="upper right"
 310 |     )
 311 | 
 312 |     ax2.set_ylim(0, 55.0)
 313 |     ax1.set_ylim(0, 6.5)
 314 | 
 315 |     # ax1.set_xlabel("Algorithm", fontsize=30)
 316 |     ax1.set_ylabel("Mean sequence state cost per lap (PT)", fontsize=40)
 317 |     ax2.set_ylabel("Mean sequence state cost per lap (OA)", fontsize=40)
 318 | 
 319 |     ax1.tick_params(labelsize=45)
 320 |     ax2.tick_params(labelsize=45)
 321 | 
 322 |     # text mean
 323 |     for i, mean in enumerate(pt_means):
 324 |         ax1.text(
 325 |             i,
 326 |             mean + 0.15,
 327 |             "{:.2f}".format(mean),
 328 |             ha="right",
 329 |             va="center",
 330 |             fontsize=45,
 331 |         )
 332 |     for i, mean in enumerate(oa_means):
 333 |         ax2.text(
 334 |             i,
 335 |             mean + 1.5,
 336 |             "{:.2f}".format(mean),
 337 |             ha="left",
 338 |             va="center",
 339 |             fontsize=45,
 340 |         )
 341 | 
 342 |     # save as pdf
 343 |     path = os.path.join(save_dir, "comparison_with_baselines.pdf")
 344 |     plt.savefig(path, bbox_inches="tight")
 345 | 
 346 |     plt.show()
 347 | 
 348 | 
 349 | if __name__ == "__main__":
 350 |     fire.Fire(double_cost_box)

```

`src\eval\src\eval.py`:

```py
   1 | #!/usr/bin/env python
   2 | # -*- coding: utf-8 -*-
   3 | """Evaluate local planner on a simulation."""
   4 | import os
   5 | import rospy
   6 | import time
   7 | import csv
   8 | from std_msgs.msg import Bool, Float32
   9 | import message_filters
  10 | from eval_local_planner.msg import RaceInfo
  11 | from mppi_metrics_msgs.msg import MPPIMetrics
  12 | 
  13 | 
  14 | class Evaluator:
  15 |     def __init__(self, log_dir: str, eval_name: str, trial_num: int = 100):
  16 |         # existence check of log_dir
  17 |         if not os.path.exists(log_dir):
  18 |             raise FileNotFoundError("log_dir: {} does not exist.".format(log_dir))
  19 | 
  20 |         self._log_dir = log_dir
  21 |         self._eval_name = eval_name
  22 | 
  23 |         self.TRIAL_NUM = trial_num
  24 |         self.MAX_ELAPSED_TIME = 50.0
  25 | 
  26 |         self._lock = False
  27 |         self._done_initial_reset = False
  28 | 
  29 |         self._num_of_trials = 0
  30 |         self._success_list = [0] * self.TRIAL_NUM
  31 |         self._num_obstacles = 0
  32 |         self._static_obstacles_collision_patterns = []
  33 |         self._collision_list = [0] * self.TRIAL_NUM
  34 |         self._timeout_list = [0] * self.TRIAL_NUM
  35 |         self._seed_list = [0] * self.TRIAL_NUM
  36 |         self._state_cost_list_per_lap = []
  37 |         self._collision_cost_list_per_lap = []
  38 |         self._input_error_list_per_lap = []
  39 |         self._mean_state_cost_list = [1e6] * self.TRIAL_NUM
  40 |         self._mean_collision_cost_list = [1e6] * self.TRIAL_NUM
  41 |         self._mean_input_error_list = [1e6] * self.TRIAL_NUM
  42 |         self._calculation_time_per_lap = []
  43 |         self._mean_calculation_time_list = [0.0] * self.TRIAL_NUM
  44 |         self._max_calculation_time_list = [0.0] * self.TRIAL_NUM
  45 |         self._min_calculation_time_list = [1e6] * self.TRIAL_NUM
  46 | 
  47 |         # Pub and Sub
  48 |         race_info_topic = "race_info"
  49 |         self._sub_race_info = message_filters.Subscriber(
  50 |             race_info_topic, RaceInfo, queue_size=1
  51 |         )
  52 | 
  53 |         mppi_metrics_topic = "mppi/eval_metrics"
  54 |         self._mppi_metrics_sub = message_filters.Subscriber(
  55 |             mppi_metrics_topic, MPPIMetrics, queue_size=1
  56 |         )
  57 | 
  58 |         ts = message_filters.ApproximateTimeSynchronizer(
  59 |             [self._sub_race_info, self._mppi_metrics_sub],
  60 |             10,
  61 |             0.1,
  62 |             allow_headerless=False,
  63 |         )
  64 |         ts.registerCallback(self._callback)
  65 | 
  66 |         reset_env_topic = "reset_gym_env"
  67 |         self._pub_reset_env = rospy.Publisher(reset_env_topic, Bool, queue_size=1)
  68 | 
  69 |     def _callback(self, race_info_msg, mppi_metrics_msg):
  70 |         if not self._done_initial_reset:
  71 |             init_seed = 0
  72 |             self._reset_env(seed=init_seed)
  73 |             self._num_of_trials = 0
  74 |             self._seed_list[self._num_of_trials] = init_seed
  75 |             self._done_initial_reset = True
  76 |             self._static_obstacles_collision_patterns = []
  77 | 
  78 |         if self._num_of_trials >= self.TRIAL_NUM:
  79 |             self._end_eval()
  80 |             return
  81 | 
  82 |         if self._lock:
  83 |             return
  84 |         else:
  85 |             self._lock = True
  86 | 
  87 |         ego_collision = race_info_msg.ego_collision
  88 |         lap_count = race_info_msg.ego_lap_count
  89 |         elapsed_time = race_info_msg.ego_elapsed_time
  90 |         self._num_obstacles = race_info_msg.num_obstacles
  91 |         self._state_cost_list_per_lap.append(mppi_metrics_msg.state_cost)
  92 |         self._collision_cost_list_per_lap.append(mppi_metrics_msg.collision_cost)
  93 |         self._input_error_list_per_lap.append(mppi_metrics_msg.input_error)
  94 |         self._calculation_time_per_lap.append(mppi_metrics_msg.calculation_time)
  95 |         if ego_collision:
  96 |             collision_pattern = race_info_msg.static_obstacle_collisions
  97 |             if collision_pattern not in self._static_obstacles_collision_patterns:
  98 |                 # Check if the collision pattern is not already registered
  99 |                 self._static_obstacles_collision_patterns.append(collision_pattern)
 100 | 
 101 |         done = False
 102 |         # End of lap
 103 |         if lap_count > 0:
 104 |             if len(self._static_obstacles_collision_patterns) == 0:
 105 |                 self._success_list[self._num_of_trials] = 1
 106 |             done = True
 107 | 
 108 |         # Timeout
 109 |         if elapsed_time > self.MAX_ELAPSED_TIME:
 110 |             self._timeout_list[self._num_of_trials] = 1
 111 |             done = True
 112 | 
 113 |         if done:
 114 |             # count collision patterns
 115 |             self._collision_list[self._num_of_trials] = len(
 116 |                 self._static_obstacles_collision_patterns
 117 |             )
 118 | 
 119 |             # count costs per lap
 120 |             if len(self._state_cost_list_per_lap) == 0:
 121 |                 self._mean_state_cost_list[self._num_of_trials] = 0.0
 122 |                 self._mean_collision_cost_list[self._num_of_trials] = 0.0
 123 |                 self._mean_input_error_list[self._num_of_trials] = 0.0
 124 |             else:
 125 |                 self._mean_state_cost_list[self._num_of_trials] = sum(
 126 |                     self._state_cost_list_per_lap
 127 |                 ) / float(len(self._state_cost_list_per_lap))
 128 | 
 129 |                 self._mean_collision_cost_list[self._num_of_trials] = sum(
 130 |                     self._collision_cost_list_per_lap
 131 |                 ) / float(len(self._collision_cost_list_per_lap))
 132 | 
 133 |                 self._mean_input_error_list[self._num_of_trials] = sum(
 134 |                     self._input_error_list_per_lap
 135 |                 ) / float(len(self._input_error_list_per_lap))
 136 |             self._state_cost_list_per_lap = []
 137 |             self._collision_cost_list_per_lap = []
 138 |             self._input_error_list_per_lap = []
 139 | 
 140 |             # calculate mean/max/min calculation time per lap
 141 |             if len(self._calculation_time_per_lap) == 0:
 142 |                 self._mean_calculation_time_list[self._num_of_trials] = 0.0
 143 |                 self._max_calculation_time_list[self._num_of_trials] = 0.0
 144 |                 self._min_calculation_time_list[self._num_of_trials] = 0.0
 145 |             else:
 146 |                 self._mean_calculation_time_list[self._num_of_trials] = sum(
 147 |                     self._calculation_time_per_lap
 148 |                 ) / float(len(self._calculation_time_per_lap))
 149 |                 self._max_calculation_time_list[self._num_of_trials] = max(
 150 |                     self._calculation_time_per_lap
 151 |                 )
 152 |                 self._min_calculation_time_list[self._num_of_trials] = min(
 153 |                     self._calculation_time_per_lap
 154 |                 )
 155 | 
 156 |             self._num_of_trials += 1
 157 |             self._static_obstacles_collision_patterns = []
 158 | 
 159 |             # seed should be enough larger than previous seed
 160 |             # because the series of seeds are used in the same order
 161 |             if self._num_of_trials < self.TRIAL_NUM:
 162 |                 seed = self._num_of_trials * (self._num_obstacles + 5)
 163 |                 self._seed_list[self._num_of_trials] = seed
 164 |                 self._reset_env(seed=seed)
 165 | 
 166 |         self._lock = False
 167 | 
 168 |     def _publish_reset_topic(self, rate: float = 0.1, duration: float = 0.5):
 169 |         start_time = time.time()
 170 |         while time.time() - start_time < duration:
 171 |             self._pub_reset_env.publish(True)
 172 |             time.sleep(0.1)
 173 | 
 174 |     def _reset_env(self, seed: int):
 175 |         # set rosparam
 176 |         rospy.set_param("/seed", seed)
 177 |         rospy.set_param("/ego_initial_x", 0.0)
 178 |         rospy.set_param("/ego_initial_y", 0.0)
 179 |         rospy.set_param("/ego_initial_theta", 3.14)
 180 |         rospy.set_param("/opp_initial_x", 100.0)
 181 |         rospy.set_param("/opp_initial_y", 0.0)
 182 |         rospy.set_param("/opp_initial_theta", 1.57)
 183 | 
 184 |         if self._num_of_trials != 0:
 185 |             mean_state_cost = 0.0
 186 |             mean_collision_cost = 0.0
 187 |             mean_input_error = 0.0
 188 |             for i in range(self._num_of_trials):
 189 |                 mean_state_cost += self._mean_state_cost_list[i]
 190 |                 mean_collision_cost += self._mean_collision_cost_list[i]
 191 |                 mean_input_error += self._mean_input_error_list[i]
 192 |             mean_state_cost /= float(self._num_of_trials)
 193 |             mean_collision_cost /= float(self._num_of_trials)
 194 |             mean_input_error /= float(self._num_of_trials)
 195 |             rospy.logwarn(
 196 |                 "Seed: {}, Trial num: {} / {}, Collision num: {} / {}, Mean state cost: {}, Mean collision cost: {}, Mean input error: {}".format(
 197 |                     self._seed_list[self._num_of_trials - 1],
 198 |                     self._num_of_trials,
 199 |                     self.TRIAL_NUM,
 200 |                     sum(self._collision_list),
 201 |                     self._num_of_trials * self._num_obstacles,
 202 |                     mean_state_cost,
 203 |                     mean_collision_cost,
 204 |                     mean_input_error,
 205 |                 )
 206 |             )
 207 |         else:
 208 |             rospy.logwarn("Start {} trials.".format(self.TRIAL_NUM))
 209 | 
 210 |         # publish reset topic for 1 seconds
 211 |         self._publish_reset_topic()
 212 | 
 213 |     def _end_eval(self):
 214 |         # Save results to csv
 215 |         file_path = os.path.join(self._log_dir, self._eval_name + ".csv")
 216 |         with open(file_path, "w") as f:
 217 |             writer = csv.writer(f)
 218 |             writer.writerow(
 219 |                 [
 220 |                     "seed",
 221 |                     "success",
 222 |                     "collision",
 223 |                     "timeout",
 224 |                     "state_cost",
 225 |                     "collision_cost",
 226 |                     "input_error",
 227 |                     "mean_calculation_time",
 228 |                     "max_calculation_time",
 229 |                     "min_calculation_time",
 230 |                 ]
 231 |             )
 232 |             for i in range(self.TRIAL_NUM):
 233 |                 writer.writerow(
 234 |                     [
 235 |                         self._seed_list[i],
 236 |                         self._success_list[i],
 237 |                         self._collision_list[i],
 238 |                         self._timeout_list[i],
 239 |                         self._mean_state_cost_list[i],
 240 |                         self._mean_collision_cost_list[i],
 241 |                         self._mean_input_error_list[i],
 242 |                         self._mean_calculation_time_list[i],
 243 |                         self._max_calculation_time_list[i],
 244 |                         self._min_calculation_time_list[i],
 245 |                     ]
 246 |                 )
 247 | 
 248 |         rospy.logwarn_once("Evaluation finished.")
 249 |         rospy.logwarn_once("Trial num: {}".format(self._num_of_trials))
 250 |         rospy.logwarn_once("Success num: {}".format(self._success_list.count(1)))
 251 |         rospy.logwarn_once(
 252 |             "Collision num: {} / {}".format(
 253 |                 sum(self._collision_list), self._num_of_trials * self._num_obstacles
 254 |             )
 255 |         )
 256 |         rospy.logwarn_once("Not goal num: {}".format(self._timeout_list.count(1)))
 257 | 
 258 |         rospy.signal_shutdown("Evaluation finished.")
 259 | 
 260 | 
 261 | if __name__ == "__main__":
 262 |     rospy.init_node("eval_node")
 263 | 
 264 |     # get parameters
 265 |     trial_num = rospy.get_param("eval/trial_num", 100)
 266 |     log_dir = rospy.get_param("eval/log_dir", "")
 267 |     eval_name = rospy.get_param("eval/eval_name", "unknown")
 268 | 
 269 |     rospy.logwarn("Evaluation info: ")
 270 |     rospy.logwarn("Trial num: {}".format(trial_num))
 271 |     rospy.logwarn("Log dir: {}".format(log_dir))
 272 |     rospy.logwarn("Eval name: {}".format(eval_name))
 273 | 
 274 |     eval = Evaluator(log_dir=log_dir, trial_num=trial_num, eval_name=eval_name)
 275 |     rospy.spin()

```

`src\local_costmap_generator\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.13)
   2 | project(local_costmap_generator)
   3 | 
   4 | add_compile_options(-std=c++17)
   5 | add_compile_options(-Wall -Wextra)
   6 | add_compile_options(-O3)
   7 | add_compile_options(-O3 -fopenmp)
   8 | # set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
   9 | if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  10 |   set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  11 |   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  12 | endif()
  13 | 
  14 | ## Find catkin macros and libraries
  15 | ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  16 | ## is used, also find other catkin packages
  17 | find_package(catkin REQUIRED COMPONENTS
  18 |   geometry_msgs
  19 |   std_msgs
  20 |   nav_msgs
  21 |   roscpp
  22 |   tf2_geometry_msgs
  23 |   tf2_ros
  24 |   grid_map_core
  25 |   grid_map_ros
  26 |   grid_map_filters
  27 |   grid_map_loader
  28 |   grid_map_pcl
  29 |   grid_map_msgs
  30 |   grid_map_rviz_plugin
  31 |   grid_map_visualization
  32 |   laser_geometry
  33 |   sensor_msgs
  34 |   voxel_grid
  35 |   pcl_ros
  36 |   pcl_conversions
  37 | )
  38 | 
  39 | find_package(Eigen3 REQUIRED)
  40 | 
  41 | # For OpenMP
  42 | # set(OpenMP_HOME "/usr/lib/llvm-10")
  43 | # set(OpenMP_omp_LIBRARY "${OpenMP_HOME}/lib/")
  44 | # set(OpenMP_C_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
  45 | # set(OpenMP_CXX_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
  46 | # set(OpenMP_C_LIB_NAMES "omp")
  47 | # set(OpenMP_CXX_LIB_NAMES "omp")
  48 | find_package(OpenMP REQUIRED)
  49 | if(OpenMP_FOUND)
  50 |     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  51 |     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  52 | endif()
  53 | 
  54 | catkin_package(
  55 |  INCLUDE_DIRS include
  56 | #  LIBRARIES mppi
  57 | #  CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros
  58 | #  DEPENDS system_lib
  59 | )
  60 | 
  61 | ###########
  62 | ## Build ##
  63 | ###########
  64 | add_executable(${PROJECT_NAME}_node
  65 |   src/local_costmap_generator_node.cpp
  66 |   src/local_costmap_generator.cpp
  67 | )
  68 | 
  69 | ## Specify additional locations of header files
  70 | ## Your package locations should be listed before other locations
  71 | 
  72 | target_include_directories(${PROJECT_NAME}_node PUBLIC
  73 |   include
  74 |   ${catkin_INCLUDE_DIRS}
  75 | )
  76 | 
  77 | ## Add cmake target dependencies of the executable
  78 | ## same as for the library above
  79 | add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  80 | 
  81 | ## Specify libraries to link a library or executable target against
  82 | target_link_libraries(${PROJECT_NAME}_node
  83 |   ${catkin_LIBRARIES}
  84 |  )
  85 | 
  86 | if (OPENMP_FOUND)
  87 |     if (TARGET OpenMP::OpenMP_CXX)
  88 |         target_link_libraries(${PROJECT_NAME}_node OpenMP::OpenMP_CXX)
  89 |     endif ()
  90 | endif ()
  91 | 
  92 | 
  93 |  install(
  94 |   TARGETS
  95 |     ${PROJECT_NAME}_node
  96 |   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  97 |   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  98 |   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  99 | )
 100 | 
 101 | install(
 102 |   DIRECTORY
 103 |     launch
 104 |     config
 105 |   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 106 | )

```

`src\local_costmap_generator\README.md`:

```md
   1 | # local_costmap_generator

```

`src\local_costmap_generator\config\local_costmap_generator.yaml`:

```yaml
   1 | # system
   2 | robot_frame_id: ego_racecar/base_link
   3 | sensor_frame_id: laser
   4 | in_scan_topic: scan
   5 | out_costmap_topic: local_costmap
   6 | 
   7 | # parameters
   8 | 
   9 | ## common
  10 | update_rate: 0.025 # [s]
  11 | thread_num: 2
  12 | 
  13 | ## preprocess setting
  14 | is_crop_robot: true # crop points in the robot body
  15 | 
  16 | is_remove_outlier: false
  17 | sor_mean_k: 10
  18 | sor_stddev_mul_thresh: 1.0
  19 | 
  20 | is_downsample: false
  21 | downsample_resolution: 0.1 # [m]
  22 | 
  23 | is_pass_through: false
  24 | pass_through_min_from_robot: 0.0 # [m]
  25 | pass_through_max_from_robot: 2.0 # [m]
  26 | 
  27 | ## rigid body shape [m] for inflate costmap
  28 | # baselink2front: 0.5
  29 | # baselink2rear: 0.5
  30 | # baselink2right: 0.5
  31 | # baselink2left: 0.5
  32 | # buckup
  33 | baselink2front: 0.3
  34 | baselink2rear: 0.2
  35 | baselink2right: 0.2
  36 | baselink2left: 0.2
  37 | # real car
  38 | # baselink2front: 0.5
  39 | # baselink2rear: 0.15
  40 | # baselink2right: 0.14
  41 | # baselink2left: 0.14
  42 | 
  43 | ## costmap setting
  44 | map_x_length: 20.0 # [m]
  45 | map_y_length: 20.0 # [m]
  46 | map_center_offset_x: 3.0 # [m] map center ahead of robot
  47 | map_center_offset_y: 0.0 # [m] map center left of robot
  48 | map_resolution: 0.05 # [m]
  49 | max_val: 100.0 # max value of costmap, min value is 0.0

```

`src\local_costmap_generator\config\local_costmap_generator_simulation.yaml`:

```yaml
   1 | # system
   2 | robot_frame_id: ego_racecar/base_link
   3 | sensor_frame_id: ego_racecar/laser
   4 | in_scan_topic: scan
   5 | out_costmap_topic: local_costmap
   6 | 
   7 | # parameters
   8 | 
   9 | ## common
  10 | update_rate: 0.01 # [s]
  11 | thread_num: 2
  12 | 
  13 | ## preprocess setting
  14 | is_crop_robot: true # crop points in the robot body
  15 | 
  16 | is_remove_outlier: false
  17 | sor_mean_k: 10
  18 | sor_stddev_mul_thresh: 1.0
  19 | 
  20 | is_downsample: false
  21 | downsample_resolution: 0.1 # [m]
  22 | 
  23 | is_pass_through: false
  24 | pass_through_min_from_robot: 0.0 # [m]
  25 | pass_through_max_from_robot: 2.0 # [m]
  26 | 
  27 | ## rigid body shape [m] for inflate costmap
  28 | baselink2front: 0.47
  29 | baselink2rear: 0.14
  30 | baselink2right: 0.15
  31 | baselink2left: 0.15
  32 | 
  33 | ## costmap setting
  34 | map_x_length: 10.0 # [m]
  35 | map_y_length: 10.0 # [m]
  36 | map_center_offset_x: 3.0 # [m] map center ahead of robot
  37 | map_center_offset_y: 0.0 # [m] map center left of robot
  38 | map_resolution: 0.05 # [m]
  39 | max_val: 100.0 # max value of costmap, min value is 0.0

```

`src\local_costmap_generator\include\local_costmap_generator\local_costmap_generator.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <Eigen/Dense>
   5 | #include <string>
   6 | #include <vector>
   7 | 
   8 | #include <geometry_msgs/PoseStamped.h>
   9 | #include <geometry_msgs/TransformStamped.h>
  10 | #include <nav_msgs/Path.h>
  11 | #include <ros/ros.h>
  12 | #include <std_msgs/Float32.h>
  13 | #include <tf2/utils.h>
  14 | #include <tf2_eigen/tf2_eigen.h>
  15 | #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
  16 | #include <tf2_ros/transform_broadcaster.h>
  17 | #include <tf2_ros/transform_listener.h>
  18 | #include <visualization_msgs/Marker.h>
  19 | #include <visualization_msgs/MarkerArray.h>
  20 | 
  21 | #include <laser_geometry/laser_geometry.h>
  22 | #include <pcl/common/transforms.h>
  23 | #include <pcl/filters/crop_box.h>
  24 | #include <pcl/filters/filter.h>
  25 | #include <pcl/filters/passthrough.h>
  26 | #include <pcl/filters/statistical_outlier_removal.h>
  27 | #include <pcl/filters/voxel_grid.h>
  28 | #include <pcl_conversions/pcl_conversions.h>
  29 | #include <sensor_msgs/LaserScan.h>
  30 | #include <sensor_msgs/PointCloud2.h>
  31 | #include <sensor_msgs/point_cloud_conversion.h>
  32 | #include <voxel_grid/voxel_grid.h>
  33 | 
  34 | #include <grid_map_core/GridMap.hpp>
  35 | #include <grid_map_core/iterators/GridMapIterator.hpp>
  36 | #include <grid_map_filters/MedianFillFilter.hpp>
  37 | #include <grid_map_pcl/GridMapPclLoader.hpp>
  38 | #include <grid_map_pcl/helpers.hpp>
  39 | #include <grid_map_ros/grid_map_ros.hpp>
  40 | 
  41 | namespace local_costmap_generator {
  42 | 
  43 | class LocalCostmapGenerator {
  44 | private:
  45 |     using PointType = pcl::PointXYZ;
  46 | 
  47 | public:
  48 |     LocalCostmapGenerator();
  49 | 
  50 |     ~LocalCostmapGenerator(){};
  51 | 
  52 | private:
  53 |     /* ros system variables */
  54 |     ros::NodeHandle nh_;          //!< @brief ros public node handle
  55 |     ros::NodeHandle private_nh_;  //!< @brief ros private node handle
  56 |     tf2_ros::Buffer tf_buffer_;
  57 |     tf2_ros::TransformListener tf_listener_;
  58 | 
  59 |     /* pub sub */
  60 |     ros::Subscriber sub_scan_;             //!< @brief laser scan subscriber
  61 |     ros::Timer timer_costmap_;             //!< @brief timer for calculate cost map
  62 |     ros::Publisher pub_cost_map_;          //!< @brief cost map publisher
  63 |     ros::Publisher pub_rigid_body_shape_;  //!< @brief rigid body shape publisher
  64 | 
  65 |     // debug
  66 |     ros::Publisher pub_cloud_;
  67 |     // ros::Publisher pub_calc_time_;
  68 | 
  69 |     /*Parameters*/
  70 |     std::string robot_frame_id_;
  71 |     std::string sensor_frame_id_;
  72 |     double update_rate_ = 0.01;  //!< @brief update interval [s]
  73 |     int thread_num_ = 4;         //!< @brief number of threads for cost map calculation
  74 |     const std::string collision_layer_name_ = "collision_layer";
  75 | 
  76 |     // vehicle shape parameters
  77 |     struct RigidBodyShape {
  78 |         double baselink2front = 0.47;  // m
  79 |         double baselink2rear = 0.14;   // m
  80 |         double baselink2right = 0.15;  // m
  81 |         double baselink2left = 0.15;   // m
  82 |     };
  83 |     RigidBodyShape rigid_body_shape_;
  84 | 
  85 |     // preprocess filters
  86 |     bool is_remove_outlier_ = false;
  87 |     pcl::StatisticalOutlierRemoval<PointType> sor_;
  88 |     int sor_mean_k_ = 10;
  89 |     double sor_stddev_mul_thresh_ = 1.0;
  90 | 
  91 |     bool is_downsample_ = false;
  92 |     pcl::VoxelGrid<PointType> voxel_grid_filter_;
  93 |     double downsample_resolution_ = 0.1;
  94 | 
  95 |     bool is_pass_through_ = false;
  96 |     pcl::PassThrough<PointType> pass_through_filter_;
  97 |     double pass_through_min_from_robot_ = 0.0;
  98 |     double pass_through_max_from_robot_ = 2.0;
  99 | 
 100 |     bool is_crop_robot_ = true;
 101 |     pcl::CropBox<PointType> crop_box_filter_;
 102 |     Eigen::Vector4f crop_box_min_;
 103 |     Eigen::Vector4f crop_box_max_;
 104 | 
 105 |     // Inner variables
 106 |     // For scan transform
 107 |     bool is_laser_scan_received_ = false;
 108 |     sensor_msgs::PointCloud2::Ptr raw_pc2_ptr_;
 109 |     laser_geometry::LaserProjection projector_;
 110 |     geometry_msgs::TransformStamped transform_stamped_;
 111 |     Eigen::Matrix4f transform_matrix_;
 112 | 
 113 |     // For cost map calculation
 114 |     pcl::PointCloud<PointType>::Ptr raw_pcl_ptr_;
 115 |     pcl::PointCloud<PointType>::Ptr preprocessed_pcl_ptr_;
 116 |     pcl::PointCloud<PointType>::Ptr trans_preprocessed_pcl_ptr_;
 117 |     double max_val_ = 100.0;      // max value of cost map
 118 |     grid_map::GridMap cost_map_;  // value = [0, max_val_] (0: free, max_val_: occupied)
 119 | 
 120 |     /**
 121 |      * @brief main loop
 122 |      *
 123 |      */
 124 |     void timer_callback(const ros::TimerEvent&);
 125 | 
 126 |     void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
 127 | 
 128 |     /**
 129 |      * @brief preprocess_pointcloud
 130 |      * @param[in] raw_pcl_ptr_ raw point cloud in sensor frame
 131 |      * @param[out] preprocessed_pcl_ptr_ preprocessed point cloud in sensor frame
 132 |      */
 133 |     void preprocess_pointcloud(const pcl::PointCloud<PointType>::ConstPtr& in_pcl_ptr_, pcl::PointCloud<PointType>::Ptr& out_pcl_ptr_);
 134 | 
 135 |     void crop_points_within_robot(pcl::PointCloud<PointType>::Ptr& pcl_ptr_);
 136 | 
 137 |     /**
 138 |      * @brief build occupancy grid map from point cloud
 139 |      * @param[in] preprocessed_pcl_ptr_ preprocessed point cloud in sensor frame
 140 |      * @param[out] cost_map_ occupancy grid map : value = [0, max_val_] (0: free, max_val_: occupied)
 141 |      * @param[out] occupied_indices_ indices of occupied grid cells
 142 |      */
 143 |     std::vector<grid_map::Index> pointcloud_to_costmap(const pcl::PointCloud<PointType>::ConstPtr& in_pcl_ptr, grid_map::GridMap* out_cost_map) const;
 144 | 
 145 |     void inflate_rigid_body(const std::string& layer_name, const std::vector<grid_map::Index>& occupied_indices, grid_map::GridMap* cost_map) const;
 146 | 
 147 |     void publish_rigid_body_shape(const RigidBodyShape& rigid_body_shape) const;
 148 | };
 149 | 
 150 | }  // namespace local_costmap_generator

```

`src\local_costmap_generator\launch\local_costmap_generator.launch`:

```launch
   1 | <launch>
   2 |     <arg name="param_path" default="$(find local_costmap_generator)/config/local_costmap_generator.yaml"/>
   3 | 
   4 |     <node pkg="local_costmap_generator" type="local_costmap_generator_node" name="local_costmap_generator" output="screen">
   5 |         <rosparam command="load" file="$(arg param_path)"/>
   6 |     </node>
   7 | </launch>

```

`src\local_costmap_generator\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>local_costmap_generator</name>
   4 |   <version>0.0.0</version>
   5 |   <description>Model Predictive Path Integral control</description>
   6 | 
   7 |   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8 |   <!-- Example:  -->
   9 |   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 |   <maintainer email="0905honda@gmail.com">Kohei Honda</maintainer>
  11 | 
  12 | 
  13 |   <!-- One license tag required, multiple allowed, one license per tag -->
  14 |   <!-- Commonly used license strings: -->
  15 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  16 |   <license>MIT</license>
  17 | 
  18 |   <buildtool_depend>catkin</buildtool_depend>
  19 |   <build_depend>geometry_msgs</build_depend>
  20 |   <build_depend>nav_msgs</build_depend>
  21 |   <build_depend>roscpp</build_depend>
  22 |   <build_depend>tf2_geometry_msgs</build_depend>
  23 |   <build_depend>tf2_ros</build_depend>
  24 |   <build_depend>grid_map</build_depend>
  25 |   <build_depend>grid_map_ros</build_depend>
  26 |   <build_depend>grid_map_msgs</build_depend>
  27 |   <build_depend>grid_map_rviz_plugin</build_depend>
  28 |   <build_depend>grid_map_visualization</build_depend>
  29 |   <build_depend>grid_map_pcl</build_depend>
  30 |   <build_depend>laser_geometry</build_depend>
  31 |   <build_depend>sensor_msgs</build_depend>
  32 |   <build_depend>voxel_grid</build_depend>
  33 |   <build_depend>pcl_ros</build_depend>
  34 |   <build_depend>pcl_conversions</build_depend>
  35 | 
  36 |   <build_export_depend>geometry_msgs</build_export_depend>
  37 |   <build_export_depend>nav_msgs</build_export_depend>
  38 |   <build_export_depend>roscpp</build_export_depend>
  39 |   <build_export_depend>tf2_geometry_msgs</build_export_depend>
  40 |   <build_export_depend>tf2_ros</build_export_depend>
  41 |   <build_export_depend>grid_map</build_export_depend>
  42 |   <build_export_depend>grid_map_ros</build_export_depend>
  43 |   <build_export_depend>grid_map_msgs</build_export_depend>
  44 |   <build_export_depend>grid_map_rviz_plugin</build_export_depend>
  45 |   <build_export_depend>grid_map_visualization</build_export_depend>
  46 |   <build_export_depend>grid_map_pcl</build_export_depend>
  47 |   <build_export_depend>laser_geometry</build_export_depend>
  48 |   <build_export_depend>sensor_msgs</build_export_depend>
  49 |   <build_export_depend>voxel_grid</build_export_depend>
  50 |   <build_export_depend>pcl_ros</build_export_depend>
  51 |   <build_export_depend>pcl_conversions</build_export_depend>
  52 |   
  53 |   <exec_depend>geometry_msgs</exec_depend>
  54 |   <exec_depend>nav_msgs</exec_depend>
  55 |   <exec_depend>roscpp</exec_depend>
  56 |   <exec_depend>tf2_geometry_msgs</exec_depend>
  57 |   <exec_depend>tf2_ros</exec_depend>
  58 |   <exec_depend>grid_map</exec_depend>
  59 |   <exec_depend>grid_map_ros</exec_depend>
  60 |   <exec_depend>grid_map_msgs</exec_depend>
  61 |   <exec_depend>grid_map_rviz_plugin</exec_depend>
  62 |   <exec_depend>grid_map_visualization</exec_depend>
  63 |   <exec_depend>grid_map_pcl</exec_depend>
  64 |   <exec_depend>laser_geometry</exec_depend>
  65 |   <exec_depend>sensor_msgs</exec_depend>
  66 |   <exec_depend>voxel_grid</exec_depend>
  67 |   <exec_depend>pcl_ros</exec_depend>
  68 |   <exec_depend>pcl_conversions</exec_depend>
  69 | 
  70 |   <!-- The export tag contains other, unspecified, tags -->
  71 |   <export>
  72 |     <!-- Other tools can request additional information be placed here -->
  73 | 
  74 |   </export>
  75 | </package>

```

`src\local_costmap_generator\src\local_costmap_generator.cpp`:

```cpp
   1 | #include "local_costmap_generator/local_costmap_generator.hpp"
   2 | 
   3 | namespace local_costmap_generator {
   4 | 
   5 | LocalCostmapGenerator::LocalCostmapGenerator()
   6 |     : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_), cost_map_(std::vector<std::string>({collision_layer_name_})) {
   7 |     // set parameter from ros parameter server
   8 |     private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
   9 |     private_nh_.param("sensor_frame_id", sensor_frame_id_, static_cast<std::string>("laser"));
  10 |     std::string in_scan_topic;
  11 |     std::string out_costmap_topic;
  12 |     private_nh_.param("in_scan_topic", in_scan_topic, static_cast<std::string>("scan"));
  13 |     private_nh_.param("out_costmap_topic", out_costmap_topic, static_cast<std::string>("local_costmap"));
  14 | 
  15 |     // common params
  16 |     private_nh_.param("update_rate", update_rate_, 0.01);
  17 |     private_nh_.param("thread_num", thread_num_, 4);
  18 | 
  19 |     // preprocess params
  20 |     private_nh_.param("is_crop_robot", is_crop_robot_, true);
  21 |     private_nh_.param("is_remove_outlier", is_remove_outlier_, false);
  22 |     private_nh_.param("sor_mean_k", sor_mean_k_, 10);
  23 |     private_nh_.param("sor_stddev_mul_thresh", sor_stddev_mul_thresh_, 1.0);
  24 |     private_nh_.param("is_downsample", is_downsample_, false);
  25 |     private_nh_.param("downsample_resolution", downsample_resolution_, 0.1);
  26 |     private_nh_.param("is_pass_through", is_pass_through_, false);
  27 |     private_nh_.param("pass_through_min_from_robot", pass_through_min_from_robot_, 0.0);
  28 |     private_nh_.param("pass_through_max_from_robot", pass_through_max_from_robot_, 2.0);
  29 | 
  30 |     // rigid body shape params
  31 |     private_nh_.param("baselink2front", rigid_body_shape_.baselink2front, 0.47);
  32 |     private_nh_.param("baselink2rear", rigid_body_shape_.baselink2rear, 0.14);
  33 |     private_nh_.param("baselink2right", rigid_body_shape_.baselink2right, 0.15);
  34 |     private_nh_.param("baselink2left", rigid_body_shape_.baselink2left, 0.15);
  35 | 
  36 |     // costmap params
  37 |     double map_x_length = 10.0;
  38 |     double map_y_length = 10.0;
  39 |     double map_center_offset_x = 3.0;
  40 |     double map_center_offset_y = 0.0;
  41 |     double map_resolution = 0.1;
  42 |     private_nh_.param("map_x_length", map_x_length, 10.0);
  43 |     private_nh_.param("map_y_length", map_y_length, 10.0);
  44 |     private_nh_.param("map_center_offset_x", map_center_offset_x, 3.0);
  45 |     private_nh_.param("map_center_offset_y", map_center_offset_y, 0.0);
  46 |     private_nh_.param("map_resolution", map_resolution, 0.1);
  47 |     private_nh_.param("max_val", max_val_, 100.0);
  48 | 
  49 |     // initialize inner variables
  50 |     cost_map_.setFrameId(robot_frame_id_);
  51 |     cost_map_.setGeometry(grid_map::Length(map_x_length, map_y_length), map_resolution, grid_map::Position(map_center_offset_x, map_center_offset_y));
  52 |     raw_pc2_ptr_ = boost::make_shared<sensor_msgs::PointCloud2>();
  53 |     raw_pcl_ptr_ = boost::make_shared<pcl::PointCloud<PointType>>();
  54 |     preprocessed_pcl_ptr_ = boost::make_shared<pcl::PointCloud<PointType>>();
  55 |     trans_preprocessed_pcl_ptr_ = boost::make_shared<pcl::PointCloud<PointType>>();
  56 | 
  57 |     // crop box filter
  58 |     const double min_high = 0.0;
  59 |     const double max_high = 10.0;
  60 |     const double min_x = -rigid_body_shape_.baselink2rear;
  61 |     const double max_x = rigid_body_shape_.baselink2front;
  62 |     const double min_y = -rigid_body_shape_.baselink2right;
  63 |     const double max_y = rigid_body_shape_.baselink2left;
  64 |     crop_box_min_ = Eigen::Vector4f(min_x, min_y, min_high, 1.0);
  65 |     crop_box_max_ = Eigen::Vector4f(max_x, max_y, max_high, 1.0);
  66 | 
  67 |     // set publishers and subscribers
  68 |     timer_costmap_ = nh_.createTimer(ros::Duration(update_rate_), &LocalCostmapGenerator::timer_callback, this);
  69 |     pub_cost_map_ = nh_.advertise<grid_map_msgs::GridMap>(out_costmap_topic, 1, true);
  70 |     pub_rigid_body_shape_ = nh_.advertise<visualization_msgs::MarkerArray>("local_costmap/rigid_body_shape", 1, true);
  71 |     sub_scan_ = nh_.subscribe(in_scan_topic, 1, &LocalCostmapGenerator::scan_callback, this);
  72 | 
  73 |     // publish rigid body shape
  74 |     publish_rigid_body_shape(rigid_body_shape_);
  75 | 
  76 |     // debug
  77 |     pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("preprocessed_cloud", 1, true);
  78 |     // pub_calc_time_ = nh_.advertise<std_msgs::Float32>("local_costmap/calc_time", 1, true);
  79 | }
  80 | 
  81 | void LocalCostmapGenerator::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  82 |     // convert Laser scan to point cloud
  83 |     projector_.projectLaser(*scan, *raw_pc2_ptr_);
  84 | 
  85 |     // convert pointcloud2 to pcl
  86 |     pcl::fromROSMsg(*raw_pc2_ptr_, *raw_pcl_ptr_);
  87 | 
  88 |     is_laser_scan_received_ = true;
  89 | }
  90 | 
  91 | void LocalCostmapGenerator::timer_callback([[maybe_unused]] const ros::TimerEvent& te) {
  92 |     /* Status check */
  93 |     if (!is_laser_scan_received_) {
  94 |         ROS_WARN_THROTTLE(5.0, "[LocalCostmapGenerator] Waiting for laser scan data...");
  95 |         return;
  96 |     }
  97 | 
  98 |     // ======== time measurement ========
  99 |     // const auto start_time = std::chrono::system_clock::now();
 100 | 
 101 |     // preprocess point cloud
 102 |     preprocess_pointcloud(raw_pcl_ptr_, preprocessed_pcl_ptr_);
 103 | 
 104 |     // transform coordinate from sensor frame to robot frame
 105 |     try {
 106 |         transform_stamped_ = tf_buffer_.lookupTransform(robot_frame_id_, sensor_frame_id_, ros::Time(0));
 107 |     } catch (tf2::TransformException& ex) {
 108 |         ROS_WARN_THROTTLE(3.0, "[LocalCostmapGenerator] %s", ex.what());
 109 |         return;
 110 |     }
 111 |     const Eigen::Isometry3d transform_matrix = tf2::transformToEigen(transform_stamped_.transform);
 112 |     pcl::transformPointCloud(*preprocessed_pcl_ptr_, *trans_preprocessed_pcl_ptr_, transform_matrix.matrix().cast<float>());
 113 | 
 114 |     // remove points within the robot
 115 |     if (is_crop_robot_) {
 116 |         crop_points_within_robot(trans_preprocessed_pcl_ptr_);
 117 |     }
 118 | 
 119 |     // convert to grid map
 120 |     const std::vector<grid_map::Index> occupied_indices = pointcloud_to_costmap(trans_preprocessed_pcl_ptr_, &cost_map_);
 121 | 
 122 |     // inflate cost map with rigid body for collision avoidance
 123 |     inflate_rigid_body(collision_layer_name_, occupied_indices, &cost_map_);
 124 | 
 125 |     // ======== time measurement ========
 126 |     // const auto end_time = std::chrono::system_clock::now();
 127 |     // const auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000000.0;
 128 |     // std_msgs::Float32 calc_time_msg;
 129 |     // calc_time_msg.data = elapsed_time;
 130 |     // pub_calc_time_.publish(calc_time_msg);
 131 | 
 132 |     // publish cost map
 133 |     grid_map_msgs::GridMap message;
 134 |     grid_map::GridMapRosConverter::toMessage(cost_map_, message);
 135 |     message.info.header.stamp = ros::Time::now();
 136 |     pub_cost_map_.publish(message);
 137 | 
 138 |     // debug
 139 |     // Publish point preprocessed could
 140 |     sensor_msgs::PointCloud2 preprocessed_pc2;
 141 |     pcl::toROSMsg(*trans_preprocessed_pcl_ptr_, preprocessed_pc2);
 142 |     preprocessed_pc2.header.frame_id = robot_frame_id_;
 143 |     preprocessed_pc2.header.stamp = ros::Time::now();
 144 |     pub_cloud_.publish(preprocessed_pc2);
 145 | }
 146 | 
 147 | /**
 148 |  * @brief preprocess_pointcloud
 149 |  * @param[in] raw_pcl_ptr_ raw point cloud in sensor frame
 150 |  * @param[out] preprocessed_pcl_ptr_ preprocessed point cloud in sensor frame
 151 |  */
 152 | void LocalCostmapGenerator::preprocess_pointcloud(const pcl::PointCloud<PointType>::ConstPtr& raw_pcl_ptr_,
 153 |                                                   pcl::PointCloud<PointType>::Ptr& preprocessed_pcl_ptr_) {
 154 |     // NAN remove
 155 |     boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
 156 |     pcl::removeNaNFromPointCloud(*raw_pcl_ptr_, *preprocessed_pcl_ptr_, *indices);
 157 | 
 158 |     // outlier remove
 159 |     if (is_remove_outlier_) {
 160 |         sor_.setInputCloud(preprocessed_pcl_ptr_);
 161 |         sor_.setMeanK(sor_mean_k_);
 162 |         sor_.setStddevMulThresh(sor_stddev_mul_thresh_);
 163 |         sor_.filter(*preprocessed_pcl_ptr_);
 164 |     }
 165 | 
 166 |     // downsample
 167 |     if (is_downsample_) {
 168 |         voxel_grid_filter_.setInputCloud(preprocessed_pcl_ptr_);
 169 |         voxel_grid_filter_.setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
 170 |         voxel_grid_filter_.filter(*preprocessed_pcl_ptr_);
 171 |     }
 172 | 
 173 |     // pass through filter to remove too near or too far points
 174 |     if (is_pass_through_) {
 175 |         pass_through_filter_.setInputCloud(preprocessed_pcl_ptr_);
 176 |         pass_through_filter_.setFilterFieldName("z");
 177 |         pass_through_filter_.setFilterLimits(pass_through_min_from_robot_, pass_through_max_from_robot_);
 178 |         pass_through_filter_.filter(*preprocessed_pcl_ptr_);
 179 |     }
 180 | }
 181 | 
 182 | void LocalCostmapGenerator::crop_points_within_robot(pcl::PointCloud<PointType>::Ptr& pcl_ptr_) {
 183 |     crop_box_filter_.setInputCloud(pcl_ptr_);
 184 |     crop_box_filter_.setNegative(true);
 185 |     crop_box_filter_.setMin(crop_box_min_);
 186 |     crop_box_filter_.setMax(crop_box_max_);
 187 |     // Eigen::Vector4f min_point, max_point;
 188 |     // double min_high = 0.0;
 189 |     // double max_high = 2.0;
 190 |     // double min_x = -rigid_body_shape_.baselink2rear;
 191 |     // double max_x = rigid_body_shape_.baselink2front;
 192 |     // double min_y = -rigid_body_shape_.baselink2right;
 193 |     // double max_y = rigid_body_shape_.baselink2left;
 194 |     // min_point << min_x, min_y, min_high, 1.0;
 195 |     // max_point << max_x, max_y, max_high, 1.0;
 196 | 
 197 |     crop_box_filter_.filter(*pcl_ptr_);
 198 | }
 199 | 
 200 | std::vector<grid_map::Index> LocalCostmapGenerator::pointcloud_to_costmap(const pcl::PointCloud<PointType>::ConstPtr& preprocessed_pcl_ptr,
 201 |                                                                           grid_map::GridMap* cost_map) const
 202 | 
 203 | {
 204 |     grid_map::Matrix& cost_map_data = cost_map->get(collision_layer_name_);
 205 |     // costmap clear
 206 |     cost_map_data.setZero();
 207 |     std::vector<grid_map::Index> occupied_indices(preprocessed_pcl_ptr->points.size());
 208 | 
 209 | #pragma omp parallel for num_threads(thread_num_)
 210 |     for (unsigned int i = 0; i < preprocessed_pcl_ptr->points.size(); ++i) {
 211 |         const auto& point = preprocessed_pcl_ptr->points[i];
 212 |         if (cost_map->isInside(grid_map::Position(point.x, point.y))) {
 213 |             grid_map::Index index;
 214 |             cost_map->getIndex(grid_map::Position(point.x, point.y), index);
 215 |             cost_map_data(index.x(), index.y()) = max_val_;
 216 |             occupied_indices[i] = index;
 217 |         } else {
 218 |             occupied_indices[i] = grid_map::Index(-1, -1);
 219 |         }
 220 |     }
 221 | 
 222 |     // remove index (-1, -1) which means outside of costmap
 223 |     occupied_indices.erase(std::remove_if(occupied_indices.begin(), occupied_indices.end(),
 224 |                                           [](const grid_map::Index& index) { return index.x() == -1 && index.y() == -1; }),
 225 |                            occupied_indices.end());
 226 | 
 227 |     return occupied_indices;
 228 | }
 229 | 
 230 | // Inflate costmap for rigid body
 231 | void LocalCostmapGenerator::inflate_rigid_body(const std::string& layer_name,
 232 |                                                const std::vector<grid_map::Index>& occupied_indices,
 233 |                                                grid_map::GridMap* cost_map) const {
 234 |     grid_map::Matrix& cost_map_data = cost_map->get(layer_name);
 235 | 
 236 |     const int front_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2front / cost_map->getResolution()));
 237 |     const int rear_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2rear / cost_map->getResolution()));
 238 |     const int right_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2right / cost_map->getResolution()));
 239 |     const int left_offset = static_cast<int>(std::ceil(rigid_body_shape_.baselink2left / cost_map->getResolution()));
 240 | 
 241 |     const grid_map::Size map_size = cost_map->getSize();
 242 | 
 243 | // inflate costmap
 244 | #pragma omp parallel for num_threads(thread_num_)
 245 |     for (size_t i = 0; i < occupied_indices.size(); ++i) {
 246 |         const auto index = occupied_indices[i];
 247 |         const int start_y = std::max(index.y() - right_offset, 0);
 248 |         const int last_y = std::min(index.y() + left_offset, map_size(0));
 249 |         const int start_x = std::max(index.x() - rear_offset, 0);
 250 |         const int last_x = std::min(index.x() + front_offset, map_size(1));
 251 | 
 252 |         for (int x = start_x; x < last_x; ++x) {
 253 |             for (int y = start_y; y < last_y; ++y) {
 254 |                 cost_map_data(x, y) = max_val_;
 255 |             }
 256 |         }
 257 |     }
 258 | }
 259 | 
 260 | void LocalCostmapGenerator::publish_rigid_body_shape(const RigidBodyShape& rigid_body_shape) const {
 261 |     visualization_msgs::MarkerArray marker_array;
 262 |     visualization_msgs::Marker marker;
 263 | 
 264 |     marker.header.frame_id = robot_frame_id_;
 265 |     marker.header.stamp = ros::Time::now();
 266 |     marker.ns = "rigid_body_shape";
 267 |     marker.id = 0;
 268 |     marker.type = visualization_msgs::Marker::CUBE;
 269 |     marker.action = visualization_msgs::Marker::ADD;
 270 | 
 271 |     marker.pose.position.x = (rigid_body_shape.baselink2front - rigid_body_shape.baselink2rear) / 2.0;
 272 |     marker.pose.position.y = 0.0;
 273 |     marker.pose.position.z = 0.0;
 274 |     marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
 275 | 
 276 |     marker.scale.x = rigid_body_shape.baselink2front + rigid_body_shape.baselink2rear;
 277 |     marker.scale.y = rigid_body_shape.baselink2right + rigid_body_shape.baselink2left;
 278 |     marker.scale.z = 0.1;
 279 | 
 280 |     marker.color.r = 0.0;
 281 |     marker.color.g = 1.0;
 282 |     marker.color.b = 0.0;
 283 |     marker.color.a = 0.5;
 284 | 
 285 |     marker_array.markers.push_back(marker);
 286 | 
 287 |     pub_rigid_body_shape_.publish(marker_array);
 288 | }
 289 | 
 290 | }  // namespace local_costmap_generator

```

`src\local_costmap_generator\src\local_costmap_generator_node.cpp`:

```cpp
   1 | #include "local_costmap_generator/local_costmap_generator.hpp"
   2 | 
   3 | int main(int argc, char** argv) {
   4 |     ros::init(argc, argv, "local_costmap_generator");
   5 |     local_costmap_generator::LocalCostmapGenerator local_costmap_generator;
   6 |     ros::spin();
   7 |     return 0;
   8 | };

```

`src\mppi_controller\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.13)
   2 | project(mppi_controller)
   3 | 
   4 | option(BUILD_WITH_GPU "Build with GPU support" OFF)
   5 | 
   6 | add_compile_options(-std=c++17)
   7 | add_compile_options(-Wall -Wextra)
   8 | add_compile_options(-O3 -fopenmp)
   9 | # set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
  10 | if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  11 |   set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  12 |   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  13 | endif()
  14 | 
  15 | ## Find catkin macros and libraries
  16 | ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  17 | ## is used, also find other catkin packages
  18 | find_package(catkin REQUIRED COMPONENTS
  19 |   ackermann_msgs
  20 |   geometry_msgs
  21 |   nav_msgs
  22 |   roscpp
  23 |   std_msgs
  24 |   tf2_geometry_msgs
  25 |   tf2_ros
  26 |   grid_map_core
  27 |   grid_map_ros
  28 |   grid_map_filters
  29 |   grid_map_loader
  30 |   grid_map_msgs
  31 |   grid_map_rviz_plugin
  32 |   grid_map_visualization
  33 |   mppi_metrics_msgs
  34 | )
  35 | 
  36 | find_package(Eigen3 REQUIRED)
  37 | 
  38 | # For OpenMP
  39 | # set(OpenMP_HOME "/usr/lib/llvm-10")
  40 | # set(OpenMP_omp_LIBRARY "${OpenMP_HOME}/lib/")
  41 | # set(OpenMP_C_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
  42 | # set(OpenMP_CXX_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
  43 | # set(OpenMP_C_LIB_NAMES "omp")
  44 | # set(OpenMP_CXX_LIB_NAMES "omp")
  45 | find_package(OpenMP REQUIRED)
  46 | if(OpenMP_FOUND)
  47 |     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  48 |     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  49 | endif()
  50 | 
  51 | 
  52 | if(BUILD_WITH_GPU)
  53 |   find_package(CUDA REQUIRED)
  54 |   include_directories(${CUDA_INCLUDE_DIRS})
  55 |   link_directories(${CUDA_LIBRARY_DIRS})
  56 | endif()
  57 | 
  58 | catkin_package(
  59 |  INCLUDE_DIRS include
  60 | #  LIBRARIES mppi
  61 | #  CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros
  62 | #  DEPENDS system_lib
  63 | )
  64 | 
  65 | ###########
  66 | ## Build ##
  67 | ###########
  68 | 
  69 | #### Library (CPU) ####
  70 | add_library(mppi_solver SHARED
  71 |   src/prior_samples_with_costs.cpp
  72 |   src/mpc_base.cpp
  73 |   src/forward_mppi.cpp
  74 |   src/reverse_mppi.cpp
  75 |   src/stein_variational_mpc.cpp
  76 |   src/stein_variational_guided_mppi.cpp
  77 | )
  78 | target_link_libraries(mppi_solver
  79 |   ${catkin_LIBRARIES}
  80 | )
  81 | if (OPENMP_FOUND)
  82 |     if (TARGET OpenMP::OpenMP_CXX)
  83 |         target_link_libraries(mppi_solver OpenMP::OpenMP_CXX)
  84 |     endif ()
  85 | endif ()
  86 | 
  87 | target_include_directories(mppi_solver PUBLIC
  88 |   include
  89 |   ${catkin_INCLUDE_DIRS}
  90 |   ${EIGEN3_INCLUDE_DIR}
  91 | )
  92 | 
  93 | #### App ####
  94 | add_executable(${PROJECT_NAME}_node 
  95 |   src/mppi_controller_node.cpp
  96 |   src/mppi_controller_ros.cpp
  97 |   )
  98 | target_include_directories(${PROJECT_NAME}_node PUBLIC
  99 |   include
 100 |   ${catkin_INCLUDE_DIRS}
 101 |   ${EIGEN3_INCLUDE_DIR}
 102 | )
 103 | target_link_libraries(${PROJECT_NAME}_node PRIVATE
 104 |   ${catkin_LIBRARIES}
 105 |   mppi_solver
 106 | )
 107 | 
 108 | 
 109 | ### CUDA ###
 110 | if(BUILD_WITH_GPU)
 111 |   set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
 112 |   add_definitions(-DUSE_CUDA)
 113 | 
 114 |   cuda_add_library(mppi_solver_gpu SHARED
 115 |     src/mppi_gpu.cu
 116 |   )
 117 | 
 118 |   target_include_directories(mppi_solver_gpu PRIVATE
 119 |     include
 120 |     ${catkin_INCLUDE_DIRS}
 121 |     ${EIGEN3_INCLUDE_DIR}
 122 |   )
 123 | 
 124 |   target_link_libraries(mppi_solver_gpu 
 125 |     ${catkin_LIBRARIES}
 126 |     ${CUDA_LIBRARIES}
 127 |     ${CUDA_curand_LIBRARY}
 128 |   )
 129 |   cuda_add_cublas_to_target(mppi_solver_gpu)
 130 | 
 131 |   # add mppi_solver_gpu to mppi_solver
 132 |   target_link_libraries(mppi_solver
 133 |     mppi_solver_gpu
 134 |   )
 135 |   add_dependencies(mppi_solver mppi_solver_gpu)
 136 | endif()
 137 | 
 138 | 
 139 | install(
 140 |   TARGETS
 141 |     ${PROJECT_NAME}_node
 142 |   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 143 |   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 144 |   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 145 | )
 146 | 
 147 | install(
 148 |   DIRECTORY
 149 |     launch
 150 |     config
 151 |   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 152 | )

```

`src\mppi_controller\README.md`:

```md
   1 | # MPPI Controller

```

`src\mppi_controller\config\mppi_controller.yaml`:

```yaml
   1 | # system
   2 | control_cmd_topic: drive
   3 | in_reference_sdf_topic: reference_sdf
   4 | in_odom_topic: odom
   5 | is_activate_ad_topic: is_active_ad
   6 | robot_frame_id: ego_racecar/base_link
   7 | map_frame_id: map
   8 | costmap_id: f1_costmap_2d/f1_costmap/costmap # used only not localize_less_mode
   9 | local_costmap_id: local_costmap # used only localize_less_mode
  10 | backward_point_topic: backward_point
  11 | control_sampling_time: 0.025
  12 | use_local_costmap: true # if true, use local costmap of grid map. Otherwise, use global costmap of costmap 2d.
  13 | # Because the local costmap is more accurate than the global costmap (not effected pose estimation accuracy),
  14 | # we recommend to use the local costmap.
  15 | # NOTE: If you set localize_less_mode to true, force the use_local_costmap to true.
  16 | is_visualize_mppi: true
  17 | constant_speed_mode: false
  18 | # if true, the speed is fixed to reference_speed. Otherwise, the reference speed is determined by the waypoints.
  19 | # NOTE: If you set localize_less_mode to true, force the constant_speed_mode to true.
  20 | reference_speed: 1.5
  21 | collision_rate_threshold: 1.1 # [0, 1] If the collision rate is larger than this value, robot speed cmd is set to 0.
  22 | # stuck detection params
  23 | # This is currently only used in localize_less_mode.
  24 | speed_deque_size: 10
  25 | stuck_speed_threshold: 0.3 # [m/s]
  26 | steer_1st_delay: 0.1 # [s] steer delay of 1st order system
  27 | mpc_mode: "svg_mppi" # "forward_mppi" or "reverse_mppi" or "sv_mpc" or "svg_mppi"
  28 | 
  29 | common:
  30 |   thread_num: 12
  31 |   prediction_step_size: 15
  32 |   prediction_interval: 0.05
  33 |   steer_delay: 0.025 # [s] dead time. If set enough large value, the prediction is similar to DWA planner.
  34 |   max_steer_angle: 0.45
  35 |   min_steer_angle: -0.45
  36 |   speed_prediction_mode: reference # linear or constant or reference
  37 |   max_accel: 5.0
  38 |   min_accel: -3.0
  39 |   lr: 0.135
  40 |   lf: 0.189
  41 |   collision_weight: 1.0
  42 |   # If localize_less_mode, the following parameters (q_angle and q_dist) are ignored.
  43 |   q_dist: 1.0
  44 |   q_angle: 0.01
  45 |   q_terminal_dist: 1.0
  46 |   q_terminal_angle: 0.01
  47 | 
  48 | forward_mppi:
  49 |   sample_batch_num: 10000
  50 |   lambda: 3.0 # temperature parameter [0, inf) of free energy, which is a balancing term between control cost and state cost.
  51 |   alpha: 0.1 # weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal control sequence.
  52 |   non_biased_sampling_rate: 0.1 # [0, 1]. add random noise to candidate control sequence with this rate.
  53 |   # steer_cov: 0.05
  54 |   steer_cov: 0.1
  55 |   # accel_cov: 0.1
  56 | 
  57 |   # Parameters for estimating gradient of Reverse KLD
  58 |   num_itr_for_grad_estimation: 0 # If 0, vanilla MPPI is used without gradient estimation.
  59 |   step_size_for_grad_estimation: 0.001
  60 |   sample_num_for_grad_estimation: 5
  61 |   steer_cov_for_grad_estimation: 0.001
  62 | 
  63 | reverse_mppi:
  64 |   sample_batch_num: 200
  65 |   negative_ratio: 1.0
  66 |   is_sample_rejection: true
  67 |   sample_inflation_ratio: 2.0
  68 |   iteration_num: 50
  69 |   step_size: 0.05
  70 |   warm_start_ratio: 0.5
  71 |   lambda: 3.0 # temperature parameter [0, inf) of free energy, which is a balancing term between control cost and state cost.
  72 |   alpha: 0.1 # weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal control sequence.
  73 |   non_biased_sampling_rate: 0.1 # [0, 1]. add random noise to candidate control sequence with this rate.
  74 |   steer_cov: 0.1
  75 |   # accel_cov: 0.1
  76 | 
  77 | stein_variational_mpc:
  78 |   sample_batch_num: 500
  79 |   lambda: 3.0 # temperature parameter [0, inf) of free energy, which is a balancing term between control cost and state cost.
  80 |   alpha: 0.1
  81 |   non_biased_sampling_rate: 0.1 # [0, 1]. add random noise to candidate control sequence with this rate.
  82 |   # steer_cov: 0.05
  83 |   steer_cov: 0.1 # initial covariance
  84 |   # accel_cov: 0.1
  85 |   num_svgd_iteration: 3
  86 |   sample_num_for_grad_estimation: 10
  87 |   steer_cov_for_grad_estimation: 0.01
  88 |   svgd_step_size: 0.5
  89 |   is_max_posterior_estimation: false
  90 | 
  91 | svg_mppi:
  92 |   sample_batch_num: 8000
  93 |   lambda: 3.0 # temperature parameter [0, inf) of free energy, which is a balancing term between control cost and state cost.
  94 |   non_biased_sampling_rate: 0.1 # [0, 1]. add random noise to candidate control sequence with this rate.
  95 |   alpha: 0.1 # weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal control sequence.
  96 |   # steer_cov: 0.05
  97 |   steer_cov: 0.01 # initial covariance or constant covariance if is_covariance_adaptation is false
  98 |   # accel_cov: 0.1
  99 |   guide_sample_num: 1
 100 |   grad_lambda: 3.0
 101 |   sample_num_for_grad_estimation: 100
 102 |   steer_cov_for_grad_estimation: 0.01
 103 |   svgd_step_size: 0.005
 104 |   num_svgd_iteration: 10
 105 |   is_use_nominal_solution: true
 106 |   is_covariance_adaptation: true
 107 |   gaussian_fitting_lambda: 0.1
 108 |   min_steer_cov: 0.001
 109 |   max_steer_cov: 0.1

```

`src\mppi_controller\include\mppi_controller\StopWatch.hpp`:

```hpp
   1 | #ifndef STOP_WATCH_H
   2 | #define STOP_WATCH_H
   3 | 
   4 | #include <chrono>
   5 | 
   6 | // Return lap time [msec] during the previous lap()
   7 | struct StopWatch {
   8 |     StopWatch() { pre_ = std::chrono::high_resolution_clock::now(); }
   9 | 
  10 |     double lap() {
  11 |         auto tmp = std::chrono::high_resolution_clock::now();
  12 |         auto dur = tmp - pre_;
  13 |         pre_ = tmp;
  14 |         return std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count() / 1000000.0;
  15 |     }
  16 |     std::chrono::high_resolution_clock::time_point pre_;
  17 | };
  18 | 
  19 | #endif

```

`src\mppi_controller\include\mppi_controller\common.hpp`:

```hpp
   1 | #pragma once
   2 | 
   3 | #include <string>
   4 | 
   5 | namespace mppi {
   6 | namespace STATE_SPACE {
   7 |     static constexpr int x = 0;
   8 |     static constexpr int y = 1;
   9 |     static constexpr int yaw = 2;
  10 |     static constexpr int vel = 3;
  11 |     static constexpr int steer = 4;
  12 |     static constexpr int dim = 5;
  13 | };  // namespace STATE_SPACE
  14 | 
  15 | namespace CONTROL_SPACE {
  16 |     static constexpr int steer = 0;
  17 |     // static constexpr int accel = 1;
  18 |     // static constexpr int dim = 2;
  19 | 
  20 |     static constexpr int dim = 1;
  21 | };  // namespace CONTROL_SPACE
  22 | 
  23 | struct Params {
  24 |     struct Common {
  25 |         int thread_num;
  26 |         int prediction_step_size;
  27 |         double prediction_interval;
  28 |         double steer_delay;
  29 |         double steer_1st_delay;
  30 |         bool is_localize_less_mode;
  31 |         double reference_speed;
  32 |         double max_steer_angle;
  33 |         double min_steer_angle;
  34 |         double max_accel;
  35 |         double min_accel;
  36 |         std::string speed_prediction_mode;
  37 |         double lr;
  38 |         double lf;
  39 |         double q_dist;
  40 |         double q_angle;
  41 |         double q_speed;
  42 |         double collision_weight;
  43 |         double q_terminal_dist;
  44 |         double q_terminal_angle;
  45 |         double q_terminal_speed;
  46 |     };
  47 |     Common common;
  48 | 
  49 |     struct ForwardMPPI {
  50 |         int sample_batch_num;
  51 |         double lambda;
  52 |         double alpha;
  53 |         double non_biased_sampling_rate;
  54 |         double steer_cov;
  55 |         // double accel_cov;
  56 |         int sample_num_for_grad_estimation;
  57 |         double steer_cov_for_grad_estimation;
  58 |         int num_itr_for_grad_estimation;
  59 |         double step_size_for_grad_estimation;
  60 |     };
  61 |     ForwardMPPI forward_mppi;
  62 | 
  63 |     struct ReverseMPPI {
  64 |         int sample_batch_num;
  65 |         double negative_ratio;
  66 |         bool is_sample_rejection;
  67 |         double sample_inflation_ratio;
  68 |         double warm_start_ratio;
  69 |         int iteration_num;
  70 |         double step_size;
  71 |         double lambda;
  72 |         double alpha;
  73 |         double non_biased_sampling_rate;
  74 |         double steer_cov;
  75 |         // double accel_cov;
  76 |     };
  77 |     ReverseMPPI reverse_mppi;
  78 | 
  79 |     struct SteinVariationalMPC {
  80 |         int sample_batch_num;
  81 |         double lambda;
  82 |         double alpha;
  83 |         double non_biased_sampling_rate;
  84 |         double steer_cov;
  85 |         // double accel_cov;
  86 |         int num_svgd_iteration;
  87 |         int sample_num_for_grad_estimation;
  88 |         double steer_cov_for_grad_estimation;
  89 |         double svgd_step_size;
  90 |         bool is_max_posterior_estimation;
  91 |     };
  92 |     SteinVariationalMPC stein_variational_mpc;
  93 | 
  94 |     struct SVGuidedMPPI {
  95 |         int sample_batch_num;
  96 |         double lambda;
  97 |         double alpha;
  98 |         double non_biased_sampling_rate;
  99 |         double steer_cov;
 100 |         // double accel_cov;
 101 |         int guide_sample_num;
 102 |         double grad_lambda;
 103 |         int sample_num_for_grad_estimation;
 104 |         double steer_cov_for_grad_estimation;
 105 |         double svgd_step_size;
 106 |         int num_svgd_iteration;
 107 |         bool is_use_nominal_solution;
 108 |         bool is_covariance_adaptation;
 109 |         double gaussian_fitting_lambda;
 110 |         double min_steer_cov;
 111 |         double max_steer_cov;
 112 |     };
 113 |     SVGuidedMPPI svg_mppi;
 114 | };
 115 | 
 116 | namespace cpu {
 117 |     using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
 118 |     using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;
 119 |     using StateSeq = Eigen::MatrixXd;
 120 |     using ControlSeq = Eigen::MatrixXd;
 121 |     using StateSeqBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
 122 |     using ControlSeqBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
 123 |     using ControlSeqCovMatrices = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
 124 |     using XYCovMatrices = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
 125 | }  // namespace cpu
 126 | 
 127 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\forward_mppi.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <algorithm>
   5 | #include <iostream>
   6 | #include <limits>
   7 | #include <mutex>
   8 | #include <string>
   9 | #include <vector>
  10 | 
  11 | #include <Eigen/Dense>
  12 | #include <array>
  13 | #include <grid_map_core/GridMap.hpp>
  14 | #include <memory>
  15 | #include <utility>
  16 | 
  17 | #include "mppi_controller/common.hpp"
  18 | #include "mppi_controller/mpc_base.hpp"
  19 | #include "mppi_controller/mpc_template.hpp"
  20 | #include "mppi_controller/prior_samples_with_costs.hpp"
  21 | 
  22 | namespace mppi {
  23 | namespace cpu {
  24 | 
  25 |     /**
  26 |      * @brief Model Predictive Path Integral Control, G. Williams, https://arxiv.org/abs/1707.02342
  27 |      */
  28 |     class ForwardMPPI : public MPCTemplate {
  29 |     public:
  30 |         ForwardMPPI(const Params::Common& common_params, const Params::ForwardMPPI& forward_mppi_params);
  31 |         ~ForwardMPPI(){};
  32 | 
  33 |         /**
  34 |          * @brief solve mppi problem and return optimal control sequence
  35 |          * @param initial_state initial state
  36 |          * @return optimal control sequence and collision rate
  37 |          */
  38 |         std::pair<ControlSeq, double> solve(const State& initial_state) override;
  39 |         /**
  40 |          * @brief set obstacle map and reference map
  41 |          */
  42 |         void set_obstacle_map(const grid_map::GridMap& obstacle_map) override;
  43 | 
  44 |         void set_reference_map(const grid_map::GridMap& reference_map) override;
  45 | 
  46 |         /**
  47 |          * @brief get state sequence candidates and their weights, top num_samples
  48 |          * @return std::pair<std::vector<StateSeq>, std::vector<double>> state sequence candidates and their weights
  49 |          */
  50 |         std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples) const override;
  51 | 
  52 |         std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state,
  53 |                                                                         const ControlSeq& control_input_seq) const override;
  54 | 
  55 |         ControlSeqCovMatrices get_cov_matrices() const override;
  56 | 
  57 |         ControlSeq get_control_seq() const override;
  58 | 
  59 |         std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const override;
  60 | 
  61 |     private:
  62 |         const size_t prediction_step_size_;  //!< @brief prediction step size
  63 |         const int thread_num_;               //!< @brief number of thread for parallel computation
  64 | 
  65 |         // MPPI parameters
  66 |         const double lambda_;  //!< @brief temperature parameter [0, inf) of free energy.
  67 |         // temperature parameter is a balancing term between control cost and state cost.
  68 |         // If lambda_ is small, weighted state cost is dominant, thus control sequence is more aggressive.
  69 |         // If lambda_ is large, weighted control cost is dominant, thus control sequence is more smooth.
  70 |         const double alpha_;  //!< @brief weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal
  71 |                               //!< control sequence.
  72 |         const double non_biased_sampling_rate_;  //!< @brief non-previous-control-input-biased sampling rate [0, 1], Add random noise to control
  73 |                                                  //!< sequence with this rate.
  74 |         const double steer_cov_;                 //!< @brief covariance of steering angle noise
  75 |         // double accel_cov_;                      //!< @brief covariance of acceleration noise
  76 | 
  77 |         // Parameters for estimating gradient of R-KLD
  78 |         const int num_itr_for_grad_estimation_;
  79 |         const double step_size_for_grad_estimation_;
  80 |         const size_t sample_num_for_grad_estimation_;
  81 |         const double steer_cov_for_grad_estimation_;
  82 | 
  83 |         // Internal variables
  84 |         ControlSeq prev_control_seq_;
  85 |         std::vector<double> weights_ = {};  // for visualization
  86 | 
  87 |         // Libraries
  88 |         std::unique_ptr<MPCBase> mpc_base_ptr_;
  89 |         std::unique_ptr<PriorSamplesWithCosts> prior_samples_ptr_;
  90 |         std::vector<std::unique_ptr<PriorSamplesWithCosts>> grad_sampler_ptrs_;
  91 | 
  92 |     private:
  93 |         std::vector<double> calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const;
  94 | 
  95 |         std::vector<int> get_top_indices(const std::vector<double>& values, const int& num) const;
  96 | 
  97 |         ControlSeq grad_reverse_kld(const ControlSeq& mean_seq,
  98 |                                     const ControlSeq& noised_seq,
  99 |                                     const ControlSeqCovMatrices& inv_covs,
 100 |                                     const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
 101 |                                     PriorSamplesWithCosts* sampler) const;
 102 | 
 103 |         ControlSeqBatch grad_reverse_kld_batch(const PriorSamplesWithCosts& samples,
 104 |                                                const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const;
 105 | 
 106 |         ControlSeqBatch transport_samples(const PriorSamplesWithCosts& samples_with_cost,
 107 |                                           const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
 108 |                                           const int& num_itr,
 109 |                                           const double& step_size);
 110 | 
 111 |         ControlSeqBatch transport_samples(const PriorSamplesWithCosts& samples,
 112 |                                           const ControlSeqBatch& grad_kld_batch,
 113 |                                           const int& num_itr,
 114 |                                           const double& step_size) const;
 115 | 
 116 |         std::pair<ControlSeq, ControlSeqCovMatrices> estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const;
 117 |     };
 118 | 
 119 | }  // namespace cpu
 120 | 
 121 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\mpc_base.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | 
   5 | #include <algorithm>
   6 | #include <iostream>
   7 | #include <limits>
   8 | #include <mutex>
   9 | #include <string>
  10 | #include <vector>
  11 | 
  12 | #include <Eigen/Dense>
  13 | #include <array>
  14 | #include <grid_map_core/GridMap.hpp>
  15 | #include <memory>
  16 | #include <utility>
  17 | 
  18 | #include "mppi_controller/common.hpp"
  19 | #include "mppi_controller/prior_samples_with_costs.hpp"
  20 | 
  21 | namespace mppi {
  22 | namespace cpu {
  23 |     class MPCBase {
  24 |     public:
  25 |         MPCBase(const Params::Common& params, const size_t& sample_num);
  26 |         ~MPCBase(){};
  27 |         enum class SpeedPredictionMode {
  28 |             CONSTANT,
  29 |             LINEAR,
  30 |             REFERENCE,
  31 |         };
  32 | 
  33 |         void set_obstacle_map(const grid_map::GridMap& obstacle_map);
  34 | 
  35 |         void set_reference_map(const grid_map::GridMap& reference_map);
  36 | 
  37 |         std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler, const State& init_state);
  38 | 
  39 |         std::tuple<StateSeq, double, double> get_predictive_seq(const State& initial_state, const ControlSeq& control_input_seq) const;
  40 | 
  41 |         std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples,
  42 |                                                                                        const std::vector<double>& weights) const;
  43 | 
  44 |         std::pair<StateSeq, XYCovMatrices> get_proposed_distribution() const;
  45 | 
  46 |     private:
  47 |         std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler,
  48 |                                                                               const State& global_init_state,
  49 |                                                                               const grid_map::GridMap& obstacle_map,
  50 |                                                                               const grid_map::GridMap& reference_map,
  51 |                                                                               StateSeqBatch* global_state_seq_candidates,
  52 |                                                                               StateSeqBatch* local_state_seq_candidates) const;
  53 |         std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler,
  54 |                                                                               const State& local_init_state,
  55 |                                                                               const grid_map::GridMap& obstacle_map,
  56 |                                                                               StateSeqBatch* local_state_seq_candidates) const;
  57 | 
  58 |         StateSeq predict_state_seq(const ControlSeq& control_seq, const State& init_state, const grid_map::GridMap& reference_map) const;
  59 |         void predict_state_seq(const ControlSeq& control_seq,
  60 |                                const State& global_init_state,
  61 |                                const grid_map::GridMap& reference_map,
  62 |                                StateSeq* global_state_seq,
  63 |                                StateSeq* local_state_seq) const;
  64 | 
  65 |         double constant_speed_prediction(const double& current_speed) const;
  66 |         double linear_speed_prediction(const double& current_speed,
  67 |                                        const double& target_speed,
  68 |                                        const double& prediction_interval,
  69 |                                        const double& min_accel,
  70 |                                        const double& max_accel) const;
  71 |         double reference_speed_prediction(const double& pos_x, const double& pos_y, const grid_map::GridMap& reference_map) const;
  72 | 
  73 |         std::pair<double, double> state_cost(const StateSeq& local_state_seq, const grid_map::GridMap& obstacle_map) const;
  74 |         std::pair<double, double> state_cost(const StateSeq& global_state_seq,
  75 |                                              const StateSeq& local_base_state_seq,
  76 |                                              const grid_map::GridMap& obstacle_map,
  77 |                                              const grid_map::GridMap& ref_path_map) const;
  78 | 
  79 |         std::pair<StateSeq, XYCovMatrices> calc_state_distribution(const StateSeqBatch& state_seq_candidates) const;
  80 | 
  81 |     private:
  82 |         // == Constant parameters ==
  83 |         const std::string obstacle_layer_name_ = "collision_layer";
  84 |         const std::string distance_field_layer_name_ = "distance_field";
  85 |         const std::string angle_field_layer_name_ = "angle_field";
  86 |         const std::string speed_field_layer_name_ = "speed_field";
  87 | 
  88 |         const bool is_localize_less_mode_;
  89 |         const int thread_num_;  //!< @brief number of thread for parallel computation
  90 |         const size_t prediction_step_size_;
  91 |         const double prediction_interval_;  //!< @brief prediction interval [s]
  92 |         const double reference_speed_;      //!< @brief robot reference speed [m/s] Only used when speed_prediction_mode_ == CONST
  93 |         const double lr_;
  94 |         const double lf_;
  95 |         SpeedPredictionMode speed_prediction_mode_ = SpeedPredictionMode::LINEAR;
  96 |         const double max_accel_;  //!< @brief maximum acceleration [m/s^2]
  97 |         const double min_accel_;
  98 |         const double steer_delay_;  //!< @brief dead_time [s]
  99 |         const size_t steer_delay_steps_;
 100 |         const double steer_delay_tau_;  //!< @brief time delay tau [s]
 101 |         const double q_dist_;
 102 |         const double q_angle_;
 103 |         // const double q_speed_;
 104 |         const double collision_weight_;
 105 |         const double q_terminal_dist_;
 106 |         const double q_terminal_angle_;
 107 |         // const double q_terminal_speed_;
 108 | 
 109 |         // == Inner-variables ==
 110 |         grid_map::GridMap obstacle_map_;
 111 |         grid_map::GridMap reference_map_;
 112 |         // To reduce memory allocation and visualize candidates paths
 113 |         StateSeqBatch global_state_seq_candidates_ = {};
 114 |         StateSeqBatch local_state_seq_candidates_ = {};
 115 |     };
 116 | 
 117 | }  // namespace cpu
 118 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\mpc_template.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | #pragma once
   3 | 
   4 | #include <Eigen/Dense>
   5 | #include <grid_map_core/GridMap.hpp>
   6 | #include <utility>
   7 | #include "mppi_controller/common.hpp"
   8 | 
   9 | namespace mppi {
  10 | namespace cpu {
  11 |     /*
  12 |      * @brief virtual class for MPC
  13 |      */
  14 |     class MPCTemplate {
  15 |     public:
  16 |         virtual ~MPCTemplate(){};
  17 | 
  18 |         virtual std::pair<ControlSeq, double> solve(const State& initial_state) = 0;
  19 | 
  20 |         virtual void set_obstacle_map(const grid_map::GridMap& obstacle_map) = 0;
  21 | 
  22 |         virtual void set_reference_map(const grid_map::GridMap& reference_map) = 0;
  23 | 
  24 |         virtual ControlSeq get_control_seq() const = 0;
  25 | 
  26 |         virtual std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples) const = 0;
  27 | 
  28 |         virtual std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state,
  29 |                                                                                 const ControlSeq& control_input_seq) const = 0;
  30 | 
  31 |         virtual ControlSeqCovMatrices get_cov_matrices() const = 0;
  32 | 
  33 |         virtual std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const = 0;
  34 | 
  35 |         std::vector<double> softmax(const std::vector<double>& costs, const double& lambda, const int thread_num) const {
  36 |             const double min_cost = *std::min_element(costs.begin(), costs.end());
  37 |             double normalization_term = 1e-10;
  38 |             for (const auto& cost : costs) {
  39 |                 normalization_term += std::exp(-(cost - min_cost) / lambda);
  40 |             }
  41 | 
  42 |             std::vector<double> softmax_costs(costs.size());
  43 | #pragma omp parallel for num_threads(thread_num)
  44 |             for (size_t i = 0; i < costs.size(); i++) {
  45 |                 softmax_costs[i] = std::exp(-(costs[i] - min_cost) / lambda) / normalization_term;
  46 |             }
  47 | 
  48 |             return softmax_costs;
  49 |         };
  50 |     };
  51 | 
  52 | }  // namespace cpu
  53 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\mppi_controller_ros.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <Eigen/Dense>
   5 | #include <algorithm>
   6 | #include <array>
   7 | #include <deque>
   8 | #include <iostream>
   9 | #include <limits>
  10 | #include <mutex>
  11 | #include <random>
  12 | #include <string>
  13 | #include <vector>
  14 | 
  15 | #include <ackermann_msgs/AckermannDriveStamped.h>
  16 | #include <geometry_msgs/PoseStamped.h>
  17 | #include <geometry_msgs/TransformStamped.h>
  18 | #include <mppi_metrics_msgs/MPPIMetrics.h>
  19 | #include <nav_msgs/Odometry.h>
  20 | #include <nav_msgs/Path.h>
  21 | #include <ros/ros.h>
  22 | #include <std_msgs/Bool.h>
  23 | #include <std_msgs/Float32.h>
  24 | #include <tf2/utils.h>
  25 | #include <tf2_eigen/tf2_eigen.h>
  26 | #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
  27 | #include <tf2_ros/transform_broadcaster.h>
  28 | #include <tf2_ros/transform_listener.h>
  29 | #include <visualization_msgs/Marker.h>
  30 | #include <visualization_msgs/MarkerArray.h>
  31 | #include <grid_map_core/GridMap.hpp>
  32 | #include <grid_map_ros/grid_map_ros.hpp>
  33 | 
  34 | #include "mppi_controller/StopWatch.hpp"
  35 | #include "mppi_controller/common.hpp"
  36 | #include "mppi_controller/forward_mppi.hpp"
  37 | #include "mppi_controller/reverse_mppi.hpp"
  38 | #include "mppi_controller/stein_variational_guided_mppi.hpp"
  39 | #include "mppi_controller/stein_variational_mpc.hpp"
  40 | 
  41 | namespace mppi {
  42 |     class MPPIControllerROS {
  43 |     public:
  44 |         MPPIControllerROS();
  45 |         ~MPPIControllerROS(){};
  46 | 
  47 |     private:
  48 |         struct RobotState {
  49 |             double x = 0.0;
  50 |             double y = 0.0;
  51 |             double yaw = 0.0;
  52 |             double vel = 0.0;
  53 |             double steer = 0.0;
  54 |         };
  55 | 
  56 |     private:
  57 |         std::mutex mtx_;
  58 | 
  59 |         /* ros system variables */
  60 |         ros::NodeHandle nh_;          //!< @brief ros public node handle
  61 |         ros::NodeHandle private_nh_;  //!< @brief ros private node handle
  62 | 
  63 |         std::string robot_frame_id_;
  64 |         std::string map_frame_id_;
  65 |         tf2_ros::Buffer tf_buffer_;
  66 |         tf2_ros::TransformListener tf_listener_;
  67 | 
  68 |         /* pub sub */
  69 |         ros::Subscriber sub_reference_sdf_;   //!< @brief reference sdf subscriber
  70 |         ros::Subscriber sub_odom_;            //!< @brief robot odom subscriber
  71 |         ros::Subscriber sub_occupancy_grid_;  //!< @brief costmap subscriber (nav_msgs::OccupancyGrid for costmap_2d)
  72 |         ros::Subscriber sub_grid_map_;        //!< @brief grid map subscriber (grid_map_msgs::GridMap for local costmap)
  73 |         ros::Timer timer_control_;            //!< @brief timer for control command commutation
  74 |         ros::Publisher pub_ackermann_cmd_;
  75 |         ros::Subscriber sub_activated_;
  76 |         ros::Subscriber sub_backward_point_;
  77 | 
  78 |         ros::Subscriber sub_start_cmd_;
  79 |         ros::Subscriber sub_stop_cmd_;
  80 | 
  81 |         // For debug
  82 |         StopWatch stop_watch_;                             //!< @brief stop watch for calculation time
  83 |         ros::Publisher pub_best_path_;                     //!< @brief best path topic publisher
  84 |         ros::Publisher pub_nominal_path_;                  //!< @brief nominal path topic publisher
  85 |         ros::Publisher pub_candidate_paths_;               //!< @brief candidate paths topic publisher
  86 |         ros::Publisher pub_proposal_state_distributions_;  //!< @brief proposal state distribution topic publisher
  87 |         ros::Publisher pub_control_covariances_;           //!< @brief control covariance topic publisher
  88 |         ros::Publisher pub_calculation_time_;              //!< @brief calculation time topic publisher
  89 |         ros::Publisher pub_speed_;                         //!< @brief robot speed topic publisher
  90 |         ros::Publisher pub_collision_rate_;                //!< @brief collision rate topic publisher
  91 |         ros::Publisher pub_cost_;                          //!< @brief cost topic publisher
  92 |         ros::Publisher pub_mppi_metrics_;                  //!< @brief mppi metrics topic publisher
  93 | 
  94 |         /*control system parametes*/
  95 |         double control_sampling_time_;  //!< @brief control interval [s]
  96 |         bool is_localize_less_mode_ = false;
  97 |         bool constant_speed_mode_ = false;
  98 |         bool is_visualize_mppi_ = false;
  99 |         bool is_start_ = true;
 100 |         const std::string obstacle_layer_name_ = "collision_layer";
 101 |         const std::string distance_field_layer_name_ = "distance_field";
 102 |         const std::string angle_field_layer_name_ = "angle_field";
 103 |         const std::string speed_field_layer_name_ = "speed_field";
 104 | 
 105 |         /* Variables */
 106 |         RobotState robot_state_;
 107 |         double reference_speed_ = 0.0;
 108 |         double collision_rate_threshold_ = 0.95;  // [0, 1] If collision rate is over this value, stop robot
 109 |         grid_map::GridMap obstacle_map_;          // value = [0, 100], 100: collision, 0: free (obstacle layer)
 110 |         grid_map::GridMap reference_sdf_;
 111 |         ackermann_msgs::AckermannDriveStamped control_msg_;
 112 | 
 113 |         double steer_1st_delay_ = 0.1;
 114 | 
 115 |         // For stuck detection
 116 |         std::deque<float> speed_deque_;
 117 |         int speed_deque_size_ = 10;
 118 |         float stuck_speed_threshold_ = 0.1;
 119 | 
 120 |         bool is_robot_state_ok_ = false;
 121 |         bool is_reference_sdf_ok_ = false;
 122 |         bool is_costmap_ok_ = false;
 123 |         bool is_activate_ad_ = false;
 124 |         bool is_simulation_ = false;
 125 | 
 126 |         std::unique_ptr<mppi::cpu::MPCTemplate> mpc_solver_ptr_;
 127 | 
 128 |         /**
 129 |          * @brief Main loop
 130 |          *
 131 |          */
 132 |         void timer_callback(const ros::TimerEvent&);
 133 | 
 134 |         void start_cmd_callback(const std_msgs::Empty& msg);
 135 | 
 136 |         void stop_cmd_callback(const std_msgs::Empty& msg);
 137 | 
 138 |         void callback_odom(const nav_msgs::Odometry& odom);
 139 | 
 140 |         void callback_odom_with_pose(const nav_msgs::Odometry& odom);
 141 | 
 142 |         void callback_reference_sdf(const grid_map_msgs::GridMap& grid_map);
 143 | 
 144 |         void callback_grid_map(const grid_map_msgs::GridMap& grid_map);
 145 | 
 146 |         void callback_activate_signal(const std_msgs::Bool& is_activate);
 147 | 
 148 |         void publish_candidate_paths(const std::vector<mppi::cpu::StateSeq>& state_seq_batch,
 149 |                                      const std::vector<double>& weights,
 150 |                                      const ros::Publisher& publisher) const;
 151 | 
 152 |         void publish_traj(const mppi::cpu::StateSeq& state_seq,
 153 |                           const std::string& name_space,
 154 |                           const std::string& rgb,
 155 |                           const ros::Publisher& publisher) const;
 156 | 
 157 |         void publish_path(const mppi::cpu::StateSeq& state_seq,
 158 |                           const std::string& name_space,
 159 |                           const std::string& rgb,
 160 |                           const ros::Publisher& publisher) const;
 161 | 
 162 |         void publish_control_covs(const mppi::cpu::StateSeq& mean,
 163 |                                   const mppi::cpu::ControlSeqCovMatrices& cov_matrices,
 164 |                                   const ros::Publisher& publisher) const;
 165 | 
 166 |         void publish_state_seq_dists(const mppi::cpu::StateSeq& state_seq,
 167 |                                      const mppi::cpu::XYCovMatrices& cov_matrices,
 168 |                                      const ros::Publisher& publisher) const;
 169 |     };
 170 | 
 171 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\prior_samples_with_costs.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <Eigen/Dense>
   5 | #include <array>
   6 | #include <iostream>
   7 | #include <memory>
   8 | #include <random>
   9 | #include <vector>
  10 | #include "mppi_controller/common.hpp"
  11 | 
  12 | namespace mppi {
  13 | namespace cpu {
  14 | 
  15 |     class PriorSamplesWithCosts {
  16 |     private:
  17 |         const int thread_num_;
  18 |         const size_t num_samples_;
  19 |         const size_t prediction_horizon_;
  20 | 
  21 |         // For random noise generation
  22 |         std::vector<std::mt19937> rngs_;
  23 |         std::unique_ptr<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>> normal_dists_ptr_;
  24 |         const double non_biased_sampling_rate_;
  25 |         const std::array<double, CONTROL_SPACE::dim> max_control_inputs_;
  26 |         const std::array<double, CONTROL_SPACE::dim> min_control_inputs_;
  27 |         std::discrete_distribution<> discrete_dist_;
  28 | 
  29 |         ControlSeq control_seq_mean_;
  30 |         ControlSeqCovMatrices control_seq_cov_matrices_;
  31 |         ControlSeqCovMatrices control_seq_inv_cov_matrices_;
  32 | 
  33 |     public:
  34 |         ControlSeqBatch noise_seq_samples_;
  35 |         ControlSeqBatch noised_control_seq_samples_;
  36 |         std::vector<double> costs_;
  37 | 
  38 |     public:
  39 |         PriorSamplesWithCosts(const size_t& num_samples,
  40 |                               const size_t& prediction_horizon,
  41 |                               const std::array<double, CONTROL_SPACE::dim>& max_control_inputs,
  42 |                               const std::array<double, CONTROL_SPACE::dim>& min_control_inputs,
  43 |                               const double& non_biased_sampling_rate,
  44 |                               const int& thread_num,
  45 |                               const int& seed = 42);
  46 | 
  47 |         ~PriorSamplesWithCosts() = default;
  48 | 
  49 |         void random_sampling(const ControlSeq& control_seq_mean, const ControlSeqCovMatrices& control_seq_cov_matrices);
  50 | 
  51 |         std::vector<int> random_sample_choice(const size_t& num_samples, const std::vector<double>& probabilities);
  52 | 
  53 |         std::vector<double> get_costs_with_control_term(const double& lambda, const double& alpha, const ControlSeq& nominal_control_seq) const;
  54 | 
  55 |         /* Setters */
  56 |         void shrink_copy_from(const PriorSamplesWithCosts& source_samples, const std::vector<int>& indices);
  57 | 
  58 |         void inflate_copy_from(const PriorSamplesWithCosts& source_samples, const ControlSeqCovMatrices& noise_covs);
  59 | 
  60 |         void set_control_seq_mean(const ControlSeq& control_seq_mean);
  61 | 
  62 |         void set_control_seq_cov_matrices(const ControlSeqCovMatrices& control_seq_cov_matrices);
  63 | 
  64 |         /* Getters */
  65 |         size_t get_num_samples() const { return num_samples_; }
  66 | 
  67 |         size_t get_prediction_horizon() const { return prediction_horizon_; }
  68 | 
  69 |         ControlSeq get_zero_control_seq() const { return Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim); }
  70 | 
  71 |         ControlSeq get_constant_control_seq(const std::array<double, CONTROL_SPACE::dim>& value) const {
  72 |             ControlSeq control_seq = get_zero_control_seq();
  73 |             for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
  74 |                 for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
  75 |                     control_seq(i, j) = value[j];
  76 |                 }
  77 |             }
  78 |             return control_seq;
  79 |         }
  80 | 
  81 |         ControlSeqBatch get_zero_control_seq_batch() const {
  82 |             return std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  83 |                 num_samples_, Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim));
  84 |         }
  85 | 
  86 |         StateSeq get_zero_state_seq() const { return Eigen::MatrixXd::Zero(prediction_horizon_, STATE_SPACE::dim); }
  87 | 
  88 |         StateSeqBatch get_zero_state_seq_batch() const {
  89 |             return std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  90 |                 num_samples_, Eigen::MatrixXd::Zero(prediction_horizon_, STATE_SPACE::dim));
  91 |         }
  92 | 
  93 |         ControlSeqCovMatrices get_zero_control_seq_cov_matrices() const {
  94 |             return std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  95 |                 prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
  96 |         }
  97 | 
  98 |         ControlSeqCovMatrices get_constant_control_seq_cov_matrices(const std::array<double, CONTROL_SPACE::dim>& diag) const {
  99 |             ControlSeqCovMatrices control_seq_covs = get_zero_control_seq_cov_matrices();
 100 |             for (auto& cov : control_seq_covs) {
 101 |                 for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
 102 |                     cov(i, i) = diag[i];
 103 |                 }
 104 |             }
 105 |             return control_seq_covs;
 106 |         }
 107 | 
 108 |         ControlSeq get_mean() const { return control_seq_mean_; }
 109 | 
 110 |         ControlSeqCovMatrices get_cov_matrices() const { return control_seq_cov_matrices_; }
 111 | 
 112 |         ControlSeqCovMatrices get_inv_cov_matrices() const { return control_seq_inv_cov_matrices_; }
 113 | 
 114 |         ControlSeq clipped_control_seq(const ControlSeq& control_seq) const {
 115 |             ControlSeq clipped_control_seq = control_seq;
 116 |             for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
 117 |                 for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
 118 |                     clipped_control_seq(k, j) = std::clamp(control_seq(k, j), min_control_inputs_[j], max_control_inputs_[j]);
 119 |                 }
 120 |             }
 121 |             return clipped_control_seq;
 122 |         }
 123 |     };
 124 | }  // namespace cpu
 125 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\reverse_mppi.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <algorithm>
   5 | #include <iostream>
   6 | #include <limits>
   7 | #include <mutex>
   8 | #include <string>
   9 | #include <vector>
  10 | 
  11 | #include <Eigen/Dense>
  12 | #include <array>
  13 | #include <grid_map_core/GridMap.hpp>
  14 | #include <memory>
  15 | #include <utility>
  16 | 
  17 | #include "mppi_controller/common.hpp"
  18 | #include "mppi_controller/mpc_base.hpp"
  19 | #include "mppi_controller/mpc_template.hpp"
  20 | #include "mppi_controller/prior_samples_with_costs.hpp"
  21 | 
  22 | namespace mppi {
  23 | namespace cpu {
  24 | 
  25 |     /**
  26 |      * @brief Reverse MPPI with sample rejection, by K. Kobayashi, https://arxiv.org/abs/2212.04298
  27 |      */
  28 |     class ReverseMPPI : public MPCTemplate {
  29 |     public:
  30 |         ReverseMPPI(const Params::Common& common_params, const Params::ReverseMPPI& reverse_mppi_params);
  31 |         ~ReverseMPPI(){};
  32 | 
  33 |         /**
  34 |          * @brief solve mppi problem and return optimal control sequence
  35 |          * @param initial_state initial state
  36 |          * @return optimal control sequence and collision rate
  37 |          */
  38 |         std::pair<ControlSeq, double> solve(const State& initial_state) override;
  39 |         /**
  40 |          * @brief set obstacle map and reference map
  41 |          */
  42 |         void set_obstacle_map(const grid_map::GridMap& obstacle_map) override;
  43 | 
  44 |         void set_reference_map(const grid_map::GridMap& reference_map) override;
  45 | 
  46 |         /**
  47 |          * @brief get state sequence candidates and their weights, top num_samples
  48 |          * @return std::pair<std::vector<StateSeq>, std::vector<double>> state sequence candidates and their weights
  49 |          */
  50 |         std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples) const override;
  51 | 
  52 |         std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state,
  53 |                                                                         const ControlSeq& control_input_seq) const override;
  54 | 
  55 |         ControlSeqCovMatrices get_cov_matrices() const override;
  56 | 
  57 |         ControlSeq get_control_seq() const override;
  58 | 
  59 |         std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const override;
  60 | 
  61 |     private:
  62 |         const size_t prediction_step_size_;  //!< @brief prediction step size
  63 |         const int thread_num_;               //!< @brief number of thread for parallel computation
  64 | 
  65 |         // MPPI parameters
  66 |         const double negative_ratio_;
  67 |         const bool is_sample_rejection_;
  68 |         const double sample_inflation_ratio_;  // sample inflation for sample rejection
  69 |         const int iteration_num_;              //!< @brief number of iteration for MD method
  70 |         const double step_size_;               //!< @brief step size for MD method
  71 |         const double warm_start_ratio_;        //!< @brief warm start ratio
  72 |         const double lambda_;                  //!< @brief temperature parameter [0, inf) of free energy.
  73 |         const double alpha_;  //!< @brief weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal
  74 |                               //!< control sequence.
  75 |         const double non_biased_sampling_rate_;  //!< @brief non-previous-control-input-biased sampling rate [0, 1], Add random noise to control
  76 |                                                  //!< sequence with this rate.
  77 |         const double steer_cov_;                 //!< @brief covariance of steering angle noise
  78 |         // double accel_cov_;                      //!< @brief covariance of acceleration noise
  79 | 
  80 |         // Internal variables
  81 |         ControlSeq prev_control_seq_;
  82 |         ControlSeqCovMatrices prev_covs_;
  83 |         ControlSeq prev_rejected_mean_;
  84 |         ControlSeqCovMatrices prev_rejected_covs_;
  85 |         std::vector<double> weights_ = {};  // for visualization
  86 | 
  87 |         // Libraries
  88 |         std::unique_ptr<MPCBase> mpc_base_ptr_;
  89 |         std::unique_ptr<PriorSamplesWithCosts> inflated_samples_ptr_;
  90 |         std::unique_ptr<PriorSamplesWithCosts> prior_samples_ptr_;
  91 | 
  92 |     private:
  93 |         std::pair<std::vector<double>, std::vector<double>> calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const;
  94 | 
  95 |         std::pair<ControlSeq, ControlSeqCovMatrices> estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const;
  96 | 
  97 |         std::pair<ControlSeq, ControlSeqCovMatrices> weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
  98 |                                                                              const std::vector<double>& weights) const;
  99 | 
 100 |         std::pair<ControlSeq, ControlSeqCovMatrices> md_update(const ControlSeq& prior_mean,
 101 |                                                                const ControlSeqCovMatrices& prior_covs,
 102 |                                                                const ControlSeq& weighted_mean,
 103 |                                                                const ControlSeqCovMatrices& weighted_covs,
 104 |                                                                const double step_size) const;
 105 | 
 106 |         ControlSeq normal_pdf(const ControlSeq& x, const ControlSeq& mean, const ControlSeqCovMatrices& covs) const;
 107 | 
 108 |         ControlSeq interpolate(const ControlSeq& x1, const ControlSeq& x2, const double& ratio) const;
 109 |         ControlSeqCovMatrices interpolate(const ControlSeqCovMatrices& covs1, const ControlSeqCovMatrices& covs2, const double& ratio) const;
 110 |     };
 111 | 
 112 | }  // namespace cpu
 113 | 
 114 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\stein_variational_guided_mppi.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <algorithm>
   5 | #include <iostream>
   6 | #include <limits>
   7 | #include <mutex>
   8 | #include <string>
   9 | #include <vector>
  10 | 
  11 | #include <Eigen/Dense>
  12 | #include <array>
  13 | #include <grid_map_core/GridMap.hpp>
  14 | #include <memory>
  15 | #include <utility>
  16 | 
  17 | #include "mppi_controller/common.hpp"
  18 | #include "mppi_controller/mpc_base.hpp"
  19 | #include "mppi_controller/mpc_template.hpp"
  20 | #include "mppi_controller/prior_samples_with_costs.hpp"
  21 | 
  22 | namespace mppi {
  23 | namespace cpu {
  24 |     /**
  25 |      * @brief Stein Variational Guided MPPI, K. Honda
  26 |      */
  27 |     class SVGuidedMPPI : public MPCTemplate {
  28 |     public:
  29 |         SVGuidedMPPI(const Params::Common& common_params, const Params::SVGuidedMPPI& svg_mppi_params);
  30 |         ~SVGuidedMPPI(){};
  31 | 
  32 |         /**
  33 |          * @brief solve mppi problem and return optimal control sequence
  34 |          * @param initial_state initial state
  35 |          * @return optimal control sequence and collision rate
  36 |          */
  37 |         std::pair<ControlSeq, double> solve(const State& initial_state) override;
  38 |         /**
  39 |          * @brief set obstacle map and reference map
  40 |          */
  41 |         void set_obstacle_map(const grid_map::GridMap& obstacle_map) override;
  42 | 
  43 |         void set_reference_map(const grid_map::GridMap& reference_map) override;
  44 | 
  45 |         /**
  46 |          * @brief get state sequence candidates and their weights, top num_samples
  47 |          * @return std::pair<std::vector<StateSeq>, std::vector<double>> state sequence candidates and their weights
  48 |          */
  49 |         std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples) const override;
  50 | 
  51 |         std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state,
  52 |                                                                         const ControlSeq& control_input_seq) const override;
  53 | 
  54 |         ControlSeqCovMatrices get_cov_matrices() const override;
  55 | 
  56 |         ControlSeq get_control_seq() const override;
  57 | 
  58 |         std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const override;
  59 | 
  60 |     private:
  61 |         const size_t prediction_step_size_;  //!< @brief prediction step size
  62 |         const int thread_num_;               //!< @brief number of thread for parallel computation
  63 |         // MPPI parameters
  64 |         const double lambda_;  //!< @brief temperature parameter [0, inf) of free energy.
  65 |         const double alpha_;
  66 |         // temperature parameter is a balancing term between control cost and state cost.
  67 |         // If lambda_ is small, weighted state cost is dominant, thus control sequence is more aggressive.
  68 |         // If lambda_ is large, weighted control cost is dominant, thus control sequence is more smooth.
  69 |         const double non_biased_sampling_rate_;  //!< @brief non-previous-control-input-biased sampling rate [0, 1], Add random noise to control
  70 |                                                  //!< sequence with this rate.
  71 |         const double steer_cov_;                 //!< @brief covariance of steering angle noise
  72 |         // double accel_cov_;                      //!< @brief covariance of acceleration noise
  73 | 
  74 |         // Stein Variational MPC parameters
  75 |         const size_t sample_num_for_grad_estimation_;
  76 |         const double grad_lambda_;
  77 |         const double steer_cov_for_grad_estimation_;
  78 |         const double svgd_step_size_;
  79 |         const int num_svgd_iteration_;
  80 |         const bool is_use_nominal_solution_;
  81 |         const bool is_covariance_adaptation_;
  82 |         const double gaussian_fitting_lambda_;
  83 |         const double min_steer_cov_;
  84 |         const double max_steer_cov_;
  85 | 
  86 |         // Internal vars
  87 |         ControlSeq prev_control_seq_;
  88 |         ControlSeq nominal_control_seq_;
  89 |         std::vector<double> weights_ = {};  // for visualization
  90 | 
  91 |         // Libraries
  92 |         std::unique_ptr<MPCBase> mpc_base_ptr_;
  93 |         std::unique_ptr<PriorSamplesWithCosts> prior_samples_ptr_;
  94 |         std::unique_ptr<PriorSamplesWithCosts> guide_samples_ptr_;
  95 |         std::vector<std::unique_ptr<PriorSamplesWithCosts>> grad_sampler_ptrs_;
  96 | 
  97 |     private:
  98 |         ControlSeq approx_grad_log_likelihood(const ControlSeq& mean_seq,
  99 |                                               const ControlSeq& noised_seq,
 100 |                                               const ControlSeqCovMatrices& inv_covs,
 101 |                                               const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
 102 |                                               PriorSamplesWithCosts* sampler) const;
 103 | 
 104 |         ControlSeqBatch approx_grad_posterior_batch(const PriorSamplesWithCosts& samples,
 105 |                                                     const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const;
 106 | 
 107 |         std::pair<ControlSeq, ControlSeqCovMatrices> weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
 108 |                                                                              const std::vector<double>& weights) const;
 109 | 
 110 |         std::pair<ControlSeq, ControlSeqCovMatrices> estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const;
 111 | 
 112 |         std::vector<double> calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs, const ControlSeq& nominal_control_seq) const;
 113 | 
 114 |         std::pair<double, double> gaussian_fitting(const std::vector<double>& x, const std::vector<double>& y) const;
 115 |     };
 116 | 
 117 | }  // namespace cpu
 118 | 
 119 | }  // namespace mppi

```

`src\mppi_controller\include\mppi_controller\stein_variational_mpc.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <algorithm>
   5 | #include <iostream>
   6 | #include <limits>
   7 | #include <mutex>
   8 | #include <string>
   9 | #include <vector>
  10 | 
  11 | #include <Eigen/Dense>
  12 | #include <array>
  13 | #include <grid_map_core/GridMap.hpp>
  14 | #include <memory>
  15 | #include <utility>
  16 | 
  17 | #include "mppi_controller/common.hpp"
  18 | #include "mppi_controller/mpc_base.hpp"
  19 | #include "mppi_controller/mpc_template.hpp"
  20 | #include "mppi_controller/prior_samples_with_costs.hpp"
  21 | 
  22 | namespace mppi {
  23 | namespace cpu {
  24 |     /**
  25 |      * @brief Stein Variational MPC, by A. Lambert, https://arxiv.org/abs/2011.07641
  26 |      */
  27 |     class SteinVariationalMPC : public MPCTemplate {
  28 |     public:
  29 |         SteinVariationalMPC(const Params::Common& common_params, const Params::SteinVariationalMPC& forward_mppi_params);
  30 |         ~SteinVariationalMPC(){};
  31 | 
  32 |         /**
  33 |          * @brief solve mppi problem and return optimal control sequence
  34 |          * @param initial_state initial state
  35 |          * @return optimal control sequence and collision rate
  36 |          */
  37 |         std::pair<ControlSeq, double> solve(const State& initial_state) override;
  38 |         /**
  39 |          * @brief set obstacle map and reference map
  40 |          */
  41 |         void set_obstacle_map(const grid_map::GridMap& obstacle_map) override;
  42 | 
  43 |         void set_reference_map(const grid_map::GridMap& reference_map) override;
  44 | 
  45 |         /**
  46 |          * @brief get state sequence candidates and their weights, top num_samples
  47 |          * @return std::pair<std::vector<StateSeq>, std::vector<double>> state sequence candidates and their weights
  48 |          */
  49 |         std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples) const override;
  50 | 
  51 |         std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state,
  52 |                                                                         const ControlSeq& control_input_seq) const override;
  53 | 
  54 |         ControlSeqCovMatrices get_cov_matrices() const override;
  55 | 
  56 |         ControlSeq get_control_seq() const override;
  57 | 
  58 |         std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const override;
  59 | 
  60 |     private:
  61 |         const size_t prediction_step_size_;  //!< @brief prediction step size
  62 |         const int thread_num_;               //!< @brief number of thread for parallel computation
  63 |         // MPPI parameters
  64 |         const double lambda_;  //!< @brief temperature parameter [0, inf) of free energy.
  65 |         // temperature parameter is a balancing term between control cost and state cost.
  66 |         const double alpha_;  //!< @brief weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal
  67 |                               //!< control sequence.
  68 |         // If lambda_ is small, weighted state cost is dominant, thus control sequence is more aggressive.
  69 |         // If lambda_ is large, weighted control cost is dominant, thus control sequence is more smooth.
  70 |         const double non_biased_sampling_rate_;  //!< @brief non-previous-control-input-biased sampling rate [0, 1], Add random noise to control
  71 |                                                  //!< sequence with this rate.
  72 |         const double steer_cov_;                 //!< @brief covariance of steering angle noise
  73 |         // double accel_cov_;                      //!< @brief covariance of acceleration noise
  74 | 
  75 |         // Stein Variational MPC parameters
  76 |         const int num_svgd_iteration_;
  77 |         const double steer_cov_for_grad_estimation_;
  78 |         const double svgd_step_size_;
  79 |         const bool is_max_posterior_estimation_;
  80 | 
  81 |         // Internal vars
  82 |         ControlSeq prev_control_seq_;
  83 |         std::vector<double> weights_ = {};  // for visualization
  84 | 
  85 |         // Libraries
  86 |         std::unique_ptr<MPCBase> mpc_base_ptr_;
  87 |         std::unique_ptr<PriorSamplesWithCosts> prior_samples_ptr_;
  88 |         std::vector<std::unique_ptr<PriorSamplesWithCosts>> grad_sampler_ptrs_;
  89 | 
  90 |     private:
  91 |         ControlSeq approx_grad_log_likelihood(const ControlSeq& mean_seq,
  92 |                                               const ControlSeq& noised_seq,
  93 |                                               const ControlSeqCovMatrices& inv_covs,
  94 |                                               const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
  95 |                                               PriorSamplesWithCosts* sampler) const;
  96 | 
  97 |         ControlSeq grad_log_normal_dist(const ControlSeq& sample, const ControlSeq& prior_mean, const ControlSeqCovMatrices& prior_covs) const;
  98 | 
  99 |         ControlSeqBatch approx_grad_posterior_batch(const PriorSamplesWithCosts& samples,
 100 |                                                     const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const;
 101 | 
 102 |         double RBF_kernel(const ControlSeq& seq1, const ControlSeq& seq2, const double& h) const;
 103 | 
 104 |         ControlSeq grad_RBF_kernel(const ControlSeq& seq, const ControlSeq& seq_const, const double& h) const;
 105 | 
 106 |         ControlSeqBatch phi_batch(const PriorSamplesWithCosts& samples, const ControlSeqBatch& grad_posterior_batch) const;
 107 | 
 108 |         std::pair<ControlSeq, ControlSeqCovMatrices> weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
 109 |                                                                              const std::vector<double>& weights) const;
 110 | 
 111 |         std::pair<ControlSeq, ControlSeqCovMatrices> estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const;
 112 | 
 113 |         std::vector<double> calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const;
 114 |     };
 115 | 
 116 | }  // namespace cpu
 117 | 
 118 | }  // namespace mppi

```

`src\mppi_controller\launch\mppi_controller.launch`:

```launch
   1 | <launch>
   2 |     <arg name="mppi_param_path" default="$(find mppi_controller)/config/mppi_controller.yaml"/>
   3 |     <arg name="is_simulation" default="false"/>
   4 |     <arg name="is_localize_less_mode" default="false"/>
   5 | 
   6 |     <node pkg="mppi_controller" type="mppi_controller_node" name="mppi_controller" output="screen">
   7 |         <rosparam command="load" file="$(arg mppi_param_path)"/>
   8 |         <param name="is_simulation" value="$(arg is_simulation)"/>
   9 |         <param name="is_localize_less_mode" value="$(arg is_localize_less_mode)"/>
  10 |     </node>
  11 | </launch>

```

`src\mppi_controller\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>mppi_controller</name>
   4 |   <version>0.0.0</version>
   5 |   <description>Model Predictive Path Integral control</description>
   6 | 
   7 |   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8 |   <!-- Example:  -->
   9 |   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 |   <maintainer email="0905honda@gmail.com">Kohei Honda</maintainer>
  11 | 
  12 | 
  13 |   <!-- One license tag required, multiple allowed, one license per tag -->
  14 |   <!-- Commonly used license strings: -->
  15 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  16 |   <license>MIT</license>
  17 | 
  18 |   <buildtool_depend>catkin</buildtool_depend>
  19 |   <build_depend>ackermann_msgs</build_depend>
  20 |   <build_depend>geometry_msgs</build_depend>
  21 |   <build_depend>nav_msgs</build_depend>
  22 |   <build_depend>roscpp</build_depend>
  23 |   <build_depend>std_msgs</build_depend>
  24 |   <build_depend>tf2_geometry_msgs</build_depend>
  25 |   <build_depend>tf2_ros</build_depend>
  26 |   <build_depend>grid_map</build_depend>
  27 |   <build_depend>grid_map_ros</build_depend>
  28 |   <build_depend>grid_map_msgs</build_depend>
  29 |   <build_depend>grid_map_rviz_plugin</build_depend>
  30 |   <build_depend>grid_map_visualization</build_depend>
  31 |   <build_depend>mppi_metrics_msgs</build_depend>
  32 | 
  33 |   <build_export_depend>ackermann_msgs</build_export_depend>
  34 |   <build_export_depend>geometry_msgs</build_export_depend>
  35 |   <build_export_depend>nav_msgs</build_export_depend>
  36 |   <build_export_depend>roscpp</build_export_depend>
  37 |   <build_export_depend>std_msgs</build_export_depend>
  38 |   <build_export_depend>tf2_geometry_msgs</build_export_depend>
  39 |   <build_export_depend>tf2_ros</build_export_depend>
  40 |   <build_export_depend>grid_map</build_export_depend>
  41 |   <build_export_depend>grid_map_ros</build_export_depend>
  42 |   <build_export_depend>grid_map_msgs</build_export_depend>
  43 |   <build_export_depend>grid_map_rviz_plugin</build_export_depend>
  44 |   <build_export_depend>grid_map_visualization</build_export_depend>
  45 |   <build_export_depend>mppi_metrics_msgs</build_export_depend>
  46 |   
  47 |   <exec_depend>ackermann_msgs</exec_depend>
  48 |   <exec_depend>geometry_msgs</exec_depend>
  49 |   <exec_depend>nav_msgs</exec_depend>
  50 |   <exec_depend>roscpp</exec_depend>
  51 |   <exec_depend>std_msgs</exec_depend>
  52 |   <exec_depend>tf2_geometry_msgs</exec_depend>
  53 |   <exec_depend>tf2_ros</exec_depend>
  54 |   <exec_depend>grid_map</exec_depend>
  55 |   <exec_depend>grid_map_ros</exec_depend>
  56 |   <exec_depend>grid_map_msgs</exec_depend>
  57 |   <exec_depend>grid_map_rviz_plugin</exec_depend>
  58 |   <exec_depend>grid_map_visualization</exec_depend>
  59 |   <exec_depend>mppi_metrics_msgs</exec_depend>
  60 | 
  61 |   <!-- The export tag contains other, unspecified, tags -->
  62 |   <export>
  63 |     <!-- Other tools can request additional information be placed here -->
  64 | 
  65 |   </export>
  66 | </package>

```

`src\mppi_controller\src\forward_mppi.cpp`:

```cpp
   1 | #include "mppi_controller/forward_mppi.hpp"
   2 | 
   3 | namespace mppi {
   4 | namespace cpu {
   5 |     ForwardMPPI::ForwardMPPI(const Params::Common& common_params, const Params::ForwardMPPI& forward_mppi_params)
   6 |         : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
   7 |           thread_num_(common_params.thread_num),
   8 |           lambda_(forward_mppi_params.lambda),
   9 |           alpha_(forward_mppi_params.alpha),
  10 |           non_biased_sampling_rate_(forward_mppi_params.non_biased_sampling_rate),
  11 |           steer_cov_(forward_mppi_params.steer_cov),
  12 |           num_itr_for_grad_estimation_(forward_mppi_params.num_itr_for_grad_estimation),
  13 |           step_size_for_grad_estimation_(forward_mppi_params.step_size_for_grad_estimation),
  14 |           sample_num_for_grad_estimation_(forward_mppi_params.sample_num_for_grad_estimation),
  15 |           steer_cov_for_grad_estimation_(forward_mppi_params.steer_cov_for_grad_estimation) {
  16 |         const size_t sample_batch_num = static_cast<size_t>(forward_mppi_params.sample_batch_num);
  17 |         mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_batch_num);
  18 | 
  19 |         const double max_steer_angle = common_params.max_steer_angle;
  20 |         const double min_steer_angle = common_params.min_steer_angle;
  21 |         std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
  22 |         std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};
  23 |         prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
  24 |                                                                      non_biased_sampling_rate_, thread_num_);
  25 |         prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
  26 | 
  27 |         // initialize grad samplers
  28 |         for (size_t i = 0; i < sample_batch_num; i++) {
  29 |             grad_sampler_ptrs_.emplace_back(std::make_unique<PriorSamplesWithCosts>(
  30 |                 sample_num_for_grad_estimation_, prediction_step_size_, max_control_inputs, min_control_inputs, non_biased_sampling_rate_, 1, i));
  31 |         }
  32 |     }
  33 | 
  34 |     std::pair<ControlSeq, double> ForwardMPPI::solve(const State& initial_state) {
  35 |         int collision_num = 0;
  36 |         const std::array<double, CONTROL_SPACE::dim> control_cov_diag = {steer_cov_};
  37 | 
  38 |         ControlSeq control_seq_mean = prev_control_seq_;
  39 |         ControlSeqCovMatrices control_seq_cov_matrices = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);
  40 | 
  41 |         // generate random noised control sequences based on previous control sequence as mean
  42 |         prior_samples_ptr_->random_sampling(control_seq_mean, control_seq_cov_matrices);
  43 | 
  44 |         // Transport random samples with stein variational gradient descent
  45 |         auto func_calc_costs = [&](const PriorSamplesWithCosts& sampler) { return mpc_base_ptr_->calc_sample_costs(sampler, initial_state).first; };
  46 |         prior_samples_ptr_->noised_control_seq_samples_ =
  47 |             transport_samples(*prior_samples_ptr_, func_calc_costs, num_itr_for_grad_estimation_, step_size_for_grad_estimation_);
  48 |         // const ControlSeqBatch grad_kld_batch = grad_reverse_kld_batch(*prior_samples_ptr_, func_calc_costs);
  49 |         // prior_samples_ptr_->noised_control_seq_samples_ = transport_samples(*prior_samples_ptr_, grad_kld_batch,
  50 |         //                                                                     num_itr_for_grad_estimation_, step_size_for_grad_estimation_);
  51 | 
  52 |         // Predict and calculate trajectory costs
  53 |         auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
  54 |         prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
  55 |         collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });
  56 | 
  57 |         // calculate weights for each state sequence based on costs and prior samples
  58 |         const std::vector<double> weights = calc_weights(*prior_samples_ptr_);
  59 |         weights_ = weights;  // for visualization
  60 | 
  61 |         // calculate optimal control sequence by weighted sum of control sequences
  62 |         ControlSeq updated_control_seq = prior_samples_ptr_->get_zero_control_seq();
  63 |         for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
  64 |             updated_control_seq += weights[i] * prior_samples_ptr_->noised_control_seq_samples_.at(i);
  65 |         }
  66 | 
  67 |         // update mean control sequence for next iteration
  68 |         prev_control_seq_ = updated_control_seq;
  69 | 
  70 |         const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());
  71 | 
  72 |         return std::make_pair(updated_control_seq, collision_rate);
  73 |     }
  74 | 
  75 |     void ForwardMPPI::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };
  76 | 
  77 |     void ForwardMPPI::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };
  78 | 
  79 |     std::pair<std::vector<StateSeq>, std::vector<double>> ForwardMPPI::get_state_seq_candidates(const int& num_samples) const {
  80 |         return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
  81 |     }
  82 | 
  83 |     std::tuple<StateSeq, double, double, double> ForwardMPPI::get_predictive_seq(const State& initial_state,
  84 |                                                                                  const ControlSeq& control_input_seq) const {
  85 |         const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
  86 |         double input_error = 0.0;
  87 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
  88 |             input_error += (control_input_seq.row(i)).norm();
  89 |         }
  90 |         return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
  91 |     }
  92 | 
  93 |     ControlSeqCovMatrices ForwardMPPI::get_cov_matrices() const {
  94 |         const auto [_mean, cov] = estimate_mu_and_sigma(*prior_samples_ptr_);
  95 |         return cov;
  96 |     }
  97 | 
  98 |     ControlSeq ForwardMPPI::get_control_seq() const { return prev_control_seq_; }
  99 | 
 100 |     std::pair<StateSeq, XYCovMatrices> ForwardMPPI::get_proposed_state_distribution() const { return mpc_base_ptr_->get_proposed_distribution(); }
 101 | 
 102 |     // == private functions ==
 103 |     std::vector<double> ForwardMPPI::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const {
 104 |         // calculate costs with control cost term
 105 |         const std::vector<double> costs_with_control_term =
 106 |             prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, prior_samples_ptr_->get_zero_control_seq());
 107 | 
 108 |         // softmax weights
 109 |         return softmax(costs_with_control_term, lambda_, thread_num_);
 110 |     }
 111 | 
 112 |     std::vector<int> ForwardMPPI::get_top_indices(const std::vector<double>& values, const int& num) const {
 113 |         std::vector<int> indices(values.size());
 114 |         std::iota(indices.begin(), indices.end(), 0);
 115 | 
 116 |         std::partial_sort(indices.begin(), indices.begin() + num, indices.end(), [&](int i1, int i2) { return values[i1] > values[i2]; });
 117 | 
 118 |         indices.resize(num);
 119 | 
 120 |         return indices;
 121 |     }
 122 | 
 123 |     ControlSeq ForwardMPPI::grad_reverse_kld(const ControlSeq& mean_seq,
 124 |                                              const ControlSeq& noised_seq,
 125 |                                              const ControlSeqCovMatrices& inv_covs,
 126 |                                              const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
 127 |                                              PriorSamplesWithCosts* sampler) const {
 128 |         const ControlSeqCovMatrices grad_cov = sampler->get_constant_control_seq_cov_matrices({steer_cov_for_grad_estimation_});
 129 | 
 130 |         // generate gaussian random samples, center of which is input_seq
 131 |         sampler->random_sampling(noised_seq, grad_cov);
 132 | 
 133 |         // calculate forward simulation and costs
 134 |         sampler->costs_ = calc_costs(*sampler);
 135 | 
 136 |         double sum_of_costs = 0.0;
 137 |         ControlSeq sum_of_grads = mean_seq * 0.0;
 138 |         const ControlSeqCovMatrices sampler_inv_covs = sampler->get_inv_cov_matrices();
 139 |         for (size_t i = 0; i < sampler->get_num_samples(); i++) {
 140 |             double cost_with_control_term = (sampler->costs_[i]) / lambda_;
 141 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 142 |                 const double control_term = mean_seq.row(j) * inv_covs[j] * sampler->noised_control_seq_samples_[i].row(j).transpose();
 143 |                 cost_with_control_term += control_term;
 144 |             }
 145 |             const double exp_cost = std::exp(-cost_with_control_term);
 146 |             sum_of_costs += exp_cost;
 147 | 
 148 |             ControlSeq grad_log_gaussian = mean_seq * 0.0;
 149 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 150 |                 grad_log_gaussian.row(j) = exp_cost * sampler_inv_covs[j] * (sampler->noised_control_seq_samples_[i] - noised_seq).row(j).transpose();
 151 |             }
 152 |             sum_of_grads += grad_log_gaussian;
 153 |         }
 154 | 
 155 |         const ControlSeq phi = -sum_of_grads / (sum_of_costs + 1e-10);
 156 | 
 157 |         return phi;
 158 |     }
 159 | 
 160 |     ControlSeqBatch ForwardMPPI::grad_reverse_kld_batch(const PriorSamplesWithCosts& samples,
 161 |                                                         const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const {
 162 |         ControlSeqBatch grad_kld = samples.get_zero_state_seq_batch();
 163 |         const ControlSeq mean = samples.get_mean();
 164 |         const ControlSeqCovMatrices inv_covs = samples.get_inv_cov_matrices();
 165 | #pragma omp parallel for num_threads(thread_num_)
 166 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 167 |             grad_kld[i] = grad_reverse_kld(mean, samples.noised_control_seq_samples_[i], inv_covs, calc_costs, grad_sampler_ptrs_.at(i).get());
 168 |         }
 169 | 
 170 |         return grad_kld;
 171 |     }
 172 | 
 173 |     ControlSeqBatch ForwardMPPI::transport_samples(const PriorSamplesWithCosts& samples_with_cost,
 174 |                                                    const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
 175 |                                                    const int& num_itr,
 176 |                                                    const double& step_size) {
 177 |         // If all cost is too small, return samples as it is
 178 |         // const double max_cost = *std::max_element(samples_with_cost.costs_.begin(), samples_with_cost.costs_.end());
 179 |         // if (max_cost < 1e-5)
 180 |         // {
 181 |         //     return samples_with_cost.noised_control_seq_samples_;
 182 |         // }
 183 | 
 184 |         ControlSeqBatch transported_samples = samples_with_cost.noised_control_seq_samples_;
 185 |         const ControlSeq mean = samples_with_cost.get_mean();
 186 |         const ControlSeqCovMatrices inv_covs = samples_with_cost.get_inv_cov_matrices();
 187 |         for (int i = 0; i < num_itr; i++) {
 188 | #pragma omp parallel for num_threads(thread_num_)
 189 |             for (size_t j = 0; j < transported_samples.size(); j++) {
 190 |                 const ControlSeq grad = grad_reverse_kld(mean, transported_samples[j], inv_covs, calc_costs, grad_sampler_ptrs_.at(j).get());
 191 | 
 192 |                 transported_samples[j] = transported_samples[j] - step_size * grad;
 193 |             }
 194 |         }
 195 | 
 196 |         return transported_samples;
 197 |     }
 198 | 
 199 |     ControlSeqBatch ForwardMPPI::transport_samples(const PriorSamplesWithCosts& samples,
 200 |                                                    const ControlSeqBatch& grad_kld_batch,
 201 |                                                    const int& num_itr,
 202 |                                                    const double& step_size) const {
 203 |         ControlSeqBatch transported_samples = samples.noised_control_seq_samples_;
 204 | 
 205 |         for (int i = 0; i < num_itr; i++) {
 206 | #pragma omp parallel for num_threads(thread_num_)
 207 |             for (size_t j = 0; j < transported_samples.size(); j++) {
 208 |                 transported_samples[j] = transported_samples[j] - step_size * grad_kld_batch[j];
 209 |             }
 210 |         }
 211 | 
 212 |         return transported_samples;
 213 |     }
 214 | 
 215 |     std::pair<ControlSeq, ControlSeqCovMatrices> ForwardMPPI::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
 216 |         ControlSeq mu = samples.get_zero_control_seq();
 217 |         ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();
 218 | 
 219 | // calculate mean
 220 | #pragma omp parallel for num_threads(thread_num_)
 221 | 
 222 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 223 |             mu += samples.noised_control_seq_samples_[i];
 224 |         }
 225 |         mu /= static_cast<double>(samples.get_num_samples());
 226 | 
 227 |         // mu = samples.control_seq_mean_;
 228 | 
 229 | // calculate covariance matrices
 230 | #pragma omp parallel for num_threads(thread_num_)
 231 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 232 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 233 |                 sigma[j] += (samples.noised_control_seq_samples_[i].row(j) - mu.row(j)).transpose() *
 234 |                             (samples.noised_control_seq_samples_[i].row(j) - mu.row(j));
 235 |             }
 236 |         }
 237 | 
 238 |         for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 239 |             sigma[j] /= static_cast<double>(samples.get_num_samples());
 240 | 
 241 |             // add small value to avoid singular matrix
 242 |             sigma[j] += 1e-5 * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
 243 |         }
 244 | 
 245 |         return std::make_pair(mu, sigma);
 246 |     }
 247 | 
 248 | }  // namespace cpu
 249 | }  // namespace mppi

```

`src\mppi_controller\src\mpc_base.cpp`:

```cpp
   1 | #include "mppi_controller/mpc_base.hpp"
   2 | 
   3 | namespace mppi {
   4 | namespace cpu {
   5 |     // #### Public functions ####
   6 | 
   7 |     MPCBase::MPCBase(const Params::Common& params, const size_t& sample_num)
   8 |         : is_localize_less_mode_(params.is_localize_less_mode),
   9 |           thread_num_(params.thread_num),
  10 |           prediction_step_size_(static_cast<size_t>(params.prediction_step_size)),
  11 |           prediction_interval_(params.prediction_interval),
  12 |           reference_speed_(params.reference_speed),
  13 |           lr_(params.lr),
  14 |           lf_(params.lf),
  15 |           max_accel_(params.max_accel),
  16 |           min_accel_(params.min_accel),
  17 |           steer_delay_(params.steer_delay),
  18 |           steer_delay_steps_(static_cast<size_t>(std::ceil(steer_delay_ / prediction_interval_))),
  19 |           steer_delay_tau_(params.steer_1st_delay),
  20 |           q_dist_(params.q_dist),
  21 |           q_angle_(params.q_angle),
  22 |           // q_speed_(params.q_speed),
  23 |           collision_weight_(params.collision_weight),
  24 |           q_terminal_dist_(params.q_terminal_dist),
  25 |           q_terminal_angle_(params.q_terminal_angle)
  26 |     // q_terminal_speed_(params.q_terminal_speed),
  27 |     {
  28 |         // set parameters
  29 | 
  30 |         if (params.speed_prediction_mode == "constant") {
  31 |             speed_prediction_mode_ = SpeedPredictionMode::CONSTANT;
  32 |         } else if (params.speed_prediction_mode == "linear") {
  33 |             speed_prediction_mode_ = SpeedPredictionMode::LINEAR;
  34 |         } else if (params.speed_prediction_mode == "reference") {
  35 |             speed_prediction_mode_ = SpeedPredictionMode::REFERENCE;
  36 |         } else {
  37 |             std::cout << "[MPPI] Invalid speed prediction mode: " << params.speed_prediction_mode << std::endl;
  38 |             exit(1);
  39 |         }
  40 | 
  41 |         // check validate mode
  42 |         if (speed_prediction_mode_ == SpeedPredictionMode::REFERENCE && is_localize_less_mode_) {
  43 |             std::cout << "[MPPI] Invalid speed prediction mode: " << params.speed_prediction_mode << std::endl;
  44 |             std::cout << "[MPPI] Speed prediction mode must be constant or linear in localize less mode" << std::endl;
  45 |             exit(1);
  46 |         }
  47 | 
  48 |         // initialize inner variables
  49 |         global_state_seq_candidates_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  50 |             sample_num, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim));
  51 |         local_state_seq_candidates_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  52 |             sample_num, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim));
  53 |     }
  54 | 
  55 |     void MPCBase::set_obstacle_map(const grid_map::GridMap& obstacle_map) { obstacle_map_ = obstacle_map; }
  56 | 
  57 |     void MPCBase::set_reference_map(const grid_map::GridMap& reference_map) { reference_map_ = reference_map; }
  58 | 
  59 |     std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler, const State& init_state) {
  60 |         if (is_localize_less_mode_) {
  61 |             return calc_sample_costs(sampler, init_state, obstacle_map_, &local_state_seq_candidates_);
  62 |         } else {
  63 |             return calc_sample_costs(sampler, init_state, obstacle_map_, reference_map_, &global_state_seq_candidates_, &local_state_seq_candidates_);
  64 |         }
  65 |     }
  66 | 
  67 |     std::tuple<StateSeq, double, double> MPCBase::get_predictive_seq(const State& initial_state, const ControlSeq& control_input_seq) const {
  68 |         if (is_localize_less_mode_) {
  69 |             const StateSeq local_state_seq = predict_state_seq(control_input_seq, initial_state, reference_map_);
  70 |             const auto [cost, collision_cost] = state_cost(local_state_seq, obstacle_map_);
  71 |             return std::make_tuple(local_state_seq, cost - collision_cost, collision_cost);
  72 |         } else {
  73 |             StateSeq global_state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
  74 |             StateSeq local_sate_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
  75 |             predict_state_seq(control_input_seq, initial_state, reference_map_, &global_state_seq, &local_sate_seq);
  76 |             const auto [cost, collision_cost] = state_cost(global_state_seq, local_sate_seq, obstacle_map_, reference_map_);
  77 |             return std::make_tuple(global_state_seq, cost - collision_cost, collision_cost);
  78 |         }
  79 |     }
  80 | 
  81 |     std::pair<std::vector<StateSeq>, std::vector<double>> MPCBase::get_state_seq_candidates(const int& _num_samples,
  82 |                                                                                             const std::vector<double>& weights) const {
  83 |         if (weights.size() == 0) {
  84 |             std::cerr << "weights is empty" << std::endl;
  85 |             return std::make_pair(std::vector<StateSeq>(), std::vector<double>());
  86 |         }
  87 | 
  88 |         const int num_samples = std::min(static_cast<int>(weights.size()), _num_samples);
  89 | 
  90 |         std::vector<double> sorted_weights = weights;
  91 |         std::sort(sorted_weights.data(), sorted_weights.data() + sorted_weights.size());
  92 | 
  93 |         // get indices of top num_samples
  94 |         std::vector<int> indices;
  95 |         for (int i = 0; i < num_samples; i++) {
  96 |             const double weight = sorted_weights[sorted_weights.size() - 1 - i];
  97 |             const int index = std::distance(weights.data(), std::find(weights.data(), weights.data() + weights.size(), weight));
  98 |             indices.push_back(index);
  99 |         }
 100 | 
 101 |         std::vector<double> selected_weights(num_samples);
 102 |         std::vector<StateSeq> selected_state_seq_candidates(num_samples);
 103 |         for (int i = 0; i < num_samples; i++) {
 104 |             selected_weights[i] = weights[indices.at(i)];
 105 |             if (is_localize_less_mode_) {
 106 |                 selected_state_seq_candidates[i] = local_state_seq_candidates_.at(indices.at(i));
 107 |             } else {
 108 |                 selected_state_seq_candidates[i] = global_state_seq_candidates_.at(indices.at(i));
 109 |             }
 110 |         }
 111 | 
 112 |         return std::make_pair(selected_state_seq_candidates, selected_weights);
 113 |     }
 114 | 
 115 |     std::pair<StateSeq, XYCovMatrices> MPCBase::get_proposed_distribution() const {
 116 |         if (is_localize_less_mode_) {
 117 |             return calc_state_distribution(local_state_seq_candidates_);
 118 |         } else {
 119 |             return calc_state_distribution(global_state_seq_candidates_);
 120 |         }
 121 |     }
 122 | 
 123 |     // #### Private functions ####
 124 |     std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler,
 125 |                                                                                    const State& global_init_state,
 126 |                                                                                    const grid_map::GridMap& obstacle_map,
 127 |                                                                                    const grid_map::GridMap& reference_map,
 128 |                                                                                    StateSeqBatch* global_state_seq_candidates,
 129 |                                                                                    StateSeqBatch* local_state_seq_candidates) const {
 130 |         std::vector<double> costs(sampler.get_num_samples());
 131 |         std::vector<double> collision_costs(sampler.get_num_samples());
 132 | 
 133 |         // Rollout for each control sequence
 134 | #pragma omp parallel for num_threads(thread_num_)
 135 |         for (size_t i = 0; i < sampler.get_num_samples(); i++) {
 136 |             // Predict state sequence
 137 |             predict_state_seq(sampler.noised_control_seq_samples_[i], global_init_state, reference_map, &global_state_seq_candidates->at(i),
 138 |                               &local_state_seq_candidates->at(i));
 139 | 
 140 |             // Calculate cost
 141 |             const auto [cost, collision_cost] =
 142 |                 state_cost(global_state_seq_candidates->at(i), local_state_seq_candidates->at(i), obstacle_map, reference_map);
 143 |             costs.at(i) = cost;
 144 |             collision_costs.at(i) = collision_cost;
 145 |         }
 146 |         return std::make_pair(costs, collision_costs);
 147 |     }
 148 | 
 149 |     std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler,
 150 |                                                                                    const State& local_init_state,
 151 |                                                                                    const grid_map::GridMap& obstacle_map,
 152 |                                                                                    StateSeqBatch* local_state_seq_candidates) const {
 153 |         std::vector<double> costs(sampler.get_num_samples());
 154 |         std::vector<double> collision_costs(sampler.get_num_samples());
 155 | 
 156 | // Rollout for each control sequence
 157 | #pragma omp parallel for num_threads(thread_num_)
 158 |         for (size_t i = 0; i < sampler.get_num_samples(); i++) {
 159 |             // Predict state sequence
 160 |             local_state_seq_candidates->at(i) = predict_state_seq(sampler.noised_control_seq_samples_[i], local_init_state, obstacle_map);
 161 | 
 162 |             // calculate cost
 163 |             const auto [cost, collision_cost] = state_cost(local_state_seq_candidates->at(i), obstacle_map);
 164 |             costs.at(i) = cost;
 165 |             collision_costs.at(i) = collision_cost;
 166 |         }
 167 | 
 168 |         return std::make_pair(costs, collision_costs);
 169 |     }
 170 | 
 171 |     // Predict local or global state sequence from control sequence
 172 |     StateSeq MPCBase::predict_state_seq(const ControlSeq& control_seq, const State& init_state, const grid_map::GridMap& reference_map) const {
 173 |         StateSeq state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
 174 |         // observed state
 175 |         state_seq.row(0) = init_state;
 176 | 
 177 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 178 |             double steer_angle = 0.0;
 179 |             if (i <= steer_delay_steps_) {
 180 |                 // To consider input delay
 181 |                 steer_angle = control_seq(0, CONTROL_SPACE::steer);
 182 |             } else {
 183 |                 steer_angle = control_seq(i, CONTROL_SPACE::steer);
 184 |             }
 185 | 
 186 |             // const double accel = control_seq(i, CONTROL_SPACE::accel);
 187 | 
 188 |             const double x = state_seq(i, STATE_SPACE::x);
 189 |             const double y = state_seq(i, STATE_SPACE::y);
 190 |             const double yaw = state_seq(i, STATE_SPACE::yaw);
 191 |             const double vel = state_seq(i, STATE_SPACE::vel);
 192 |             const double steer = state_seq(i, STATE_SPACE::steer);
 193 | 
 194 |             // Kinematic Bicycle Model
 195 |             const double beta = atan(lf_ / (lf_ + lr_) * tan(steer_angle));
 196 |             const double delta_x = vel * cos(yaw + beta) * prediction_interval_;
 197 |             const double delta_y = vel * sin(yaw + beta) * prediction_interval_;
 198 |             const double delta_yaw = vel * sin(beta) / lr_ * prediction_interval_;
 199 |             const double delta_steer = ((steer_angle - steer) / steer_delay_tau_) * prediction_interval_;
 200 |             double next_vel = 0.0;
 201 |             if (speed_prediction_mode_ == SpeedPredictionMode::CONSTANT) {
 202 |                 next_vel = constant_speed_prediction(vel);
 203 |             } else if (speed_prediction_mode_ == SpeedPredictionMode::LINEAR) {
 204 |                 next_vel = linear_speed_prediction(vel, reference_speed_, prediction_interval_, min_accel_, max_accel_);
 205 |             } else if (speed_prediction_mode_ == SpeedPredictionMode::REFERENCE) {
 206 |                 next_vel = reference_speed_prediction(x, y, reference_map);
 207 |             }
 208 | 
 209 |             state_seq(i + 1, STATE_SPACE::x) = x + delta_x;
 210 |             state_seq(i + 1, STATE_SPACE::y) = y + delta_y;
 211 |             state_seq(i + 1, STATE_SPACE::yaw) = std::atan2(sin(yaw + delta_yaw), cos(yaw + delta_yaw));
 212 |             state_seq(i + 1, STATE_SPACE::vel) = next_vel;
 213 |             state_seq(i + 1, STATE_SPACE::steer) = steer + delta_steer;
 214 |         }
 215 |         return state_seq;
 216 |     }
 217 | 
 218 |     // Predict both local and global state sequence from control sequence
 219 |     void MPCBase::predict_state_seq(const ControlSeq& control_seq,
 220 |                                     const State& global_init_state,
 221 |                                     const grid_map::GridMap& reference_map,
 222 |                                     StateSeq* global_state_seq,
 223 |                                     StateSeq* local_state_seq) const {
 224 |         // observed state
 225 |         global_state_seq->row(0) = global_init_state;
 226 | 
 227 |         // // observed state
 228 |         local_state_seq->row(0) = State::Zero(STATE_SPACE::dim);
 229 |         local_state_seq->row(0)(STATE_SPACE::vel) = global_init_state(STATE_SPACE::vel);
 230 | 
 231 |         // This is for linear prediction mode of speed
 232 |         const double init_x = global_init_state(STATE_SPACE::x);
 233 |         const double init_y = global_init_state(STATE_SPACE::y);
 234 |         double current_reference_speed = 0;
 235 |         if (reference_map.isInside(grid_map::Position(init_x, init_y))) {
 236 |             current_reference_speed = reference_map.atPosition(speed_field_layer_name_, grid_map::Position(init_x, init_y));
 237 |         }
 238 | 
 239 |         // double prev_steer_angle = control_seq(0, CONTROL_SPACE::steer);
 240 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 241 |             double steer_angle = 0.0;
 242 |             if (i <= steer_delay_steps_) {
 243 |                 // To consider input delay
 244 |                 steer_angle = control_seq(0, CONTROL_SPACE::steer);
 245 |             } else {
 246 |                 steer_angle = control_seq(i, CONTROL_SPACE::steer);
 247 |             }
 248 |             // const double steer_angle = control_seq(i, CONTROL_SPACE::steer);
 249 |             // const double accel = control_seq(i, CONTROL_SPACE::accel);
 250 | 
 251 |             // Update global state
 252 |             const double global_x = global_state_seq->row(i)(STATE_SPACE::x);
 253 |             const double global_y = global_state_seq->row(i)(STATE_SPACE::y);
 254 |             const double global_yaw = global_state_seq->row(i)(STATE_SPACE::yaw);
 255 |             const double global_vel = global_state_seq->row(i)(STATE_SPACE::vel);
 256 |             const double global_steer = global_state_seq->row(i)(STATE_SPACE::steer);
 257 | 
 258 |             const double beta_global = atan(lf_ / (lf_ + lr_) * tan(global_steer));
 259 | 
 260 |             // Kinematic Bicycle Model with steer 1st order delay
 261 |             const double delta_global_x = global_vel * cos(global_yaw + beta_global) * prediction_interval_;
 262 |             const double delta_global_y = global_vel * sin(global_yaw + beta_global) * prediction_interval_;
 263 |             const double delta_global_yaw = global_vel * sin(beta_global) / lr_ * prediction_interval_;
 264 |             const double delta_global_steer = ((steer_angle - global_steer) / steer_delay_tau_) * prediction_interval_;
 265 | 
 266 |             double next_vel = 0.0;
 267 |             if (speed_prediction_mode_ == SpeedPredictionMode::CONSTANT) {
 268 |                 next_vel = constant_speed_prediction(global_vel);
 269 |             } else if (speed_prediction_mode_ == SpeedPredictionMode::LINEAR) {
 270 |                 next_vel = linear_speed_prediction(global_vel, current_reference_speed, prediction_interval_, min_accel_, max_accel_);
 271 |             } else if (speed_prediction_mode_ == SpeedPredictionMode::REFERENCE) {
 272 |                 next_vel = reference_speed_prediction(global_x, global_y, reference_map);
 273 |             }
 274 | 
 275 |             global_state_seq->row(i + 1)(STATE_SPACE::x) = global_x + delta_global_x;
 276 |             global_state_seq->row(i + 1)(STATE_SPACE::y) = global_y + delta_global_y;
 277 |             global_state_seq->row(i + 1)(STATE_SPACE::yaw) = std::atan2(sin(global_yaw + delta_global_yaw), cos(global_yaw + delta_global_yaw));
 278 |             global_state_seq->row(i + 1)(STATE_SPACE::vel) = next_vel;
 279 |             global_state_seq->row(i + 1)(STATE_SPACE::steer) = global_steer + delta_global_steer;
 280 | 
 281 |             // Update local state
 282 |             const double local_x = local_state_seq->row(i)(STATE_SPACE::x);
 283 |             const double local_y = local_state_seq->row(i)(STATE_SPACE::y);
 284 |             const double local_yaw = local_state_seq->row(i)(STATE_SPACE::yaw);
 285 |             const double local_vel = local_state_seq->row(i)(STATE_SPACE::vel);
 286 |             const double local_steer = local_state_seq->row(i)(STATE_SPACE::steer);
 287 | 
 288 |             const double beta_local = atan(lf_ / (lf_ + lr_) * tan(local_steer));
 289 | 
 290 |             // Kinematic Bicycle Model with steer 1st order delay
 291 |             const double delta_local_x = local_vel * cos(local_yaw + beta_local) * prediction_interval_;
 292 |             const double delta_local_y = local_vel * sin(local_yaw + beta_local) * prediction_interval_;
 293 |             const double delta_local_yaw = local_vel * sin(beta_local) / lr_ * prediction_interval_;
 294 |             const double delta_local_steer = ((steer_angle - local_steer) / steer_delay_tau_) * prediction_interval_;
 295 | 
 296 |             local_state_seq->row(i + 1)(STATE_SPACE::x) = local_x + delta_local_x;
 297 |             local_state_seq->row(i + 1)(STATE_SPACE::y) = local_y + delta_local_y;
 298 |             local_state_seq->row(i + 1)(STATE_SPACE::yaw) = std::atan2(sin(local_yaw + delta_local_yaw), cos(local_yaw + delta_local_yaw));
 299 |             local_state_seq->row(i + 1)(STATE_SPACE::vel) = next_vel;
 300 |             local_state_seq->row(i + 1)(STATE_SPACE::steer) = local_steer + delta_local_steer;
 301 |         }
 302 |     }
 303 | 
 304 |     double MPCBase::constant_speed_prediction(const double& current_speed) const { return current_speed; }
 305 | 
 306 |     double MPCBase::linear_speed_prediction(const double& current_speed,
 307 |                                             const double& target_speed,
 308 |                                             const double& prediction_interval,
 309 |                                             const double& min_accel,
 310 |                                             const double& max_accel) const {
 311 |         if (current_speed < target_speed) {
 312 |             return std::min(current_speed + max_accel * prediction_interval, target_speed);
 313 |         } else if (current_speed > target_speed) {
 314 |             return std::max(current_speed - min_accel * prediction_interval, target_speed);
 315 |         } else {
 316 |             return current_speed;
 317 |         }
 318 |     }
 319 | 
 320 |     double MPCBase::reference_speed_prediction(const double& pos_x, const double& pos_y, const grid_map::GridMap& reference_map) const {
 321 |         if (reference_map.isInside(grid_map::Position(pos_x, pos_y))) {
 322 |             return reference_map.atPosition(speed_field_layer_name_, grid_map::Position(pos_x, pos_y));
 323 |         } else {
 324 |             return 0.0;
 325 |         }
 326 |     }
 327 | 
 328 |     // calculate state cost using only obstacle map from local state sequence
 329 |     std::pair<double, double> MPCBase::state_cost(const StateSeq& local_state_seq, const grid_map::GridMap& obstacle_map) const {
 330 |         // calc cost for each state
 331 |         double sum_collision_cost = 0.0;
 332 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 333 |             // state cost
 334 |             const State local_state = local_state_seq.row(i);
 335 | 
 336 |             // collision cost
 337 |             // check inside of gridmap
 338 |             double collision_cost = 10.0;
 339 |             if (obstacle_map.isInside(grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)))) {
 340 |                 collision_cost =
 341 |                     obstacle_map.atPosition(obstacle_layer_name_, grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)));
 342 |             }
 343 | 
 344 |             sum_collision_cost += collision_cost * collision_weight_;
 345 |         }
 346 | 
 347 |         // terminal cost
 348 |         const State terminal_local_state = local_state_seq.row(prediction_step_size_ - 1);
 349 |         double collision_cost = 10.0;
 350 |         if (obstacle_map.isInside(grid_map::Position(terminal_local_state(STATE_SPACE::x), terminal_local_state(STATE_SPACE::y)))) {
 351 |             collision_cost = obstacle_map.atPosition(obstacle_layer_name_,
 352 |                                                      grid_map::Position(terminal_local_state(STATE_SPACE::x), terminal_local_state(STATE_SPACE::y)));
 353 |         }
 354 | 
 355 |         sum_collision_cost += collision_cost * collision_weight_;
 356 | 
 357 |         return std::make_pair(sum_collision_cost, sum_collision_cost);
 358 |     }
 359 | 
 360 |     // calculate state cost using obstacle map and reference map from both global state sequence and local state sequence
 361 |     // obstacle cost is calculated from local state sequence
 362 |     // reference cost is calculated from global state sequence
 363 |     std::pair<double, double> MPCBase::state_cost(const StateSeq& global_state_seq,
 364 |                                                   const StateSeq& local_state_seq,
 365 |                                                   const grid_map::GridMap& local_obstacle_map,
 366 |                                                   const grid_map::GridMap& ref_path_map) const {
 367 |         // calc cost for each state
 368 |         double sum_ref_cost = 0.0;
 369 |         double sum_collision_cost = 0.0;
 370 |         const double max_diff_ref_path = ref_path_map.getLength().x();
 371 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 372 |             // state cost
 373 |             const State global_state = global_state_seq.row(i);
 374 |             const State local_state = local_state_seq.row(i);
 375 |             // const double diff_speed = reference_speed_ - state(STATE_SPACE::vel);
 376 |             double diff_ref_path = max_diff_ref_path;
 377 |             double diff_angle = 2.0;
 378 |             if (ref_path_map.isInside(grid_map::Position(global_state(STATE_SPACE::x), global_state(STATE_SPACE::y)))) {
 379 |                 diff_ref_path = ref_path_map.atPosition(distance_field_layer_name_,
 380 |                                                         grid_map::Position(global_state(STATE_SPACE::x), global_state(STATE_SPACE::y)));
 381 |                 diff_angle =
 382 |                     global_state(STATE_SPACE::yaw) -
 383 |                     ref_path_map.atPosition(angle_field_layer_name_, grid_map::Position(global_state(STATE_SPACE::x), global_state(STATE_SPACE::y)));
 384 |                 diff_angle = std::atan2(sin(diff_angle), cos(diff_angle));
 385 |             }
 386 |             // cost += q_speed_ * diff_speed * diff_speed;
 387 |             sum_ref_cost += q_dist_ * diff_ref_path * diff_ref_path;
 388 |             sum_ref_cost += q_angle_ * diff_angle * diff_angle;
 389 | 
 390 |             // collision cost
 391 |             // check inside of gridmap
 392 |             double collision_cost = 100.0;
 393 |             if (local_obstacle_map.isInside(grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)))) {
 394 |                 collision_cost =
 395 |                     local_obstacle_map.atPosition(obstacle_layer_name_, grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)));
 396 |             }
 397 | 
 398 |             sum_collision_cost += collision_cost * collision_weight_;
 399 |         }
 400 | 
 401 |         // terminal cost
 402 |         const State global_terminal_state = global_state_seq.row(prediction_step_size_ - 1);
 403 |         const State local_terminal_state = local_state_seq.row(prediction_step_size_ - 1);
 404 |         // const double diff_terminal_speed = reference_speed_ - terminal_state(STATE_SPACE::vel);
 405 |         double diff_terminal_ref_path = max_diff_ref_path;
 406 |         double diff_terminal_angle = 2.0;
 407 |         if (ref_path_map.isInside(grid_map::Position(global_terminal_state(STATE_SPACE::x), global_terminal_state(STATE_SPACE::y)))) {
 408 |             diff_terminal_ref_path = ref_path_map.atPosition(
 409 |                 distance_field_layer_name_, grid_map::Position(global_terminal_state(STATE_SPACE::x), global_terminal_state(STATE_SPACE::y)));
 410 |             diff_terminal_angle = global_terminal_state(STATE_SPACE::yaw) -
 411 |                                   ref_path_map.atPosition(angle_field_layer_name_, grid_map::Position(global_terminal_state(STATE_SPACE::x),
 412 |                                                                                                       global_terminal_state(STATE_SPACE::y)));
 413 |             diff_terminal_angle = std::atan2(sin(diff_terminal_angle), cos(diff_terminal_angle));
 414 |         }
 415 |         // cost += q_terminal_speed_ * diff_terminal_speed * diff_terminal_speed;
 416 |         sum_ref_cost += q_terminal_dist_ * diff_terminal_ref_path * diff_terminal_ref_path;
 417 |         sum_ref_cost += q_terminal_angle_ * diff_terminal_angle * diff_terminal_angle;
 418 | 
 419 |         double collision_cost = 10.0;
 420 |         if (local_obstacle_map.isInside(grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)))) {
 421 |             collision_cost = local_obstacle_map.atPosition(
 422 |                 obstacle_layer_name_, grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)));
 423 |         }
 424 | 
 425 |         sum_collision_cost += collision_cost * collision_weight_;
 426 | 
 427 |         return std::make_pair(sum_ref_cost + sum_collision_cost, sum_collision_cost);
 428 |     }
 429 | 
 430 |     std::pair<StateSeq, XYCovMatrices> MPCBase::calc_state_distribution(const StateSeqBatch& state_seq_candidates) const {
 431 |         // calc mean state
 432 |         StateSeq mean_state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
 433 |         for (size_t i = 0; i < state_seq_candidates.size(); i++) {
 434 |             mean_state_seq += state_seq_candidates[i];
 435 |         }
 436 |         mean_state_seq /= static_cast<double>(state_seq_candidates.size());
 437 | 
 438 |         // calc covariance matrices of x and y
 439 |         XYCovMatrices xy_cov_matrices =
 440 |             std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(prediction_step_size_, Eigen::MatrixXd::Zero(2, 2));
 441 | #pragma omp parallel for num_threads(thread_num_)
 442 |         for (size_t i = 0; i < state_seq_candidates.size(); i++) {
 443 |             for (size_t j = 0; j < prediction_step_size_; j++) {
 444 |                 Eigen::VectorXd diff = Eigen::VectorXd::Zero(2);
 445 |                 diff(0) = state_seq_candidates[i](j, STATE_SPACE::x) - mean_state_seq(j, STATE_SPACE::x);
 446 |                 diff(1) = state_seq_candidates[i](j, STATE_SPACE::y) - mean_state_seq(j, STATE_SPACE::y);
 447 |                 xy_cov_matrices[j] += diff * diff.transpose();
 448 |             }
 449 |         }
 450 |         for (size_t j = 0; j < prediction_step_size_; j++) {
 451 |             xy_cov_matrices[j] /= static_cast<double>(state_seq_candidates.size());
 452 |         }
 453 | 
 454 |         return std::make_pair(mean_state_seq, xy_cov_matrices);
 455 |     }
 456 | 
 457 | }  // namespace cpu
 458 | }  // namespace mppi

```

`src\mppi_controller\src\mppi_controller_node.cpp`:

```cpp
   1 | #include "mppi_controller/mppi_controller_ros.hpp"
   2 | 
   3 | int main(int argc, char** argv) {
   4 |     ros::init(argc, argv, "mppi_controller");
   5 |     mppi::MPPIControllerROS mppi_controller;
   6 |     ros::spin();
   7 |     return 0;
   8 | };

```

`src\mppi_controller\src\mppi_controller_ros.cpp`:

```cpp
   1 | #include "mppi_controller/mppi_controller_ros.hpp"
   2 | 
   3 | namespace mppi {
   4 | MPPIControllerROS::MPPIControllerROS() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_) {
   5 |     // set parameters from ros parameter server
   6 |     private_nh_.param("is_simulation", is_simulation_, false);
   7 |     private_nh_.param("is_localize_less_mode", is_localize_less_mode_, false);
   8 | 
   9 |     private_nh_.param("control_sampling_time", control_sampling_time_, 0.1);
  10 | 
  11 |     private_nh_.param("is_visualize_mppi", is_visualize_mppi_, false);
  12 |     private_nh_.param("constant_speed_mode", constant_speed_mode_, false);
  13 | 
  14 |     std::string control_cmd_topic;
  15 |     std::string in_reference_sdf_topic;
  16 |     std::string in_odom_topic;
  17 |     std::string is_activate_ad_topic;
  18 |     std::string occupacy_grid_id;
  19 |     std::string local_costmap_id;
  20 |     std::string in_backward_point_topic;
  21 | 
  22 |     private_nh_.param("control_cmd_topic", control_cmd_topic, static_cast<std::string>("drive"));
  23 |     private_nh_.param("in_reference_sdf_topic", in_reference_sdf_topic, static_cast<std::string>("reference_sdf"));
  24 |     private_nh_.param("in_odom_topic", in_odom_topic, static_cast<std::string>("odom"));
  25 |     private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
  26 |     private_nh_.param("map_frame_id", map_frame_id_, static_cast<std::string>("map"));
  27 |     private_nh_.param("costmap_id", occupacy_grid_id, static_cast<std::string>("costmap"));
  28 |     private_nh_.param("local_costmap_id", local_costmap_id, static_cast<std::string>("local_costmap"));
  29 |     private_nh_.param("backward_point_topic", in_backward_point_topic, static_cast<std::string>("backward_point"));
  30 | 
  31 |     private_nh_.param("is_activate_ad_topic", is_activate_ad_topic, static_cast<std::string>("is_active_ad"));
  32 |     private_nh_.param("collision_rate_threshold", collision_rate_threshold_, 0.95);
  33 |     private_nh_.param("speed_queue_size", speed_deque_size_, static_cast<int>(10));
  34 |     private_nh_.param("stuck_speed_threshold", stuck_speed_threshold_, static_cast<float>(0.1));
  35 | 
  36 |     private_nh_.param("steer_1st_delay", steer_1st_delay_, static_cast<double>(0.1));
  37 | 
  38 |     // fill deque
  39 |     for (int i = 0; i < speed_deque_size_; i++) {
  40 |         const double enough_speed = stuck_speed_threshold_ + 1.0;
  41 |         speed_deque_.push_back(enough_speed);
  42 |     }
  43 | 
  44 |     // load params
  45 |     Params params;
  46 |     // Common params for all MPC methods
  47 |     params.common.is_localize_less_mode = is_localize_less_mode_;
  48 |     params.common.steer_1st_delay = steer_1st_delay_;
  49 | 
  50 |     private_nh_.param("common/thread_num", params.common.thread_num, 12);
  51 |     private_nh_.param("reference_speed", params.common.reference_speed, 5.0);
  52 |     reference_speed_ = params.common.reference_speed;
  53 |     private_nh_.param("common/prediction_step_size", params.common.prediction_step_size, 10);
  54 |     private_nh_.param("common/prediction_interval", params.common.prediction_interval, 0.1);
  55 | 
  56 |     steer_1st_delay_ = std::max(steer_1st_delay_, params.common.prediction_interval);
  57 | 
  58 |     private_nh_.param("common/steer_delay", params.common.steer_delay, 0.1);
  59 |     private_nh_.param("common/max_steer_angle", params.common.max_steer_angle, 3.0);
  60 |     private_nh_.param("common/min_steer_angle", params.common.min_steer_angle, -3.0);
  61 |     private_nh_.param("common/max_accel", params.common.max_accel, 2.0);
  62 |     private_nh_.param("common/min_accel", params.common.min_accel, -2.0);
  63 |     private_nh_.param("common/speed_prediction_mode", params.common.speed_prediction_mode, static_cast<std::string>("linear"));
  64 |     private_nh_.param("common/lr", params.common.lr, 0.17145);
  65 |     private_nh_.param("common/lf", params.common.lf, 0.15875);
  66 |     private_nh_.param("common/q_dist", params.common.q_dist, 1.0);
  67 |     private_nh_.param("common/q_angle", params.common.q_angle, 0.0);
  68 |     // private_nh_.param("common/q_speed", params.q_speed, 10.0);
  69 |     private_nh_.param("common/collision_weight", params.common.collision_weight, 0.01);
  70 |     private_nh_.param("common/q_terminal_dist", params.common.q_terminal_dist, 1.0);
  71 |     private_nh_.param("common/q_terminal_angle", params.common.q_terminal_angle, 0.0);
  72 |     // private_nh_.param("common/q_terminal_speed",
  73 |     // params.q_terminal_speed, 10.0);
  74 | 
  75 |     // Forward MPPI params
  76 |     private_nh_.param("forward_mppi/sample_batch_num", params.forward_mppi.sample_batch_num, 1000);
  77 |     private_nh_.param("forward_mppi/lambda", params.forward_mppi.lambda, 10.0);
  78 |     private_nh_.param("forward_mppi/alpha", params.forward_mppi.alpha, 0.1);
  79 |     private_nh_.param("forward_mppi/non_biased_sampling_rate", params.forward_mppi.non_biased_sampling_rate, 0.1);
  80 |     private_nh_.param("forward_mppi/steer_cov", params.forward_mppi.steer_cov, 0.5);
  81 |     // private_nh_.param("forward_mppi/accel_cov", params.accel_cov, 0.5);
  82 |     private_nh_.param("forward_mppi/num_itr_for_grad_estimation", params.forward_mppi.num_itr_for_grad_estimation, 1);
  83 |     private_nh_.param("forward_mppi/step_size_for_grad_estimation", params.forward_mppi.step_size_for_grad_estimation, 0.1);
  84 |     private_nh_.param("forward_mppi/sample_num_for_grad_estimation", params.forward_mppi.sample_num_for_grad_estimation, 10);
  85 |     private_nh_.param("forward_mppi/steer_cov_for_grad_estimation", params.forward_mppi.steer_cov_for_grad_estimation, 0.05);
  86 | 
  87 |     // Reverse MPPI params
  88 |     private_nh_.param("reverse_mppi/sample_batch_num", params.reverse_mppi.sample_batch_num, 1000);
  89 |     private_nh_.param("reverse_mppi/negative_ratio", params.reverse_mppi.negative_ratio, 1.0);
  90 |     private_nh_.param("reverse_mppi/is_sample_rejection", params.reverse_mppi.is_sample_rejection, true);
  91 |     private_nh_.param("reverse_mppi/sample_inflation_ratio", params.reverse_mppi.sample_inflation_ratio, 2.0);
  92 |     private_nh_.param("reverse_mppi/iteration_num", params.reverse_mppi.iteration_num, 5);
  93 |     private_nh_.param("reverse_mppi/step_size", params.reverse_mppi.step_size, 0.5);
  94 |     private_nh_.param("reverse_mppi/warm_start_ratio", params.reverse_mppi.warm_start_ratio, 0.5);
  95 |     private_nh_.param("reverse_mppi/lambda", params.reverse_mppi.lambda, 10.0);
  96 |     private_nh_.param("reverse_mppi/alpha", params.reverse_mppi.alpha, 0.1);
  97 |     private_nh_.param("reverse_mppi/non_biased_sampling_rate", params.reverse_mppi.non_biased_sampling_rate, 0.1);
  98 |     private_nh_.param("reverse_mppi/steer_cov", params.reverse_mppi.steer_cov, 0.5);
  99 |     // private_nh_.param("reverse_mppi/accel_cov", params.accel_cov, 0.5);
 100 | 
 101 |     // Stein variational MPC params
 102 |     private_nh_.param("stein_variational_mpc/sample_batch_num", params.stein_variational_mpc.sample_batch_num, 1000);
 103 |     private_nh_.param("stein_variational_mpc/lambda", params.stein_variational_mpc.lambda, 10.0);
 104 |     private_nh_.param("stein_variational_mpc/alpha", params.stein_variational_mpc.alpha, 0.1);
 105 |     private_nh_.param("stein_variational_mpc/non_biased_sampling_rate", params.stein_variational_mpc.non_biased_sampling_rate, 0.1);
 106 |     private_nh_.param("stein_variational_mpc/steer_cov", params.stein_variational_mpc.steer_cov, 0.5);
 107 |     // private_nh_.param("stein_variational_mpc/accel_cov", params.accel_cov,
 108 |     // 0.5);
 109 |     private_nh_.param("stein_variational_mpc/num_svgd_iteration", params.stein_variational_mpc.num_svgd_iteration, 100);
 110 |     private_nh_.param("stein_variational_mpc/sample_num_for_grad_estimation", params.stein_variational_mpc.sample_num_for_grad_estimation, 10);
 111 |     private_nh_.param("stein_variational_mpc/steer_cov_for_grad_estimation", params.stein_variational_mpc.steer_cov_for_grad_estimation, 0.05);
 112 |     private_nh_.param("stein_variational_mpc/svgd_step_size", params.stein_variational_mpc.svgd_step_size, 0.1);
 113 |     private_nh_.param("stein_variational_mpc/is_max_posterior_estimation", params.stein_variational_mpc.is_max_posterior_estimation, false);
 114 | 
 115 |     // Stein Variational Guided MPPI params
 116 |     private_nh_.param("svg_mppi/sample_batch_num", params.svg_mppi.sample_batch_num, 1000);
 117 |     private_nh_.param("svg_mppi/lambda", params.svg_mppi.lambda, 10.0);
 118 |     private_nh_.param("svg_mppi/alpha", params.svg_mppi.alpha, 0.1);
 119 |     private_nh_.param("svg_mppi/non_biased_sampling_rate", params.svg_mppi.non_biased_sampling_rate, 0.1);
 120 |     private_nh_.param("svg_mppi/steer_cov", params.svg_mppi.steer_cov, 0.5);
 121 |     // private_nh_.param("svg_mppi/accel_cov", params.accel_cov, 0.5);
 122 |     private_nh_.param("svg_mppi/guide_sample_num", params.svg_mppi.guide_sample_num, 1);
 123 |     private_nh_.param("svg_mppi/grad_lambda", params.svg_mppi.grad_lambda, 3.0);
 124 |     private_nh_.param("svg_mppi/sample_num_for_grad_estimation", params.svg_mppi.sample_num_for_grad_estimation, 10);
 125 |     private_nh_.param("svg_mppi/steer_cov_for_grad_estimation", params.svg_mppi.steer_cov_for_grad_estimation, 0.05);
 126 |     private_nh_.param("svg_mppi/svgd_step_size", params.svg_mppi.svgd_step_size, 0.1);
 127 |     private_nh_.param("svg_mppi/num_svgd_iteration", params.svg_mppi.num_svgd_iteration, 100);
 128 |     private_nh_.param("svg_mppi/is_use_nominal_solution", params.svg_mppi.is_use_nominal_solution, true);
 129 |     private_nh_.param("svg_mppi/is_covariance_adaptation", params.svg_mppi.is_covariance_adaptation, true);
 130 |     private_nh_.param("svg_mppi/gaussian_fitting_lambda", params.svg_mppi.gaussian_fitting_lambda, 0.1);
 131 |     private_nh_.param("svg_mppi/min_steer_cov", params.svg_mppi.min_steer_cov, 0.001);
 132 |     private_nh_.param("svg_mppi/max_steer_cov", params.svg_mppi.max_steer_cov, 0.1);
 133 | 
 134 |     const std::string mpc_mode = private_nh_.param<std::string>("mpc_mode", "");
 135 |     if (mpc_mode == "forward_mppi") {
 136 |         mpc_solver_ptr_ = std::make_unique<mppi::cpu::ForwardMPPI>(params.common, params.forward_mppi);
 137 |     } else if (mpc_mode == "reverse_mppi") {
 138 |         mpc_solver_ptr_ = std::make_unique<mppi::cpu::ReverseMPPI>(params.common, params.reverse_mppi);
 139 |     } else if (mpc_mode == "sv_mpc") {
 140 |         mpc_solver_ptr_ = std::make_unique<mppi::cpu::SteinVariationalMPC>(params.common, params.stein_variational_mpc);
 141 |     } else if (mpc_mode == "svg_mppi") {
 142 |         mpc_solver_ptr_ = std::make_unique<mppi::cpu::SVGuidedMPPI>(params.common, params.svg_mppi);
 143 |     } else {
 144 |         ROS_ERROR("Invalid MPC mode: %s", mpc_mode.c_str());
 145 |         exit(1);
 146 |     }
 147 | 
 148 |     // set publishers and subscribers
 149 |     pub_ackermann_cmd_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(control_cmd_topic, 1);
 150 |     timer_control_ = nh_.createTimer(ros::Duration(control_sampling_time_), &MPPIControllerROS::timer_callback, this);
 151 |     sub_activated_ = nh_.subscribe(is_activate_ad_topic, 1, &MPPIControllerROS::callback_activate_signal, this);
 152 |     if (is_localize_less_mode_) {
 153 |         sub_odom_ = nh_.subscribe(in_odom_topic, 1, &MPPIControllerROS::callback_odom,
 154 |                                   this);  // subscribe only odometry
 155 |         sub_grid_map_ = nh_.subscribe(local_costmap_id, 1, &MPPIControllerROS::callback_grid_map, this);
 156 |     } else {
 157 |         sub_odom_ = nh_.subscribe(in_odom_topic, 1, &MPPIControllerROS::callback_odom_with_pose,
 158 |                                   this);  // subscribe odometry and localized pose
 159 |         sub_reference_sdf_ = nh_.subscribe(in_reference_sdf_topic, 1, &MPPIControllerROS::callback_reference_sdf, this);
 160 | 
 161 |         sub_grid_map_ = nh_.subscribe(local_costmap_id, 1, &MPPIControllerROS::callback_grid_map, this);
 162 |     }
 163 | 
 164 |     sub_start_cmd_ = nh_.subscribe("mppi/start", 1, &MPPIControllerROS::start_cmd_callback, this);
 165 |     sub_stop_cmd_ = nh_.subscribe("mppi/stop", 1, &MPPIControllerROS::stop_cmd_callback, this);
 166 | 
 167 |     // For debug
 168 |     pub_best_path_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/best_path", 1, true);
 169 |     pub_nominal_path_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/nominal_path", 1, true);
 170 |     pub_candidate_paths_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/candidate_paths", 1, true);
 171 |     pub_proposal_state_distributions_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/proposal_state_distributions", 1, true);
 172 |     pub_control_covariances_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/control_covariances", 1, true);
 173 |     pub_calculation_time_ = nh_.advertise<std_msgs::Float32>("mppi/calculation_time", 1, true);
 174 |     pub_speed_ = nh_.advertise<std_msgs::Float32>("mppi/speed", 1, true);
 175 |     pub_collision_rate_ = nh_.advertise<std_msgs::Float32>("mppi/collision_rate", 1, true);
 176 |     pub_cost_ = nh_.advertise<std_msgs::Float32>("mppi/cost", 1, true);
 177 |     pub_mppi_metrics_ = nh_.advertise<mppi_metrics_msgs::MPPIMetrics>("mppi/eval_metrics", 1, true);
 178 | }
 179 | 
 180 | // Get current pose and velocity used with localization model
 181 | void MPPIControllerROS::callback_odom_with_pose(const nav_msgs::Odometry& odom) {
 182 |     /*Get current pose via tf*/
 183 |     geometry_msgs::TransformStamped trans_form_stamped;
 184 |     try {
 185 |         trans_form_stamped = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
 186 |     } catch (const tf2::TransformException& ex) {
 187 |         ROS_WARN_THROTTLE(3.0, "[MPPIController]: %s", ex.what());
 188 |         return;
 189 |     };
 190 | 
 191 |     /*Update status*/
 192 |     robot_state_.x = trans_form_stamped.transform.translation.x;
 193 |     robot_state_.y = trans_form_stamped.transform.translation.y;
 194 |     const double _yaw = tf2::getYaw(trans_form_stamped.transform.rotation);
 195 |     robot_state_.yaw = std::atan2(std::sin(_yaw), std::cos(_yaw));
 196 |     robot_state_.vel = odom.twist.twist.linear.x;
 197 | 
 198 |     is_robot_state_ok_ = true;
 199 | }
 200 | 
 201 | // Get only current velocity used with localization less model
 202 | void MPPIControllerROS::callback_odom(const nav_msgs::Odometry& odom) {
 203 |     /*Update status*/
 204 |     robot_state_.x = 0.0;
 205 |     robot_state_.y = 0.0;
 206 |     robot_state_.yaw = 0.0;
 207 |     robot_state_.vel = odom.twist.twist.linear.x;
 208 | 
 209 |     is_robot_state_ok_ = true;
 210 | }
 211 | 
 212 | void MPPIControllerROS::callback_activate_signal(const std_msgs::Bool& is_activate) {
 213 |     is_activate_ad_ = static_cast<bool>(is_activate.data);
 214 | }
 215 | 
 216 | void MPPIControllerROS::callback_grid_map(const grid_map_msgs::GridMap& grid_map) {
 217 |     // make grid map for obstacle layer
 218 |     if (!grid_map::GridMapRosConverter::fromMessage(grid_map, obstacle_map_)) {
 219 |         ROS_ERROR("[MPPIControllerROS]Failed to convert grid map to grid map");
 220 |         return;
 221 |     }
 222 | 
 223 |     is_costmap_ok_ = true;
 224 | }
 225 | 
 226 | void MPPIControllerROS::callback_reference_sdf(const grid_map_msgs::GridMap& grid_map) {
 227 |     // make grid map for reference path layer
 228 |     if (!grid_map::GridMapRosConverter::fromMessage(grid_map, reference_sdf_)) {
 229 |         ROS_ERROR("[MPPIControllerROS]Failed to convert reference sdf to grid map");
 230 |         return;
 231 |     }
 232 |     is_reference_sdf_ok_ = true;
 233 | }
 234 | 
 235 | void MPPIControllerROS::start_cmd_callback([[maybe_unused]] const std_msgs::Empty& msg) {
 236 |     is_start_ = true;
 237 |     ROS_INFO("[MPD] start cmd received");
 238 | }
 239 | 
 240 | void MPPIControllerROS::stop_cmd_callback([[maybe_unused]] const std_msgs::Empty& msg) {
 241 |     is_start_ = false;
 242 |     ROS_INFO("[MPD] stop cmd received");
 243 | }
 244 | 
 245 | // Control loop
 246 | void MPPIControllerROS::timer_callback([[maybe_unused]] const ros::TimerEvent& te) {
 247 |     /* Status check */
 248 |     if (is_localize_less_mode_) {
 249 |         if (!is_robot_state_ok_ || !is_costmap_ok_) {
 250 |             ROS_WARN_THROTTLE(5.0, "[MPPIControllerROS] Not ready: Robot state: %d, Map: %d", is_robot_state_ok_, is_costmap_ok_);
 251 |             return;
 252 |         }
 253 |     } else {
 254 |         if (!is_robot_state_ok_ || !is_reference_sdf_ok_ || !is_costmap_ok_) {
 255 |             ROS_WARN_THROTTLE(5.0,
 256 |                               "[MPPIControllerROS] Not ready: Robot state: %d, "
 257 |                               "Reference SDF: %d, Map: %d",
 258 |                               is_robot_state_ok_, is_reference_sdf_ok_, is_costmap_ok_);
 259 |             return;
 260 |         }
 261 |     }
 262 | 
 263 |     if (!is_activate_ad_ && !is_simulation_) {
 264 |         // publish zero control command and return
 265 |         ROS_WARN_THROTTLE(5.0, "[MPPIControllerROS] waiting to activate signal");
 266 |         control_msg_.header.stamp = ros::Time::now();
 267 |         control_msg_.drive.steering_angle = 0.0;
 268 |         control_msg_.drive.speed = 0.0;
 269 |         pub_ackermann_cmd_.publish(control_msg_);
 270 | 
 271 |         return;
 272 |     }
 273 | 
 274 |     if (!is_start_) {
 275 |         // publish zero control command and
 276 |         ROS_WARN_THROTTLE(5.0, "[MPPIControllerROS] waiting to start signal from rviz");
 277 |         control_msg_.header.stamp = ros::Time::now();
 278 |         control_msg_.drive.steering_angle = 0.0;
 279 |         control_msg_.drive.speed = 0.0;
 280 |         pub_ackermann_cmd_.publish(control_msg_);
 281 | 
 282 |         // publish predicted state for debug
 283 |         const int num_visualized_samples = 100;
 284 |         const auto [predict_state_seq_batch, weights] = mpc_solver_ptr_->get_state_seq_candidates(num_visualized_samples);
 285 |         publish_candidate_paths(predict_state_seq_batch, weights, pub_candidate_paths_);
 286 | 
 287 |         return;
 288 |     }
 289 | 
 290 |     mtx_.lock();
 291 | 
 292 |     stop_watch_.lap();
 293 | 
 294 |     mpc_solver_ptr_->set_obstacle_map(obstacle_map_);
 295 |     if (!is_localize_less_mode_) {
 296 |         mpc_solver_ptr_->set_reference_map(reference_sdf_);
 297 |     }
 298 | 
 299 |     // steer observer
 300 |     const double prev_steer_cmd = control_msg_.drive.steering_angle;
 301 |     robot_state_.steer = robot_state_.steer + (prev_steer_cmd - robot_state_.steer) * (1 - exp(-control_sampling_time_ / steer_1st_delay_));
 302 | 
 303 |     // Solve MPC
 304 |     mppi::cpu::State initial_state = mppi::cpu::State::Zero();
 305 |     initial_state[STATE_SPACE::x] = robot_state_.x;
 306 |     initial_state[STATE_SPACE::y] = robot_state_.y;
 307 |     initial_state[STATE_SPACE::yaw] = robot_state_.yaw;
 308 |     initial_state[STATE_SPACE::vel] = robot_state_.vel;
 309 |     initial_state[STATE_SPACE::steer] = robot_state_.steer;
 310 |     const auto [updated_control_seq, collision_rate] = mpc_solver_ptr_->solve(initial_state);
 311 | 
 312 |     mtx_.unlock();
 313 | 
 314 |     // predict state seq
 315 |     const auto [best_state_seq, state_cost, collision_cost, input_error] = mpc_solver_ptr_->get_predictive_seq(initial_state, updated_control_seq);
 316 | 
 317 |     // Publish control command
 318 |     const double steering_angle = updated_control_seq(0, CONTROL_SPACE::steer);
 319 |     // const double accel = updated_control_seq(0, CONTROL_SPACE::accel);
 320 | 
 321 |     // const double speed = robot_state_.vel + accel * control_sampling_time_;
 322 |     // speed_cmd_ += accel * control_sampling_time_;
 323 |     control_msg_.header.stamp = ros::Time::now();
 324 |     control_msg_.drive.steering_angle = std::atan2(std::sin(steering_angle), std::cos(steering_angle));
 325 |     double speed_cmd = 0.0;
 326 |     if (constant_speed_mode_ || is_localize_less_mode_) {
 327 |         // Note: constant speed is used in localize_less_mode because reference sdf
 328 |         // is not available
 329 |         speed_cmd = reference_speed_;
 330 |     } else {
 331 |         if (reference_sdf_.isInside(grid_map::Position(robot_state_.x, robot_state_.y))) {
 332 |             // speed_cmd = reference_sdf_.atPosition(speed_field_layer_name_,
 333 |             // grid_map::Position(ahead_x, ahead_y));
 334 |             speed_cmd = reference_sdf_.atPosition(speed_field_layer_name_, grid_map::Position(robot_state_.x, robot_state_.y));
 335 |         } else {
 336 |             speed_cmd = reference_speed_;
 337 |             ROS_WARN(
 338 |                 "[MPPIControllerROS] Robot is out of reference sdf map. Use "
 339 |                 "constant speed mode.");
 340 |         }
 341 |     }
 342 |     control_msg_.drive.speed = speed_cmd;
 343 | 
 344 |     pub_ackermann_cmd_.publish(control_msg_);
 345 | 
 346 |     const double calculation_time = stop_watch_.lap();
 347 | 
 348 |     // ==========  For debug ===============
 349 |     if (is_visualize_mppi_) {
 350 |         // publish predicted state
 351 |         const int num_visualized_samples = 100;
 352 |         const auto [predict_state_seq_batch, weights] = mpc_solver_ptr_->get_state_seq_candidates(num_visualized_samples);
 353 |         publish_candidate_paths(predict_state_seq_batch, weights, pub_candidate_paths_);
 354 | 
 355 |         // Get covariance of proposed distribution
 356 |         const auto cov_matrices = mpc_solver_ptr_->get_cov_matrices();
 357 | 
 358 |         // publish best state
 359 |         publish_traj(best_state_seq, "best_path", "red", pub_best_path_);
 360 | 
 361 |         // publish control covariance
 362 |         publish_control_covs(best_state_seq, cov_matrices, pub_control_covariances_);
 363 | 
 364 |         // publish proposed state distribution
 365 |         const auto [mean, xycov_mat] = mpc_solver_ptr_->get_proposed_state_distribution();
 366 |         publish_state_seq_dists(mean, xycov_mat, pub_proposal_state_distributions_);
 367 | 
 368 |         // publish nominal state
 369 |         const auto nominal_control_seq = mpc_solver_ptr_->get_control_seq();
 370 |         const auto nominal_state_seq = std::get<0>(mpc_solver_ptr_->get_predictive_seq(initial_state, nominal_control_seq));
 371 |         publish_path(nominal_state_seq, "nominal_path", "g", pub_nominal_path_);
 372 |     } else {
 373 |         // publish best state
 374 |         publish_traj(best_state_seq, "best_path", "red", pub_best_path_);
 375 |     }
 376 | 
 377 |     // publish cost of the best state seq
 378 |     std_msgs::Float32 cost_msg;
 379 |     cost_msg.data = static_cast<float>(state_cost + collision_cost);
 380 |     pub_cost_.publish(cost_msg);
 381 | 
 382 |     // publish speed
 383 |     std_msgs::Float32 speed_msg;
 384 |     speed_msg.data = robot_state_.vel;
 385 |     pub_speed_.publish(speed_msg);
 386 | 
 387 |     // publish calculate time
 388 |     std_msgs::Float32 calculation_time_msg;
 389 |     calculation_time_msg.data = static_cast<float>(calculation_time);
 390 |     pub_calculation_time_.publish(calculation_time_msg);
 391 | 
 392 |     // publish collision rate
 393 |     std_msgs::Float32 collision_rate_msg;
 394 |     collision_rate_msg.data = static_cast<float>(collision_rate);
 395 |     pub_collision_rate_.publish(collision_rate_msg);
 396 | 
 397 |     // publish mppi metrics
 398 |     mppi_metrics_msgs::MPPIMetrics mppi_metrics_msg;
 399 |     mppi_metrics_msg.header.stamp = ros::Time::now();
 400 |     mppi_metrics_msg.calculation_time = calculation_time;
 401 |     mppi_metrics_msg.state_cost = state_cost;
 402 |     mppi_metrics_msg.collision_cost = collision_cost;
 403 |     mppi_metrics_msg.input_error = input_error;
 404 |     pub_mppi_metrics_.publish(mppi_metrics_msg);
 405 | }
 406 | 
 407 | void MPPIControllerROS::publish_traj(const mppi::cpu::StateSeq& state_seq,
 408 |                                      const std::string& name_space,
 409 |                                      const std::string& rgb,
 410 |                                      const ros::Publisher& publisher) const {
 411 |     visualization_msgs::MarkerArray marker_array;
 412 | 
 413 |     visualization_msgs::Marker arrow;
 414 |     if (is_localize_less_mode_) {
 415 |         arrow.header.frame_id = robot_frame_id_;
 416 |     } else {
 417 |         arrow.header.frame_id = map_frame_id_;
 418 |     }
 419 |     arrow.header.stamp = ros::Time::now();
 420 |     arrow.ns = name_space;
 421 |     arrow.type = visualization_msgs::Marker::ARROW;
 422 |     arrow.action = visualization_msgs::Marker::ADD;
 423 |     geometry_msgs::Vector3 arrow_scale;
 424 |     arrow_scale.x = 0.02;
 425 |     arrow_scale.y = 0.04;
 426 |     arrow_scale.z = 0.1;
 427 |     arrow.scale = arrow_scale;
 428 |     arrow.pose.position.x = 0.0;
 429 |     arrow.pose.position.y = 0.0;
 430 |     arrow.pose.position.z = 0.0;
 431 |     arrow.pose.orientation.x = 0.0;
 432 |     arrow.pose.orientation.y = 0.0;
 433 |     arrow.pose.orientation.z = 0.0;
 434 |     arrow.pose.orientation.w = 1.0;
 435 |     arrow.color.a = 1.0;
 436 |     arrow.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
 437 |     arrow.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
 438 |     arrow.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);
 439 | 
 440 |     // arrow.lifetime = ros::Duration(0.1);
 441 |     arrow.points.resize(2);
 442 | 
 443 |     for (int i = 0; i < state_seq.rows(); i++) {
 444 |         arrow.id = i;
 445 |         const auto state = state_seq.row(i);
 446 |         const double length = abs(state[STATE_SPACE::vel]) * 0.1;
 447 |         geometry_msgs::Point start;
 448 |         start.x = state[STATE_SPACE::x];
 449 |         start.y = state[STATE_SPACE::y];
 450 |         start.z = 0.1;
 451 | 
 452 |         geometry_msgs::Point end;
 453 |         end.x = state[STATE_SPACE::x] + length * cos(state[STATE_SPACE::yaw]);
 454 |         end.y = state[STATE_SPACE::y] + length * sin(state[STATE_SPACE::yaw]);
 455 |         end.z = 0.1;
 456 | 
 457 |         arrow.points[0] = start;
 458 |         arrow.points[1] = end;
 459 | 
 460 |         marker_array.markers.push_back(arrow);
 461 |     }
 462 |     publisher.publish(marker_array);
 463 | }
 464 | 
 465 | void MPPIControllerROS::publish_path(const mppi::cpu::StateSeq& state_seq,
 466 |                                      const std::string& name_space,
 467 |                                      const std::string& rgb,
 468 |                                      const ros::Publisher& publisher) const {
 469 |     visualization_msgs::MarkerArray marker_array;
 470 | 
 471 |     visualization_msgs::Marker line;
 472 |     if (is_localize_less_mode_) {
 473 |         line.header.frame_id = robot_frame_id_;
 474 |     } else {
 475 |         line.header.frame_id = map_frame_id_;
 476 |     }
 477 |     line.header.stamp = ros::Time::now();
 478 |     line.ns = name_space + "_line";
 479 |     line.id = 0;
 480 |     line.type = visualization_msgs::Marker::LINE_STRIP;
 481 |     line.action = visualization_msgs::Marker::ADD;
 482 |     line.pose.orientation.x = 0.0;
 483 |     line.pose.orientation.y = 0.0;
 484 |     line.pose.orientation.z = 0.0;
 485 |     line.pose.orientation.w = 1.0;
 486 |     line.scale.x = 0.01;
 487 |     line.color.a = 1.0;
 488 |     line.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
 489 |     line.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
 490 |     line.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);
 491 |     // line.lifetime = ros::Duration(0.1);
 492 | 
 493 |     // nodes
 494 |     visualization_msgs::Marker nodes;
 495 |     if (is_localize_less_mode_) {
 496 |         nodes.header.frame_id = robot_frame_id_;
 497 |     } else {
 498 |         nodes.header.frame_id = map_frame_id_;
 499 |     }
 500 |     nodes.header.stamp = ros::Time::now();
 501 |     nodes.ns = name_space + "_nodes";
 502 |     nodes.id = 1;
 503 |     nodes.type = visualization_msgs::Marker::SPHERE_LIST;
 504 |     nodes.action = visualization_msgs::Marker::ADD;
 505 |     nodes.pose.orientation.x = 0.0;
 506 |     nodes.pose.orientation.y = 0.0;
 507 |     nodes.pose.orientation.z = 0.0;
 508 |     nodes.pose.orientation.w = 1.0;
 509 |     const double node_size = 0.1;
 510 |     nodes.scale.x = node_size;
 511 |     nodes.scale.y = node_size;
 512 |     nodes.scale.z = node_size;
 513 |     nodes.color.a = 1.0;
 514 |     nodes.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
 515 |     nodes.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
 516 |     nodes.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);
 517 |     // nodes.lifetime = ros::Duration(0.1);
 518 | 
 519 |     for (int j = 0; j < state_seq.rows(); j++) {
 520 |         geometry_msgs::Point point;
 521 |         point.x = state_seq(j, STATE_SPACE::x);
 522 |         point.y = state_seq(j, STATE_SPACE::y);
 523 |         point.z = 0.1;
 524 |         line.points.push_back(point);
 525 |         nodes.points.push_back(point);
 526 |     }
 527 |     marker_array.markers.push_back(line);
 528 |     marker_array.markers.push_back(nodes);
 529 | 
 530 |     publisher.publish(marker_array);
 531 | }
 532 | 
 533 | void MPPIControllerROS::publish_candidate_paths(const std::vector<mppi::cpu::StateSeq>& state_seq_batch,
 534 |                                                 const std::vector<double>& weights,
 535 |                                                 const ros::Publisher& publisher) const {
 536 |     assert(state_seq_batch.size() == weights.size());
 537 | 
 538 |     visualization_msgs::MarkerArray marker_array;
 539 | 
 540 |     const double max_weight = weights[0];
 541 |     const double max_node_size = 0.05;
 542 |     for (size_t i = 0; i < state_seq_batch.size(); i++) {
 543 |         visualization_msgs::Marker line;
 544 |         if (is_localize_less_mode_) {
 545 |             line.header.frame_id = robot_frame_id_;
 546 |         } else {
 547 |             line.header.frame_id = map_frame_id_;
 548 |         }
 549 |         line.header.stamp = ros::Time::now();
 550 |         line.ns = "candidate_path_line";
 551 |         line.id = i;
 552 |         line.type = visualization_msgs::Marker::LINE_STRIP;
 553 |         line.action = visualization_msgs::Marker::ADD;
 554 |         line.pose.orientation.x = 0.0;
 555 |         line.pose.orientation.y = 0.0;
 556 |         line.pose.orientation.z = 0.0;
 557 |         line.pose.orientation.w = 1.0;
 558 |         line.scale.x = 0.01;
 559 |         line.color.a = 0.3;
 560 |         line.color.r = 0.0;
 561 |         line.color.g = 0.5;
 562 |         line.color.b = 1.0;
 563 |         // line.lifetime = ros::Duration(0.1);
 564 | 
 565 |         // nodes
 566 |         visualization_msgs::Marker nodes;
 567 |         if (is_localize_less_mode_) {
 568 |             nodes.header.frame_id = robot_frame_id_;
 569 |         } else {
 570 |             nodes.header.frame_id = map_frame_id_;
 571 |         }
 572 |         nodes.header.stamp = ros::Time::now();
 573 |         nodes.ns = "candidate_path_nodes";
 574 |         nodes.id = line.id + 10000 + i;
 575 |         nodes.type = visualization_msgs::Marker::SPHERE_LIST;
 576 |         nodes.action = visualization_msgs::Marker::ADD;
 577 |         nodes.pose.orientation.x = 0.0;
 578 |         nodes.pose.orientation.y = 0.0;
 579 |         nodes.pose.orientation.z = 0.0;
 580 |         nodes.pose.orientation.w = 1.0;
 581 |         nodes.scale.x = weights[i] * max_node_size / max_weight + 0.01;
 582 |         nodes.scale.y = 0.01;
 583 |         nodes.scale.z = 0.01;
 584 |         nodes.color.a = 0.6;
 585 |         nodes.color.r = 0.0;
 586 |         nodes.color.g = 0.5;
 587 |         nodes.color.b = 1.0;
 588 |         // nodes.lifetime = ros::Duration(0.1);
 589 | 
 590 |         for (int j = 0; j < state_seq_batch.at(0).rows(); j++) {
 591 |             geometry_msgs::Point point;
 592 |             point.x = state_seq_batch.at(i)(j, STATE_SPACE::x);
 593 |             point.y = state_seq_batch.at(i)(j, STATE_SPACE::y);
 594 |             point.z = 0.0;
 595 |             line.points.push_back(point);
 596 |             nodes.points.push_back(point);
 597 |         }
 598 |         marker_array.markers.push_back(line);
 599 |         marker_array.markers.push_back(nodes);
 600 |     }
 601 |     publisher.publish(marker_array);
 602 | }
 603 | 
 604 | void MPPIControllerROS::publish_control_covs(const mppi::cpu::StateSeq& state_seq,
 605 |                                              const mppi::cpu::ControlSeqCovMatrices& cov_matrices,
 606 |                                              const ros::Publisher& publisher) const {
 607 |     visualization_msgs::MarkerArray marker_array;
 608 |     visualization_msgs::Marker ellipse;
 609 |     if (is_localize_less_mode_) {
 610 |         ellipse.header.frame_id = robot_frame_id_;
 611 |     } else {
 612 |         ellipse.header.frame_id = map_frame_id_;
 613 |     }
 614 |     ellipse.header.stamp = ros::Time::now();
 615 |     ellipse.ns = "steer_covariance";
 616 |     ellipse.type = visualization_msgs::Marker::SPHERE;
 617 |     ellipse.action = visualization_msgs::Marker::ADD;
 618 |     geometry_msgs::Vector3 ellipse_scale;
 619 |     ellipse_scale.x = 0.0;
 620 |     ellipse_scale.y = 0.0;
 621 |     ellipse_scale.z = 0.0;
 622 |     ellipse.scale = ellipse_scale;
 623 |     ellipse.pose.position.x = 0.0;
 624 |     ellipse.pose.position.y = 0.0;
 625 |     ellipse.pose.position.z = 0.0;
 626 |     ellipse.pose.orientation.x = 0.0;
 627 |     ellipse.pose.orientation.y = 0.0;
 628 |     ellipse.pose.orientation.z = 0.0;
 629 |     ellipse.pose.orientation.w = 1.0;
 630 |     ellipse.color.a = 0.4;
 631 |     // yellow
 632 |     ellipse.color.r = 1.0;
 633 |     ellipse.color.g = 1.0;
 634 |     ellipse.color.b = 0.0;
 635 |     // ellipse.lifetime = ros::Duration(1.0);
 636 | 
 637 |     const double control_cov_scale = 1.5;
 638 | 
 639 |     for (int i = 0; i < state_seq.rows() - 1; i++) {
 640 |         // ellipse of proposal distribution
 641 |         ellipse.id = i;
 642 |         const auto cov_matrix = cov_matrices[i];
 643 |         // Only publish steer
 644 |         const double cov = cov_matrix(CONTROL_SPACE::steer, CONTROL_SPACE::steer);
 645 |         const double std_dev = sqrt(cov) * control_cov_scale;
 646 |         ellipse_scale.x = 2.0 * std_dev + 0.01;
 647 |         ellipse_scale.y = 2.0 * std_dev + 0.01;
 648 |         ellipse_scale.z = 0.01;
 649 |         ellipse.scale = ellipse_scale;
 650 |         ellipse.pose.position.x = state_seq(i, STATE_SPACE::x);
 651 |         ellipse.pose.position.y = state_seq(i, STATE_SPACE::y);
 652 | 
 653 |         marker_array.markers.push_back(ellipse);
 654 |     }
 655 |     publisher.publish(marker_array);
 656 | }
 657 | 
 658 | void MPPIControllerROS::publish_state_seq_dists(const mppi::cpu::StateSeq& state_seq,
 659 |                                                 const mppi::cpu::XYCovMatrices& cov_matrices,
 660 |                                                 const ros::Publisher& publisher) const {
 661 |     visualization_msgs::MarkerArray marker_array;
 662 |     visualization_msgs::Marker ellipse;
 663 |     if (is_localize_less_mode_) {
 664 |         ellipse.header.frame_id = robot_frame_id_;
 665 |     } else {
 666 |         ellipse.header.frame_id = map_frame_id_;
 667 |     }
 668 |     ellipse.header.stamp = ros::Time::now();
 669 |     ellipse.ns = "proposal_state_distributions";
 670 |     ellipse.type = visualization_msgs::Marker::SPHERE;
 671 |     ellipse.action = visualization_msgs::Marker::ADD;
 672 |     geometry_msgs::Vector3 ellipse_scale;
 673 |     ellipse_scale.x = 0.0;
 674 |     ellipse_scale.y = 0.0;
 675 |     ellipse_scale.z = 0.0;
 676 |     ellipse.scale = ellipse_scale;
 677 |     ellipse.pose.position.x = 0.0;
 678 |     ellipse.pose.position.y = 0.0;
 679 |     ellipse.pose.position.z = 0.0;
 680 |     ellipse.pose.orientation.x = 0.0;
 681 |     ellipse.pose.orientation.y = 0.0;
 682 |     ellipse.pose.orientation.z = 0.0;
 683 |     ellipse.pose.orientation.w = 1.0;
 684 |     ellipse.color.a = 0.4;
 685 |     // yellow
 686 |     ellipse.color.r = 0.0;
 687 |     ellipse.color.g = 0.0;
 688 |     ellipse.color.b = 1.0;
 689 |     // ellipse.lifetime = ros::Duration(0.1);
 690 | 
 691 |     for (int i = 0; i < state_seq.rows(); i++) {
 692 |         ellipse.id = i;
 693 |         ellipse.pose.position.x = state_seq(i, STATE_SPACE::x);
 694 |         ellipse.pose.position.y = state_seq(i, STATE_SPACE::y);
 695 | 
 696 |         // get length and angle of major and minor axis
 697 |         Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov_matrices.at(i).block<2, 2>(0, 0));
 698 |         const auto eigen_value = eigensolver.eigenvalues();
 699 |         const auto eigen_vector = eigensolver.eigenvectors();
 700 |         const double major_axis_length = std::sqrt(eigen_value(0));
 701 |         const double minor_axis_length = std::sqrt(eigen_value(1));
 702 |         const double yaw = std::atan2(eigen_vector(1, 0), eigen_vector(0, 0));
 703 | 
 704 |         ellipse.scale.x = 2.0 * major_axis_length + 0.1;
 705 |         ellipse.scale.y = 2.0 * minor_axis_length + 0.1;
 706 |         ellipse.scale.z = 0.1;
 707 |         // yaw to quaternion
 708 |         tf2::Quaternion q;
 709 |         q.setRPY(0.0, 0.0, yaw);
 710 |         ellipse.pose.orientation.x = q.x();
 711 |         ellipse.pose.orientation.y = q.y();
 712 |         ellipse.pose.orientation.z = q.z();
 713 |         ellipse.pose.orientation.w = q.w();
 714 | 
 715 |         marker_array.markers.push_back(ellipse);
 716 |     }
 717 |     publisher.publish(marker_array);
 718 | }
 719 | 
 720 | }  // namespace mppi

```

`src\mppi_controller\src\prior_samples_with_costs.cpp`:

```cpp
   1 | #include "mppi_controller/prior_samples_with_costs.hpp"
   2 | 
   3 | namespace mppi {
   4 | namespace cpu {
   5 |     PriorSamplesWithCosts::PriorSamplesWithCosts(const size_t& num_samples,
   6 |                                                  const size_t& prediction_horizon,
   7 |                                                  const std::array<double, CONTROL_SPACE::dim>& max_control_inputs,
   8 |                                                  const std::array<double, CONTROL_SPACE::dim>& min_control_inputs,
   9 |                                                  const double& non_biased_sampling_rate,
  10 |                                                  const int& thread_num,
  11 |                                                  const int& seed)
  12 |         : thread_num_(thread_num),
  13 |           num_samples_(num_samples),
  14 |           prediction_horizon_(prediction_horizon),
  15 |           non_biased_sampling_rate_(non_biased_sampling_rate),
  16 |           max_control_inputs_(max_control_inputs),
  17 |           min_control_inputs_(min_control_inputs) {
  18 |         control_seq_mean_ = Eigen::MatrixXd::Zero(prediction_horizon - 1, CONTROL_SPACE::dim);
  19 |         control_seq_cov_matrices_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  20 |             prediction_horizon - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
  21 |         control_seq_inv_cov_matrices_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  22 |             prediction_horizon - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
  23 |         noised_control_seq_samples_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  24 |             num_samples, Eigen::MatrixXd::Zero(prediction_horizon - 1, CONTROL_SPACE::dim));
  25 |         noise_seq_samples_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
  26 |             num_samples, Eigen::MatrixXd::Zero(prediction_horizon - 1, CONTROL_SPACE::dim));
  27 |         costs_ = std::vector<double>(num_samples, 0.0);
  28 | 
  29 |         // Initialize random number generator
  30 |         for (int i = 0; i < thread_num_; i++) {
  31 |             rngs_.push_back(std::mt19937(seed + i));
  32 |         }
  33 | 
  34 |         // Initialize normal distributions
  35 |         normal_dists_ptr_ = std::make_unique<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>>();
  36 |         for (size_t i = 0; i < prediction_horizon - 1; i++) {
  37 |             std::array<std::normal_distribution<>, CONTROL_SPACE::dim> normal_dists = {};
  38 |             for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
  39 |                 std::normal_distribution<> normal_dist(0.0, 1.0);
  40 |                 normal_dists[j] = normal_dist;
  41 |             }
  42 |             (*normal_dists_ptr_).push_back(normal_dists);
  43 |         }
  44 |     }
  45 | 
  46 |     void PriorSamplesWithCosts::random_sampling(const ControlSeq& control_seq_mean, const ControlSeqCovMatrices& control_seq_cov_matrices) {
  47 |         set_control_seq_mean(control_seq_mean);
  48 |         set_control_seq_cov_matrices(control_seq_cov_matrices);
  49 | 
  50 |         // set normal distributions parameters
  51 |         for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
  52 |             for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
  53 |                 const double std_dev = std::sqrt(control_seq_cov_matrices_[i](j, j));
  54 |                 std::normal_distribution<>::param_type param(0.0, std_dev);
  55 |                 (*normal_dists_ptr_)[i][j].param(param);
  56 |             }
  57 |         }
  58 | 
  59 | #pragma omp parallel for num_threads(thread_num_)
  60 |         for (size_t i = 0; i < num_samples_; i++) {
  61 |             // generate noise sequence
  62 |             for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
  63 |                 for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
  64 |                     noise_seq_samples_[i](j, k) = (*normal_dists_ptr_)[j][k](rngs_[omp_get_thread_num()]);
  65 |                 }
  66 |             }
  67 | 
  68 |             // sampling control sequences with non-biased (around zero) sampling rate
  69 |             if (i < static_cast<size_t>((1 - non_biased_sampling_rate_) * num_samples_)) {
  70 |                 // biased sampling (around control_seq_mean)
  71 |                 noised_control_seq_samples_[i] = control_seq_mean + noise_seq_samples_[i];
  72 |             } else {
  73 |                 // non-biased sampling (around zero)
  74 |                 noised_control_seq_samples_[i] = noise_seq_samples_[i];
  75 |             }
  76 | 
  77 |             // clip input with control input constraints
  78 |             for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
  79 |                 for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
  80 |                     noised_control_seq_samples_[i](k, j) =
  81 |                         std::clamp(noised_control_seq_samples_[i](k, j), min_control_inputs_[j], max_control_inputs_[j]);
  82 |                 }
  83 |             }
  84 |         }
  85 |     }
  86 | 
  87 |     std::vector<int> PriorSamplesWithCosts::random_sample_choice(const size_t& num_samples, const std::vector<double>& probabilities) {
  88 |         if (num_samples >= num_samples_) {
  89 |             std::cout << "Error: num_samples is larger than num_samples_." << std::endl;
  90 |             exit(1);
  91 |         }
  92 | 
  93 |         if (probabilities.size() != num_samples_) {
  94 |             std::cout << "Error: probability size is not equal to num_samples_." << std::endl;
  95 |             exit(1);
  96 |         }
  97 | 
  98 |         // choose random indices with probability
  99 |         discrete_dist_.param(std::discrete_distribution<>::param_type(probabilities.begin(), probabilities.end()));
 100 |         std::vector<int> indices(num_samples);
 101 | #pragma omp parallel for num_threads(thread_num_)
 102 |         for (size_t i = 0; i < num_samples; i++) {
 103 |             indices[i] = discrete_dist_(rngs_[omp_get_thread_num()]);
 104 |         }
 105 | 
 106 |         return indices;
 107 |     }
 108 | 
 109 |     std::vector<double> PriorSamplesWithCosts::get_costs_with_control_term(const double& lambda,
 110 |                                                                            const double& alpha,
 111 |                                                                            const ControlSeq& nominal_control_seq) const {
 112 |         std::vector<double> costs_with_control_term = costs_;
 113 | #pragma omp parallel for num_threads(thread_num_)
 114 |         for (size_t i = 0; i < num_samples_; i++) {
 115 |             for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
 116 |                 const double control_term = lambda * (1 - alpha) * (control_seq_mean_.row(j) - nominal_control_seq.row(j)) *
 117 |                                             control_seq_inv_cov_matrices_[j] * noised_control_seq_samples_[i].row(j).transpose();
 118 |                 costs_with_control_term[i] += control_term;
 119 |             }
 120 |         }
 121 |         return costs_with_control_term;
 122 |     }
 123 | 
 124 |     // copy part of samples that are selected from source samples
 125 |     void PriorSamplesWithCosts::shrink_copy_from(const PriorSamplesWithCosts& source_samples, const std::vector<int>& indices) {
 126 |         // check: source samples size is larger than indices size because shrinked samples are stored in this class
 127 |         if (indices.size() != num_samples_) {
 128 |             std::cout << "Error: indices size is not equal to num_samples_." << std::endl;
 129 |             exit(1);
 130 |         }
 131 | 
 132 |         if (source_samples.prediction_horizon_ != prediction_horizon_) {
 133 |             std::cout << "Error: source_samples.prediction_horizon_ is not equal to prediction_horizon_." << std::endl;
 134 |             exit(1);
 135 |         }
 136 | 
 137 |         if (source_samples.num_samples_ < indices.size()) {
 138 |             std::cout << "Error: source_samples.num_samples_ is smaller than indices.size()." << std::endl;
 139 |             exit(1);
 140 |         }
 141 | 
 142 | #pragma omp parallel for num_threads(thread_num_)
 143 |         for (size_t i = 0; i < indices.size(); i++) {
 144 |             noised_control_seq_samples_[i] = source_samples.noised_control_seq_samples_[indices[i]];
 145 |             noise_seq_samples_[i] = source_samples.noise_seq_samples_[indices[i]];
 146 |             costs_[i] = source_samples.costs_[indices[i]];
 147 |         }
 148 |         set_control_seq_mean(source_samples.control_seq_mean_);
 149 |         set_control_seq_cov_matrices(source_samples.control_seq_cov_matrices_);
 150 |     }
 151 | 
 152 |     // copy from source samples and inflate samples until num_samples_ is satisfied with probabilities
 153 |     // covもいるね
 154 |     void PriorSamplesWithCosts::inflate_copy_from(const PriorSamplesWithCosts& source_samples, const ControlSeqCovMatrices& noise_covs) {
 155 |         // check
 156 |         if (source_samples.prediction_horizon_ != prediction_horizon_) {
 157 |             std::cout << "Error: source_samples.prediction_horizon_ is not equal to prediction_horizon_." << std::endl;
 158 |             exit(1);
 159 |         }
 160 | 
 161 |         if (source_samples.num_samples_ > num_samples_) {
 162 |             std::cout << "Error: source_samples.num_samples_ should is smaller than num_samples_." << std::endl;
 163 |             exit(1);
 164 |         }
 165 | 
 166 |         // copy samples from source samples
 167 | #pragma omp parallel for num_threads(thread_num_)
 168 |         for (size_t i = 0; i < source_samples.num_samples_; i++) {
 169 |             noised_control_seq_samples_[i] = source_samples.noised_control_seq_samples_[i];
 170 |             noise_seq_samples_[i] = source_samples.noise_seq_samples_[i];
 171 |             costs_[i] = source_samples.costs_[i];
 172 |         }
 173 | 
 174 |         // inflate samples
 175 |         const size_t num_inflated_samples = num_samples_ - source_samples.num_samples_;
 176 |         std::vector<double> probabilities(source_samples.num_samples_, 1.0 / source_samples.num_samples_);
 177 |         discrete_dist_.param(std::discrete_distribution<>::param_type(probabilities.begin(), probabilities.end()));
 178 |         std::vector<int> indices(num_inflated_samples, 0);
 179 | #pragma omp parallel for num_threads(thread_num_)
 180 |         for (size_t i = 0; i < num_inflated_samples; i++) {
 181 |             indices[i] = discrete_dist_(rngs_[omp_get_thread_num()]);
 182 |         }
 183 | 
 184 |         // add sample with noise
 185 |         for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
 186 |             for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
 187 |                 const double std_dev = std::sqrt(noise_covs[i](j, j));
 188 |                 std::normal_distribution<>::param_type param(0.0, std_dev);
 189 |                 (*normal_dists_ptr_)[i][j].param(param);
 190 |             }
 191 |         }
 192 | #pragma omp parallel for num_threads(thread_num_)
 193 |         for (size_t i = 0; i < num_inflated_samples; i++) {
 194 |             for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
 195 |                 for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
 196 |                     noise_seq_samples_[source_samples.num_samples_ + i](j, k) = (*normal_dists_ptr_)[j][k](rngs_[omp_get_thread_num()]);
 197 |                 }
 198 |             }
 199 | 
 200 |             noised_control_seq_samples_[source_samples.num_samples_ + i] =
 201 |                 source_samples.noised_control_seq_samples_[indices[i]] + noise_seq_samples_[source_samples.num_samples_ + i];
 202 | 
 203 |             // clip input with control input constraints
 204 |             for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
 205 |                 for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
 206 |                     noised_control_seq_samples_[source_samples.num_samples_ + i](k, j) = std::clamp(
 207 |                         noised_control_seq_samples_[source_samples.num_samples_ + i](k, j), min_control_inputs_[j], max_control_inputs_[j]);
 208 |                 }
 209 |             }
 210 |         }
 211 |     }
 212 | 
 213 |     void PriorSamplesWithCosts::set_control_seq_mean(const ControlSeq& control_seq_mean) { control_seq_mean_ = control_seq_mean; }
 214 | 
 215 |     void PriorSamplesWithCosts::set_control_seq_cov_matrices(const ControlSeqCovMatrices& control_seq_cov_matrices) {
 216 |         // To prevent singular matrix, add small value to diagonal elements
 217 |         const double eps = 1e-4;
 218 |         control_seq_cov_matrices_ = control_seq_cov_matrices;
 219 | 
 220 |         // calculate inverse of covariance matrices in advance to reduce computational cost
 221 |         for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
 222 |             control_seq_inv_cov_matrices_[i] =
 223 |                 control_seq_cov_matrices_[i].inverse() + eps * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
 224 |         }
 225 |     }
 226 | 
 227 | }  // namespace cpu
 228 | 
 229 | }  // namespace mppi

```

`src\mppi_controller\src\reverse_mppi.cpp`:

```cpp
   1 | #include "mppi_controller/reverse_mppi.hpp"
   2 | 
   3 | namespace mppi {
   4 | namespace cpu {
   5 |     ReverseMPPI::ReverseMPPI(const Params::Common& common_params, const Params::ReverseMPPI& reverse_mppi_params)
   6 |         : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
   7 |           thread_num_(common_params.thread_num),
   8 |           negative_ratio_(reverse_mppi_params.negative_ratio),
   9 |           is_sample_rejection_(reverse_mppi_params.is_sample_rejection),
  10 |           sample_inflation_ratio_(reverse_mppi_params.sample_inflation_ratio),
  11 |           iteration_num_(reverse_mppi_params.iteration_num),
  12 |           step_size_(reverse_mppi_params.step_size),
  13 |           warm_start_ratio_(reverse_mppi_params.warm_start_ratio),
  14 |           lambda_(reverse_mppi_params.lambda),
  15 |           alpha_(reverse_mppi_params.alpha),
  16 |           non_biased_sampling_rate_(reverse_mppi_params.non_biased_sampling_rate),
  17 |           steer_cov_(reverse_mppi_params.steer_cov) {
  18 |         const size_t sample_batch_num = static_cast<size_t>(reverse_mppi_params.sample_batch_num);
  19 |         mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_batch_num);
  20 | 
  21 |         const double max_steer_angle = common_params.max_steer_angle;
  22 |         const double min_steer_angle = common_params.min_steer_angle;
  23 |         std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
  24 |         std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};
  25 | 
  26 |         prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
  27 |                                                                      non_biased_sampling_rate_, thread_num_);
  28 | 
  29 |         const size_t inflated_sample_batch_num = static_cast<size_t>(std::round(sample_batch_num * sample_inflation_ratio_));
  30 |         inflated_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(inflated_sample_batch_num, prediction_step_size_, max_control_inputs,
  31 |                                                                         min_control_inputs, non_biased_sampling_rate_, thread_num_);
  32 | 
  33 |         prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
  34 |         const std::array<double, CONTROL_SPACE::dim> control_cov_diag = {steer_cov_};
  35 |         prev_covs_ = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);
  36 | 
  37 |         prev_rejected_mean_ = prior_samples_ptr_->get_zero_control_seq();
  38 |         prev_rejected_covs_ = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);
  39 |     }
  40 | 
  41 |     std::pair<ControlSeq, double> ReverseMPPI::solve(const State& initial_state) {
  42 |         int collision_num = 0;
  43 | 
  44 |         ControlSeq control_seq_mean = prev_control_seq_;
  45 |         ControlSeqCovMatrices control_seq_cov_matrices = prev_covs_;
  46 |         ControlSeq rejected_mean = prev_rejected_mean_;
  47 |         ControlSeqCovMatrices rejected_covs = prev_rejected_covs_;
  48 |         for (int i = 0; i < iteration_num_; i++) {
  49 |             if (is_sample_rejection_) {
  50 |                 // generate inflated (sample_batch_num x sample_inflation_ratio) samples, not sample rejection yet
  51 |                 inflated_samples_ptr_->random_sampling(control_seq_mean, control_seq_cov_matrices);
  52 | 
  53 |                 // calculate sample rejection probabilities
  54 |                 ControlSeq normal_prob = normal_pdf(rejected_mean, control_seq_mean, control_seq_cov_matrices);
  55 |                 std::vector<double> probabilities(inflated_samples_ptr_->get_num_samples(), 0.0);
  56 | #pragma omp parallel for num_threads(thread_num_)
  57 |                 for (size_t j = 0; j < inflated_samples_ptr_->get_num_samples(); j++) {
  58 |                     const ControlSeq pr = normal_pdf(inflated_samples_ptr_->noised_control_seq_samples_[j], rejected_mean, rejected_covs);
  59 |                     probabilities[j] = (normal_prob + pr).cwiseInverse().prod();
  60 |                 }
  61 | 
  62 |                 // normalize probabilities
  63 |                 const double sum_prob = std::min(std::accumulate(probabilities.begin(), probabilities.end(), 0.0), 1e-10);
  64 | #pragma omp parallel for num_threads(thread_num_)
  65 |                 for (size_t j = 0; j < inflated_samples_ptr_->get_num_samples(); j++) {
  66 |                     probabilities[j] /= sum_prob;
  67 |                 }
  68 | 
  69 |                 const std::vector<int> selected_indices =
  70 |                     inflated_samples_ptr_->random_sample_choice(prior_samples_ptr_->get_num_samples(), probabilities);
  71 |                 prior_samples_ptr_->shrink_copy_from(*inflated_samples_ptr_, selected_indices);
  72 |             } else {
  73 |                 prior_samples_ptr_->random_sampling(control_seq_mean, control_seq_cov_matrices);
  74 |             }
  75 | 
  76 |             // Predict and calculate trajectory costs
  77 |             auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
  78 |             prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
  79 |             collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });
  80 | 
  81 |             // calculate weights for each state sequence based on costs and prior samples
  82 |             const auto [weights_eval, weights_reject] = calc_weights(*prior_samples_ptr_);
  83 |             weights_ = weights_eval;  // for visualization
  84 | 
  85 |             // calculate weighted mean and standard deviation of evaluated and rejected samples
  86 |             const auto [mean_eval, sigma_eval] = weighted_mean_and_sigma(*prior_samples_ptr_, weights_eval);
  87 |             const auto [mean_reject, sigma_reject] = weighted_mean_and_sigma(*prior_samples_ptr_, weights_reject);
  88 | 
  89 |             // MD update
  90 |             const auto [new_mean, new_covs] = md_update(control_seq_mean, control_seq_cov_matrices, mean_eval, sigma_eval, step_size_);
  91 |             control_seq_mean = new_mean;
  92 |             control_seq_cov_matrices = new_covs;
  93 | 
  94 |             const auto [new_rejected_mean, new_rejected_covs] = md_update(rejected_mean, rejected_covs, mean_reject, sigma_reject, step_size_);
  95 |             rejected_mean = new_rejected_mean;
  96 |             rejected_covs = new_rejected_covs;
  97 |         }
  98 | 
  99 |         // Warm start for next control iteration
 100 |         prev_control_seq_ = interpolate(prev_control_seq_, control_seq_mean, warm_start_ratio_);
 101 |         prev_covs_ = interpolate(prev_covs_, control_seq_cov_matrices, warm_start_ratio_);
 102 |         prev_rejected_mean_ = interpolate(prev_rejected_mean_, rejected_mean, warm_start_ratio_);
 103 |         prev_rejected_covs_ = interpolate(prev_rejected_covs_, rejected_covs, warm_start_ratio_);
 104 | 
 105 |         const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());
 106 | 
 107 |         return std::make_pair(control_seq_mean, collision_rate);
 108 |     }
 109 | 
 110 |     void ReverseMPPI::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };
 111 | 
 112 |     void ReverseMPPI::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };
 113 | 
 114 |     std::pair<std::vector<StateSeq>, std::vector<double>> ReverseMPPI::get_state_seq_candidates(const int& num_samples) const {
 115 |         return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
 116 |     }
 117 | 
 118 |     std::tuple<StateSeq, double, double, double> ReverseMPPI::get_predictive_seq(const State& initial_state,
 119 |                                                                                  const ControlSeq& control_input_seq) const {
 120 |         const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
 121 |         double input_error = 0.0;
 122 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 123 |             input_error += (control_input_seq.row(i)).norm();
 124 |         }
 125 |         return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
 126 |     }
 127 | 
 128 |     ControlSeqCovMatrices ReverseMPPI::get_cov_matrices() const {
 129 |         const auto [_mean, cov] = estimate_mu_and_sigma(*prior_samples_ptr_);
 130 |         return cov;
 131 |     }
 132 | 
 133 |     ControlSeq ReverseMPPI::get_control_seq() const { return prev_control_seq_; }
 134 | 
 135 |     std::pair<StateSeq, XYCovMatrices> ReverseMPPI::get_proposed_state_distribution() const { return mpc_base_ptr_->get_proposed_distribution(); }
 136 | 
 137 |     // == private functions ==
 138 |     std::pair<std::vector<double>, std::vector<double>> ReverseMPPI::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const {
 139 |         // calculate costs with control cost term
 140 |         const std::vector<double> costs_with_control_term =
 141 |             prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, prior_samples_ptr_->get_zero_control_seq());
 142 | 
 143 |         // calculate normalization term
 144 |         double positive_normalization_term = 1e-10;
 145 |         double negative_normalization_term = 1e-10;
 146 |         const double min_cost = *std::min_element(costs_with_control_term.begin(), costs_with_control_term.end());
 147 |         for (size_t i = 0; i < prior_samples_with_costs.get_num_samples(); i++) {
 148 |             positive_normalization_term += std::exp(-1.0 / lambda_ * (costs_with_control_term[i] - min_cost));
 149 |             negative_normalization_term += std::exp(1.0 / lambda_ * (costs_with_control_term[i] - min_cost));
 150 |         }
 151 | 
 152 |         std::vector<double> weights_eval(prior_samples_with_costs.get_num_samples(), 0.0);
 153 |         std::vector<double> weights_reject(prior_samples_with_costs.get_num_samples(), 0.0);
 154 | // calculate weights for importance sampling
 155 | #pragma omp parallel for num_threads(thread_num_)
 156 |         for (size_t i = 0; i < prior_samples_with_costs.get_num_samples(); i++) {
 157 |             const double positive_term = std::exp(-1.0 / lambda_ * (costs_with_control_term[i] - min_cost)) / positive_normalization_term;
 158 |             const double negative_term =
 159 |                 negative_ratio_ * std::exp(1.0 / lambda_ * (costs_with_control_term[i] - min_cost)) / negative_normalization_term;
 160 | 
 161 |             const double reversed_weight = positive_term - negative_term;
 162 |             weights_eval[i] = std::max(reversed_weight, 0.0);
 163 |             weights_reject[i] = -std::min(reversed_weight, 0.0);
 164 |         }
 165 | 
 166 |         // normalize weights
 167 |         const double sum_weights_eval = std::accumulate(weights_eval.begin(), weights_eval.end(), 0.0);
 168 |         const double sum_weights_reject = std::accumulate(weights_reject.begin(), weights_reject.end(), 0.0);
 169 | #pragma omp parallel for num_threads(thread_num_)
 170 |         for (size_t i = 0; i < prior_samples_with_costs.get_num_samples(); i++) {
 171 |             weights_eval[i] /= sum_weights_eval;
 172 |             weights_reject[i] /= sum_weights_reject;
 173 |         }
 174 | 
 175 |         return std::make_pair(weights_eval, weights_reject);
 176 |     }
 177 | 
 178 |     std::pair<ControlSeq, ControlSeqCovMatrices> ReverseMPPI::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
 179 |         ControlSeq mu = samples.get_zero_control_seq();
 180 |         ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();
 181 | 
 182 | // calculate mean
 183 | #pragma omp parallel for num_threads(thread_num_)
 184 | 
 185 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 186 |             mu += samples.noised_control_seq_samples_[i];
 187 |         }
 188 |         mu /= static_cast<double>(samples.get_num_samples());
 189 | 
 190 |         // mu = samples.control_seq_mean_;
 191 | 
 192 | // calculate covariance matrices
 193 | #pragma omp parallel for num_threads(thread_num_)
 194 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 195 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 196 |                 sigma[j] += (samples.noised_control_seq_samples_[i].row(j) - mu.row(j)).transpose() *
 197 |                             (samples.noised_control_seq_samples_[i].row(j) - mu.row(j));
 198 |             }
 199 |         }
 200 | 
 201 |         for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 202 |             sigma[j] /= static_cast<double>(samples.get_num_samples());
 203 | 
 204 |             // add small value to avoid singular matrix
 205 |             sigma[j] += 1e-5 * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
 206 |         }
 207 | 
 208 |         return std::make_pair(mu, sigma);
 209 |     }
 210 | 
 211 |     std::pair<ControlSeq, ControlSeqCovMatrices> ReverseMPPI::weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
 212 |                                                                                       const std::vector<double>& weights) const {
 213 |         ControlSeq mean = samples.get_zero_control_seq();
 214 |         ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();
 215 | 
 216 |         const ControlSeq prior_mean = samples.get_mean();
 217 |         const ControlSeqCovMatrices prior_inv_covs = samples.get_inv_cov_matrices();
 218 | #pragma omp parallel for num_threads(thread_num_)
 219 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 220 |             mean += weights[i] * samples.noised_control_seq_samples_[i];
 221 | 
 222 |             const ControlSeq diff = samples.noised_control_seq_samples_[i] - prior_mean;
 223 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 224 |                 sigma[j] += weights[i] * diff.row(j).transpose() * prior_inv_covs[j] * diff.row(j);
 225 |             }
 226 |         }
 227 | 
 228 |         return std::make_pair(mean, sigma);
 229 |     }
 230 | 
 231 |     std::pair<ControlSeq, ControlSeqCovMatrices> ReverseMPPI::md_update(const ControlSeq& prior_mean,
 232 |                                                                         const ControlSeqCovMatrices& prior_covs,
 233 |                                                                         const ControlSeq& weighted_mean,
 234 |                                                                         const ControlSeqCovMatrices& weighted_covs,
 235 |                                                                         const double step_size) const {
 236 |         ControlSeq updated_mean = prior_mean * 0.0;
 237 |         updated_mean = prior_mean - step_size * (prior_mean - weighted_mean);
 238 | 
 239 |         ControlSeqCovMatrices prior_stds = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
 240 |             prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
 241 |         ControlSeqCovMatrices weighted_stds = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
 242 |             prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
 243 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 244 |             prior_stds[i] = prior_covs[i].diagonal().cwiseSqrt().asDiagonal();
 245 |             weighted_stds[i] = weighted_covs[i].diagonal().cwiseSqrt().asDiagonal();
 246 |         }
 247 | 
 248 |         ControlSeqCovMatrices tmp = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
 249 |             prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
 250 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 251 |             tmp[i] = -step_size * (prior_stds[i] - weighted_stds[i]);
 252 |         }
 253 |         ControlSeqCovMatrices updated_covs = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
 254 |             prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
 255 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 256 |             const Eigen::MatrixXd updated_std =
 257 |                 0.5 * (tmp[i] + (tmp[i].transpose() * tmp[i] + 4.0 * prior_stds[i].transpose() * prior_stds[i]).cwiseSqrt());
 258 |             updated_covs[i] = updated_std * updated_std;
 259 |         }
 260 | 
 261 |         return std::make_pair(updated_mean, updated_covs);
 262 |     }
 263 | 
 264 |     ControlSeq ReverseMPPI::normal_pdf(const ControlSeq& x, const ControlSeq& mean, const ControlSeqCovMatrices& var) const {
 265 |         ControlSeq pdf = x * 0.0;
 266 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 267 |             pdf.row(i) = (x.row(i) - mean.row(i))
 268 |                              .cwiseProduct((var[i].diagonal().cwiseSqrt().asDiagonal().inverse() * (x.row(i) - mean.row(i)).transpose()).transpose())
 269 |                              .array()
 270 |                              .exp() /
 271 |                          std::sqrt(std::pow(2.0 * M_PI, CONTROL_SPACE::dim) * var[i].determinant());
 272 |         }
 273 |         return pdf;
 274 |     }
 275 | 
 276 |     ControlSeq ReverseMPPI::interpolate(const ControlSeq& x1, const ControlSeq& x2, const double& ratio) const {
 277 |         ControlSeq x = x1 * 0.0;
 278 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 279 |             x.row(i) = (1.0 - ratio) * x1.row(i) + ratio * x2.row(i);
 280 |         }
 281 |         return x;
 282 |     }
 283 | 
 284 |     ControlSeqCovMatrices ReverseMPPI::interpolate(const ControlSeqCovMatrices& covs1,
 285 |                                                    const ControlSeqCovMatrices& covs2,
 286 |                                                    const double& ratio) const {
 287 |         ControlSeqCovMatrices covs = covs1;
 288 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 289 |             covs[i] = (1.0 - ratio) * covs1[i] + ratio * covs2[i];
 290 |         }
 291 |         return covs;
 292 |     }
 293 | 
 294 | }  // namespace cpu
 295 | }  // namespace mppi

```

`src\mppi_controller\src\stein_variational_guided_mppi.cpp`:

```cpp
   1 | #include "mppi_controller/stein_variational_guided_mppi.hpp"
   2 | 
   3 | namespace mppi {
   4 | namespace cpu {
   5 |     SVGuidedMPPI::SVGuidedMPPI(const Params::Common& common_params, const Params::SVGuidedMPPI& svg_mppi_params)
   6 |         : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
   7 |           thread_num_(common_params.thread_num),
   8 |           lambda_(svg_mppi_params.lambda),
   9 |           alpha_(svg_mppi_params.alpha),
  10 |           non_biased_sampling_rate_(svg_mppi_params.non_biased_sampling_rate),
  11 |           steer_cov_(svg_mppi_params.steer_cov),
  12 |           sample_num_for_grad_estimation_(svg_mppi_params.sample_num_for_grad_estimation),
  13 |           grad_lambda_(svg_mppi_params.grad_lambda),
  14 |           steer_cov_for_grad_estimation_(svg_mppi_params.steer_cov_for_grad_estimation),
  15 |           svgd_step_size_(svg_mppi_params.svgd_step_size),
  16 |           num_svgd_iteration_(svg_mppi_params.num_svgd_iteration),
  17 |           is_use_nominal_solution_(svg_mppi_params.is_use_nominal_solution),
  18 |           is_covariance_adaptation_(svg_mppi_params.is_covariance_adaptation),
  19 |           gaussian_fitting_lambda_(svg_mppi_params.gaussian_fitting_lambda),
  20 |           min_steer_cov_(svg_mppi_params.min_steer_cov),
  21 |           max_steer_cov_(svg_mppi_params.max_steer_cov) {
  22 |         const size_t sample_batch_num = static_cast<size_t>(svg_mppi_params.sample_batch_num);
  23 |         const size_t guide_sample_num = static_cast<size_t>(svg_mppi_params.guide_sample_num);
  24 |         const size_t sample_num_for_grad_estimation = static_cast<size_t>(svg_mppi_params.sample_num_for_grad_estimation);
  25 | 
  26 |         const size_t sample_num_for_cache = std::max(std::max(sample_batch_num, sample_num_for_grad_estimation), guide_sample_num);
  27 |         mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_num_for_cache);
  28 | 
  29 |         const double max_steer_angle = common_params.max_steer_angle;
  30 |         const double min_steer_angle = common_params.min_steer_angle;
  31 |         std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
  32 |         std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};
  33 |         prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
  34 |                                                                      non_biased_sampling_rate_, thread_num_);
  35 | 
  36 |         guide_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(guide_sample_num, prediction_step_size_, max_control_inputs, min_control_inputs,
  37 |                                                                      non_biased_sampling_rate_, thread_num_);
  38 | 
  39 |         prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
  40 |         nominal_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
  41 | 
  42 |         // initialize prior distribution
  43 |         const ControlSeqCovMatrices control_seq_cov_matrices = guide_samples_ptr_->get_constant_control_seq_cov_matrices({steer_cov_});
  44 |         guide_samples_ptr_->random_sampling(guide_samples_ptr_->get_zero_control_seq(), control_seq_cov_matrices);
  45 | 
  46 |         // initialize grad samplers
  47 |         for (size_t i = 0; i < sample_batch_num; i++) {
  48 |             grad_sampler_ptrs_.emplace_back(std::make_unique<PriorSamplesWithCosts>(sample_num_for_grad_estimation_, prediction_step_size_,
  49 |                                                                                     max_control_inputs, min_control_inputs, non_biased_sampling_rate_,
  50 |                                                                                     thread_num_, i));
  51 |         }
  52 |     }
  53 | 
  54 |     std::pair<ControlSeq, double> SVGuidedMPPI::solve(const State& initial_state) {
  55 |         // Transport guide particles by SVGD
  56 |         std::vector<double> costs_history;
  57 |         std::vector<ControlSeq> control_seq_history;
  58 |         auto func_calc_costs = [&](const PriorSamplesWithCosts& sampler) { return mpc_base_ptr_->calc_sample_costs(sampler, initial_state).first; };
  59 |         for (int i = 0; i < num_svgd_iteration_; i++) {
  60 |             // Transport samples by stein variational gradient descent
  61 |             const ControlSeqBatch grad_log_posterior = approx_grad_posterior_batch(*guide_samples_ptr_, func_calc_costs);
  62 | #pragma omp parallel for num_threads(thread_num_)
  63 |             for (size_t i = 0; i < guide_samples_ptr_->get_num_samples(); i++) {
  64 |                 guide_samples_ptr_->noised_control_seq_samples_[i] += svgd_step_size_ * grad_log_posterior[i];
  65 |             }
  66 | 
  67 |             // store costs and samples for adaptive covariance calculation
  68 |             const std::vector<double> costs = mpc_base_ptr_->calc_sample_costs(*guide_samples_ptr_, initial_state).first;
  69 |             // guide_samples_ptr_->costs_ = costs;
  70 |             // const std::vector<double> cost_with_control_term = guide_samples_ptr_->get_costs_with_control_term(gaussian_fitting_lambda, 0,
  71 |             // prior_samples_ptr_->get_zero_control_seq());
  72 |             costs_history.insert(costs_history.end(), costs.begin(), costs.end());
  73 |             control_seq_history.insert(control_seq_history.end(), guide_samples_ptr_->noised_control_seq_samples_.begin(),
  74 |                                        guide_samples_ptr_->noised_control_seq_samples_.end());
  75 |         }
  76 |         const auto guide_costs = mpc_base_ptr_->calc_sample_costs(*guide_samples_ptr_, initial_state).first;
  77 |         const size_t min_idx = std::distance(guide_costs.begin(), std::min_element(guide_costs.begin(), guide_costs.end()));
  78 |         const ControlSeq best_particle = guide_samples_ptr_->noised_control_seq_samples_[min_idx];
  79 | 
  80 |         // calculate adaptive covariance matrices for prior distribution
  81 |         // TODO: Support multiple control input dimensions
  82 |         ControlSeqCovMatrices covs = prior_samples_ptr_->get_constant_control_seq_cov_matrices({steer_cov_});
  83 |         if (is_covariance_adaptation_) {
  84 |             // calculate softmax costs
  85 |             const std::vector<double> softmax_costs = softmax(costs_history, gaussian_fitting_lambda_, thread_num_);
  86 | 
  87 |             // calculate covariance using gaussian fitting
  88 |             for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
  89 |                 std::vector<double> steer_samples(control_seq_history.size());
  90 |                 std::vector<double> q_star(softmax_costs.size());
  91 |                 for (size_t j = 0; j < steer_samples.size(); j++) {
  92 |                     steer_samples[j] = control_seq_history[j](i, 0);
  93 |                     q_star[j] = softmax_costs[j];
  94 |                 }
  95 |                 const double sigma = gaussian_fitting(steer_samples, q_star).second;
  96 | 
  97 |                 const double sigma_clamped = std::clamp(sigma, min_steer_cov_, max_steer_cov_);
  98 | 
  99 |                 covs[i] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * sigma_clamped;
 100 |             }
 101 |         }
 102 | 
 103 |         // random sampling from prior distribution
 104 |         prior_samples_ptr_->random_sampling(prev_control_seq_, covs);
 105 | 
 106 |         // Rollout samples and calculate costs
 107 |         auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
 108 |         prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
 109 | 
 110 |         // calculate weights
 111 |         if (is_use_nominal_solution_) {
 112 |             // with nominal sequence
 113 |             nominal_control_seq_ = best_particle;
 114 |         } else {
 115 |             // without nominal sequence
 116 |             nominal_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
 117 |         }
 118 |         const std::vector<double> weights = calc_weights(*prior_samples_ptr_, nominal_control_seq_);
 119 |         weights_ = weights;  // for visualization
 120 | 
 121 |         // Get control input sequence by weighted average of samples
 122 |         ControlSeq updated_control_seq = prior_samples_ptr_->get_zero_control_seq();
 123 |         for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
 124 |             updated_control_seq += weights[i] * prior_samples_ptr_->noised_control_seq_samples_.at(i);
 125 |         }
 126 | 
 127 |         const int collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });
 128 |         const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());
 129 | 
 130 |         // update previous control sequence for next time step
 131 |         prev_control_seq_ = updated_control_seq;
 132 | 
 133 |         return std::make_pair(updated_control_seq, collision_rate);
 134 |     }
 135 | 
 136 |     void SVGuidedMPPI::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };
 137 | 
 138 |     void SVGuidedMPPI::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };
 139 | 
 140 |     std::pair<std::vector<StateSeq>, std::vector<double>> SVGuidedMPPI::get_state_seq_candidates(const int& num_samples) const {
 141 |         return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
 142 |     }
 143 | 
 144 |     std::tuple<StateSeq, double, double, double> SVGuidedMPPI::get_predictive_seq(const State& initial_state,
 145 |                                                                                   const ControlSeq& control_input_seq) const {
 146 |         const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
 147 |         double input_error = 0.0;
 148 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 149 |             input_error += (control_input_seq.row(i)).norm();
 150 |         }
 151 |         return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
 152 |     }
 153 | 
 154 |     ControlSeqCovMatrices SVGuidedMPPI::get_cov_matrices() const { return prior_samples_ptr_->get_cov_matrices(); }
 155 | 
 156 |     ControlSeq SVGuidedMPPI::get_control_seq() const { return nominal_control_seq_; }
 157 | 
 158 |     std::pair<StateSeq, XYCovMatrices> SVGuidedMPPI::get_proposed_state_distribution() const { return mpc_base_ptr_->get_proposed_distribution(); }
 159 | 
 160 |     // == private functions ==
 161 |     ControlSeq SVGuidedMPPI::approx_grad_log_likelihood(const ControlSeq& mean_seq,
 162 |                                                         const ControlSeq& noised_seq,
 163 |                                                         const ControlSeqCovMatrices& inv_covs,
 164 |                                                         const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
 165 |                                                         PriorSamplesWithCosts* sampler) const {
 166 |         const ControlSeqCovMatrices grad_cov = sampler->get_constant_control_seq_cov_matrices({steer_cov_for_grad_estimation_});
 167 | 
 168 |         // generate gaussian random samples, center of which is input_seq
 169 |         sampler->random_sampling(noised_seq, grad_cov);
 170 | 
 171 |         // calculate forward simulation and costs
 172 |         sampler->costs_ = calc_costs(*sampler);
 173 | 
 174 |         // calculate cost with control term
 175 |         // const std::vector<double> costs_with_control_term = sampler->get_costs_with_control_term(grad_lambda_, 0.0,
 176 |         // sampler->get_zero_control_seq());
 177 |         std::vector<double> exp_costs(sampler->get_num_samples());
 178 |         ControlSeq sum_of_grads = mean_seq * 0.0;
 179 |         const ControlSeqCovMatrices sampler_inv_covs = sampler->get_inv_cov_matrices();
 180 | #pragma omp parallel for num_threads(thread_num_)
 181 |         for (size_t i = 0; i < sampler->get_num_samples(); i++) {
 182 |             double cost_with_control_term = sampler->costs_[i];
 183 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 184 |                 // const double control_term = grad_lambda_ * prev_control_seq_.row(j) * inv_covs[j] *
 185 |                 // sampler->noised_control_seq_samples_[i].row(j).transpose();
 186 |                 const double diff_control_term = grad_lambda_ * (prev_control_seq_.row(j) - sampler->noised_control_seq_samples_[i].row(j)) *
 187 |                                                  inv_covs[j] *
 188 |                                                  (prev_control_seq_.row(j) - sampler->noised_control_seq_samples_[i].row(j)).transpose();
 189 |                 // cost_with_control_term += control_term + diff_control_term;
 190 |                 cost_with_control_term += diff_control_term;
 191 |             }
 192 |             const double exp_cost = std::exp(-cost_with_control_term / grad_lambda_);
 193 |             exp_costs[i] = exp_cost;
 194 | 
 195 |             ControlSeq grad_log_gaussian = mean_seq * 0.0;
 196 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 197 |                 grad_log_gaussian.row(j) = exp_cost * sampler_inv_covs[j] * (sampler->noised_control_seq_samples_[i] - noised_seq).row(j).transpose();
 198 |             }
 199 |             sum_of_grads += grad_log_gaussian;
 200 |         }
 201 |         const double sum_of_costs = std::accumulate(exp_costs.begin(), exp_costs.end(), 0.0);
 202 |         return sum_of_grads / (sum_of_costs + 1e-10);
 203 |     }
 204 | 
 205 |     ControlSeqBatch SVGuidedMPPI::approx_grad_posterior_batch(
 206 |         const PriorSamplesWithCosts& samples,
 207 |         const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const {
 208 |         ControlSeqBatch grad_log_likelihoods = samples.get_zero_control_seq_batch();
 209 |         const ControlSeq mean = samples.get_mean();
 210 |         // #pragma omp parallel for num_threads(thread_num_)
 211 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 212 |             const ControlSeq grad_log_likelihood = approx_grad_log_likelihood(
 213 |                 mean, samples.noised_control_seq_samples_[i], samples.get_inv_cov_matrices(), calc_costs, grad_sampler_ptrs_.at(i).get());
 214 |             grad_log_likelihoods[i] = grad_log_likelihood;
 215 |         }
 216 | 
 217 |         return grad_log_likelihoods;
 218 |     }
 219 | 
 220 |     std::pair<ControlSeq, ControlSeqCovMatrices> SVGuidedMPPI::weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
 221 |                                                                                        const std::vector<double>& weights) const {
 222 |         ControlSeq mean = samples.get_zero_control_seq();
 223 |         ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();
 224 | 
 225 |         const ControlSeq prior_mean = samples.get_mean();
 226 |         const ControlSeqCovMatrices prior_inv_covs = samples.get_inv_cov_matrices();
 227 | #pragma omp parallel for num_threads(thread_num_)
 228 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 229 |             mean += weights[i] * samples.noised_control_seq_samples_[i];
 230 | 
 231 |             const ControlSeq diff = samples.noised_control_seq_samples_[i] - prior_mean;
 232 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 233 |                 sigma[j] += weights[i] * diff.row(j).transpose() * prior_inv_covs[j] * diff.row(j);
 234 |             }
 235 |         }
 236 | 
 237 |         return std::make_pair(mean, sigma);
 238 |     }
 239 | 
 240 |     std::pair<ControlSeq, ControlSeqCovMatrices> SVGuidedMPPI::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
 241 |         ControlSeq mu = samples.get_zero_control_seq();
 242 |         ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();
 243 | 
 244 | // calculate mean
 245 | #pragma omp parallel for num_threads(thread_num_)
 246 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 247 |             mu += samples.noised_control_seq_samples_[i];
 248 |         }
 249 |         mu /= static_cast<double>(samples.get_num_samples());
 250 | 
 251 | #pragma omp parallel for num_threads(thread_num_)
 252 |         // calculate covariance matrices
 253 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 254 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 255 |                 sigma[j] += (samples.noised_control_seq_samples_[i].row(j) - mu.row(j)).transpose() *
 256 |                             (samples.noised_control_seq_samples_[i].row(j) - mu.row(j));
 257 |             }
 258 |         }
 259 | 
 260 |         for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 261 |             sigma[j] /= static_cast<double>(samples.get_num_samples());
 262 | 
 263 |             // add small value to avoid singular matrix
 264 |             sigma[j] += 1e-5 * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
 265 |         }
 266 | 
 267 |         return std::make_pair(mu, sigma);
 268 |     }
 269 | 
 270 |     std::vector<double> SVGuidedMPPI::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs,
 271 |                                                    const ControlSeq& nominal_control_seq) const {
 272 |         // calculate costs with control term
 273 |         const std::vector<double> costs_with_control_term =
 274 |             prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, nominal_control_seq);
 275 | 
 276 |         // softmax weights
 277 |         return softmax(costs_with_control_term, lambda_, thread_num_);
 278 |     }
 279 | 
 280 |     // Gao's Algorithm
 281 |     // H. GUO, “A Simple Algorithm for Fitting a Gaussian Function,” IEEE Signal Process, Mag.28, No. 5 (2011), 134–137.
 282 |     std::pair<double, double> SVGuidedMPPI::gaussian_fitting(const std::vector<double>& x, const std::vector<double>& y) const {
 283 |         assert(x.size() == y.size());
 284 | 
 285 |         // Should y is larger than 0 for log function
 286 |         std::vector<double> y_hat(y.size(), 0.0);
 287 |         std::transform(y.begin(), y.end(), y_hat.begin(), [](double y) { return std::max(y, 1e-10); });
 288 | 
 289 |         // const double epsilon = 1e-8;
 290 |         Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
 291 |         Eigen::Vector3d b = Eigen::Vector3d::Zero();
 292 |         for (size_t i = 0; i < x.size(); i++) {
 293 |             const double y_hat_2 = y_hat[i] * y_hat[i];
 294 |             const double y_hat_log = std::log(y_hat[i]);
 295 | 
 296 |             A(0, 0) += y_hat_2;
 297 |             A(0, 1) += y_hat_2 * x[i];
 298 |             A(0, 2) += y_hat_2 * x[i] * x[i];
 299 | 
 300 |             A(1, 0) += y_hat_2 * x[i];
 301 |             A(1, 1) += y_hat_2 * x[i] * x[i];
 302 |             A(1, 2) += y_hat_2 * x[i] * x[i] * x[i];
 303 | 
 304 |             A(2, 0) += y_hat_2 * x[i] * x[i];
 305 |             A(2, 1) += y_hat_2 * x[i] * x[i] * x[i];
 306 |             A(2, 2) += y_hat_2 * x[i] * x[i] * x[i] * x[i];
 307 | 
 308 |             b(0) += y_hat_2 * y_hat_log;
 309 |             b(1) += y_hat_2 * x[i] * y_hat_log;
 310 |             b(2) += y_hat_2 * x[i] * x[i] * y_hat_log;
 311 |         }
 312 | 
 313 |         // solve Au = b
 314 |         const Eigen::Vector3d u = A.colPivHouseholderQr().solve(b);
 315 | 
 316 |         // calculate mean and variance
 317 | 
 318 |         // original
 319 |         // const double mean = -u(1) / (2.0 * u(2));
 320 |         // const double variance = std::sqrt(-1.0 / (2.0 * u(2)));
 321 | 
 322 |         // To avoid nan;
 323 |         const double eps = 1e-5;
 324 |         const double mean = -u(1) / (2.0 * std::min(u(2), -eps));
 325 |         // const double variance = std::sqrt(1.0 / (2.0 * std::abs(std::min(u(2), -eps))));
 326 |         const double variance = std::sqrt(1.0 / (2.0 * std::abs(u(2))));
 327 | 
 328 |         return std::make_pair(mean, variance);
 329 |     }
 330 | 
 331 | }  // namespace cpu
 332 | }  // namespace mppi

```

`src\mppi_controller\src\stein_variational_mpc.cpp`:

```cpp
   1 | #include "mppi_controller/stein_variational_mpc.hpp"
   2 | 
   3 | namespace mppi {
   4 | namespace cpu {
   5 |     SteinVariationalMPC::SteinVariationalMPC(const Params::Common& common_params, const Params::SteinVariationalMPC& sv_mpc_params)
   6 |         : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
   7 |           thread_num_(common_params.thread_num),
   8 |           lambda_(sv_mpc_params.lambda),
   9 |           alpha_(sv_mpc_params.alpha),
  10 |           non_biased_sampling_rate_(sv_mpc_params.non_biased_sampling_rate),
  11 |           steer_cov_(sv_mpc_params.steer_cov),
  12 |           num_svgd_iteration_(sv_mpc_params.num_svgd_iteration),
  13 |           steer_cov_for_grad_estimation_(sv_mpc_params.steer_cov_for_grad_estimation),
  14 |           svgd_step_size_(sv_mpc_params.svgd_step_size),
  15 |           is_max_posterior_estimation_(sv_mpc_params.is_max_posterior_estimation) {
  16 |         const size_t sample_batch_num = static_cast<size_t>(sv_mpc_params.sample_batch_num);
  17 |         const size_t sample_num_for_grad_estimation = static_cast<size_t>(sv_mpc_params.sample_num_for_grad_estimation);
  18 | 
  19 |         const size_t sample_num_for_cache = std::max(sample_batch_num, sample_num_for_grad_estimation);
  20 |         mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_num_for_cache);
  21 | 
  22 |         const double max_steer_angle = common_params.max_steer_angle;
  23 |         const double min_steer_angle = common_params.min_steer_angle;
  24 |         std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
  25 |         std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};
  26 |         prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
  27 |                                                                      non_biased_sampling_rate_, thread_num_);
  28 |         prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
  29 | 
  30 |         // initialize prior distribution
  31 |         const std::array<double, CONTROL_SPACE::dim> control_cov_diag = {steer_cov_};
  32 |         ControlSeqCovMatrices control_seq_cov_matrices = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);
  33 |         prior_samples_ptr_->random_sampling(prior_samples_ptr_->get_zero_control_seq(), control_seq_cov_matrices);
  34 | 
  35 |         // initialize grad samplers
  36 |         for (size_t i = 0; i < sample_batch_num; i++) {
  37 |             grad_sampler_ptrs_.emplace_back(std::make_unique<PriorSamplesWithCosts>(
  38 |                 sample_num_for_grad_estimation, prediction_step_size_, max_control_inputs, min_control_inputs, non_biased_sampling_rate_, 1, i));
  39 |         }
  40 |     }
  41 | 
  42 |     std::pair<ControlSeq, double> SteinVariationalMPC::solve(const State& initial_state) {
  43 |         // calculate gradient of log posterior (= approx grad log_likelihood estimated by monte carlo method + grad log prior)
  44 |         // NOTE: currently, prior is approximated as normal distribution
  45 |         auto func_calc_costs = [&](const PriorSamplesWithCosts& sampler) {
  46 |             const auto [costs, _flags] = mpc_base_ptr_->calc_sample_costs(sampler, initial_state);
  47 |             return costs;
  48 |         };
  49 | 
  50 |         for (int i = 0; i < num_svgd_iteration_; i++) {
  51 |             const ControlSeqBatch grad_log_posterior = approx_grad_posterior_batch(*prior_samples_ptr_, func_calc_costs);
  52 | 
  53 |             // Transport samples by stein variational gradient descent
  54 |             const ControlSeqBatch phis = phi_batch(*prior_samples_ptr_, grad_log_posterior);
  55 | #pragma omp parallel for num_threads(thread_num_)
  56 |             for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
  57 |                 prior_samples_ptr_->noised_control_seq_samples_[i] += svgd_step_size_ * phis[i];
  58 |             }
  59 |         }
  60 | 
  61 |         // calculate weights for shifting prior distribution
  62 |         auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
  63 |         prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
  64 |         const std::vector<double> weights = calc_weights(*prior_samples_ptr_);
  65 | 
  66 |         const int collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });
  67 |         const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());
  68 | 
  69 |         // NAN and inf check for weights
  70 |         auto has_nan_or_inf = [](const double& x) { return std::isnan(x) || std::isinf(x); };
  71 |         const bool is_valid = std::none_of(weights.begin(), weights.end(), has_nan_or_inf);
  72 | 
  73 |         if (is_valid) {
  74 |             weights_ = weights;  // for visualization
  75 | 
  76 |             ControlSeq updated_control_seq = prior_samples_ptr_->get_zero_control_seq();
  77 |             if (is_max_posterior_estimation_) {
  78 |                 // max posterior
  79 |                 const int max_idx = std::distance(weights.begin(), std::max_element(weights.begin(), weights.end()));
  80 |                 updated_control_seq = prior_samples_ptr_->noised_control_seq_samples_.at(max_idx);
  81 |             } else {
  82 |                 // Get control input sequence by weighted average of samples
  83 |                 for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
  84 |                     updated_control_seq += weights[i] * prior_samples_ptr_->noised_control_seq_samples_.at(i);
  85 |                 }
  86 |             }
  87 | 
  88 |             // shift particles
  89 |             // This cause unstable behavior
  90 |             // #pragma omp parallel for num_threads(thread_num_)
  91 |             //                 for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++)
  92 |             //                 {
  93 |             //                     // shift control sequence by 1 step
  94 |             //                     ControlSeq tmp = prior_samples_ptr_->get_zero_control_seq();
  95 |             //                     for (size_t j = 0; j < prediction_step_size_ - 2; j++)
  96 |             //                     {
  97 |             //                         tmp.row(j) = prior_samples_ptr_->noised_control_seq_samples_.at(i).row(j + 1);
  98 |             //                     }
  99 |             //                     tmp.row(prediction_step_size_ - 1) = updated_control_seq.row(prediction_step_size_ - 2);
 100 |             //                     prior_samples_ptr_->noised_control_seq_samples_.at(i) = tmp;
 101 |             //                 }
 102 | 
 103 |             // update prior distribution
 104 |             const auto [new_mean, new_cov] = weighted_mean_and_sigma(*prior_samples_ptr_, weights);
 105 |             prior_samples_ptr_->set_control_seq_mean(new_mean);
 106 |             prior_samples_ptr_->set_control_seq_cov_matrices(new_cov);
 107 | 
 108 |             prev_control_seq_ = updated_control_seq;
 109 |             return std::make_pair(updated_control_seq, collision_rate);
 110 |         } else {
 111 |             // std::cout << "NAN or INF is detected in control sequence. Resampling..." << std::endl;
 112 |             prior_samples_ptr_->random_sampling(prior_samples_ptr_->get_zero_control_seq(),
 113 |                                                 prior_samples_ptr_->get_constant_control_seq_cov_matrices({steer_cov_}));
 114 | 
 115 |             return std::make_pair(prev_control_seq_, collision_rate);
 116 |         }
 117 |     }
 118 | 
 119 |     void SteinVariationalMPC::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };
 120 | 
 121 |     void SteinVariationalMPC::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };
 122 | 
 123 |     std::pair<std::vector<StateSeq>, std::vector<double>> SteinVariationalMPC::get_state_seq_candidates(const int& num_samples) const {
 124 |         return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
 125 |     }
 126 | 
 127 |     std::tuple<StateSeq, double, double, double> SteinVariationalMPC::get_predictive_seq(const State& initial_state,
 128 |                                                                                          const ControlSeq& control_input_seq) const {
 129 |         const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
 130 |         double input_error = 0.0;
 131 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 132 |             input_error += (control_input_seq.row(i)).norm();
 133 |         }
 134 |         return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
 135 |     }
 136 | 
 137 |     ControlSeqCovMatrices SteinVariationalMPC::get_cov_matrices() const {
 138 |         const auto [_mean, cov] = estimate_mu_and_sigma(*prior_samples_ptr_);
 139 |         return cov;
 140 |     }
 141 | 
 142 |     ControlSeq SteinVariationalMPC::get_control_seq() const { return prev_control_seq_; }
 143 | 
 144 |     std::pair<StateSeq, XYCovMatrices> SteinVariationalMPC::get_proposed_state_distribution() const {
 145 |         return mpc_base_ptr_->get_proposed_distribution();
 146 |     }
 147 | 
 148 |     // == private functions ==
 149 |     ControlSeq SteinVariationalMPC::approx_grad_log_likelihood(const ControlSeq& mean_seq,
 150 |                                                                const ControlSeq& noised_seq,
 151 |                                                                const ControlSeqCovMatrices& inv_covs,
 152 |                                                                const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
 153 |                                                                PriorSamplesWithCosts* sampler) const {
 154 |         const ControlSeqCovMatrices grad_cov = sampler->get_constant_control_seq_cov_matrices({steer_cov_for_grad_estimation_});
 155 | 
 156 |         // generate gaussian random samples, center of which is input_seq
 157 |         sampler->random_sampling(noised_seq, grad_cov);
 158 | 
 159 |         // calculate forward simulation and costs
 160 |         sampler->costs_ = calc_costs(*sampler);
 161 | 
 162 |         double sum_of_costs = 0.0;
 163 |         ControlSeq sum_of_grads = mean_seq * 0.0;
 164 |         const ControlSeqCovMatrices sampler_inv_covs = sampler->get_inv_cov_matrices();
 165 |         for (size_t i = 0; i < sampler->get_num_samples(); i++) {
 166 |             double cost_with_control_term = (sampler->costs_[i]) / lambda_;
 167 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 168 |                 const double control_term = mean_seq.row(j) * inv_covs[j] * sampler->noised_control_seq_samples_[i].row(j).transpose();
 169 |                 cost_with_control_term += control_term;
 170 |             }
 171 |             const double exp_cost = std::exp(-cost_with_control_term);
 172 |             sum_of_costs += exp_cost;
 173 | 
 174 |             ControlSeq grad_log_gaussian = mean_seq * 0.0;
 175 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 176 |                 grad_log_gaussian.row(j) = exp_cost * sampler_inv_covs[j] * (sampler->noised_control_seq_samples_[i] - noised_seq).row(j).transpose();
 177 |             }
 178 |             sum_of_grads += grad_log_gaussian;
 179 |         }
 180 | 
 181 |         return sum_of_grads / (sum_of_costs + 1e-10);
 182 |     }
 183 | 
 184 |     ControlSeq SteinVariationalMPC::grad_log_normal_dist(const ControlSeq& sample,
 185 |                                                          const ControlSeq& prior_mean,
 186 |                                                          const ControlSeqCovMatrices& prior_covs) const {
 187 |         // Gradient of log prior distribution
 188 |         // the prior distribution is assumed to be normal distribution
 189 |         // TODO: approximate as GMM like https://github.com/lubaroli/dust/blob/master/dust/inference/svgd.py
 190 |         ControlSeq grad_log_prior = sample * 0.0;
 191 |         ControlSeq diff = sample - prior_mean;
 192 |         for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
 193 |             grad_log_prior.row(i) = prior_covs[i].inverse() * diff.row(i).transpose();
 194 |         }
 195 |         return grad_log_prior;
 196 |     }
 197 | 
 198 |     ControlSeqBatch SteinVariationalMPC::approx_grad_posterior_batch(
 199 |         const PriorSamplesWithCosts& samples,
 200 |         const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const {
 201 |         ControlSeqBatch grad_log_likelihoods = samples.get_zero_control_seq_batch();
 202 |         const ControlSeq mean = samples.get_mean();
 203 | #pragma omp parallel for num_threads(thread_num_)
 204 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 205 |             const ControlSeq grad_log_likelihood = approx_grad_log_likelihood(
 206 |                 mean, samples.noised_control_seq_samples_[i], samples.get_inv_cov_matrices(), calc_costs, grad_sampler_ptrs_.at(i).get());
 207 |             const ControlSeq grad_log_prior =
 208 |                 grad_log_normal_dist(samples.noised_control_seq_samples_[i], samples.get_mean(), samples.get_inv_cov_matrices());
 209 |             grad_log_likelihoods[i] = grad_log_likelihood + grad_log_prior;
 210 |         }
 211 | 
 212 |         return grad_log_likelihoods;
 213 |     }
 214 | 
 215 |     double SteinVariationalMPC::RBF_kernel(const ControlSeq& seq1, const ControlSeq& seq2, const double& h) const {
 216 |         const double kernel = (seq1 - seq2).squaredNorm();
 217 |         return std::exp(-kernel / h);
 218 |     }
 219 | 
 220 |     ControlSeq SteinVariationalMPC::grad_RBF_kernel(const ControlSeq& seq, const ControlSeq& seq_const, const double& h) const {
 221 |         const double kernel = RBF_kernel(seq, seq_const, h);
 222 |         const ControlSeq grad_kernel = -2.0 / h * kernel * (seq - seq_const);
 223 |         return grad_kernel;
 224 |     }
 225 | 
 226 |     ControlSeqBatch SteinVariationalMPC::phi_batch(const PriorSamplesWithCosts& samples, const ControlSeqBatch& grad_posterior_batch) const {
 227 |         // calculate median of samples
 228 |         // This makes the sum of kernel values close to 1
 229 |         std::vector<double> dists(samples.get_num_samples(), 0.0);
 230 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 231 |             dists[i] = (samples.noised_control_seq_samples_[i]).squaredNorm();
 232 |         }
 233 |         std::sort(dists.begin(), dists.end());
 234 |         double h = dists[static_cast<size_t>(samples.get_num_samples() / 2)] / std::log(static_cast<double>(samples.get_num_samples()));
 235 |         h = std::max(h, 1e-5);
 236 | 
 237 |         // calculate phi batch
 238 |         ControlSeqBatch phi_batch = samples.get_zero_control_seq_batch();
 239 | #pragma omp parallel for num_threads(thread_num_)
 240 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 241 |             for (size_t j = 0; j < samples.get_num_samples(); j++) {
 242 |                 const double kernel = RBF_kernel(samples.noised_control_seq_samples_[j], samples.noised_control_seq_samples_[i], h);
 243 | 
 244 |                 phi_batch[i] += kernel * grad_posterior_batch[j];
 245 |                 phi_batch[i] += grad_RBF_kernel(samples.noised_control_seq_samples_[j], samples.noised_control_seq_samples_[i], h);
 246 |             }
 247 | 
 248 |             phi_batch[i] /= static_cast<double>(samples.get_num_samples());
 249 |         }
 250 | 
 251 |         return phi_batch;
 252 |     }
 253 | 
 254 |     std::pair<ControlSeq, ControlSeqCovMatrices> SteinVariationalMPC::weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
 255 |                                                                                               const std::vector<double>& weights) const {
 256 |         ControlSeq mean = samples.get_zero_control_seq();
 257 |         ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();
 258 | 
 259 |         const ControlSeq prior_mean = samples.get_mean();
 260 |         const ControlSeqCovMatrices prior_inv_covs = samples.get_inv_cov_matrices();
 261 | #pragma omp parallel for num_threads(thread_num_)
 262 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 263 |             mean += weights[i] * samples.noised_control_seq_samples_[i];
 264 | 
 265 |             const ControlSeq diff = samples.noised_control_seq_samples_[i] - prior_mean;
 266 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 267 |                 sigma[j] += weights[i] * diff.row(j).transpose() * prior_inv_covs[j] * diff.row(j);
 268 |             }
 269 |         }
 270 | 
 271 |         return std::make_pair(mean, sigma);
 272 |     }
 273 | 
 274 |     std::pair<ControlSeq, ControlSeqCovMatrices> SteinVariationalMPC::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
 275 |         ControlSeq mu = samples.get_zero_control_seq();
 276 |         ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();
 277 | 
 278 | // calculate mean
 279 | #pragma omp parallel for num_threads(thread_num_)
 280 | 
 281 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 282 |             mu += samples.noised_control_seq_samples_[i];
 283 |         }
 284 |         mu /= static_cast<double>(samples.get_num_samples());
 285 | 
 286 |         // mu = samples.control_seq_mean_;
 287 | 
 288 | // calculate covariance matrices
 289 | #pragma omp parallel for num_threads(thread_num_)
 290 |         for (size_t i = 0; i < samples.get_num_samples(); i++) {
 291 |             for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 292 |                 sigma[j] += (samples.noised_control_seq_samples_[i].row(j) - mu.row(j)).transpose() *
 293 |                             (samples.noised_control_seq_samples_[i].row(j) - mu.row(j));
 294 |             }
 295 |         }
 296 | 
 297 |         for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
 298 |             sigma[j] /= static_cast<double>(samples.get_num_samples());
 299 | 
 300 |             // add small value to avoid singular matrix
 301 |             sigma[j] += 1e-5 * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
 302 |         }
 303 | 
 304 |         return std::make_pair(mu, sigma);
 305 |     }
 306 | 
 307 |     std::vector<double> SteinVariationalMPC::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const {
 308 |         // calculate costs with control cost term
 309 |         const std::vector<double> costs_with_control_term =
 310 |             prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, prior_samples_ptr_->get_zero_control_seq());
 311 | 
 312 |         // softmax weights
 313 |         return softmax(costs_with_control_term, lambda_, thread_num_);
 314 |     }
 315 | 
 316 | }  // namespace cpu
 317 | }  // namespace mppi

```

`src\racecar_model\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.0.2)
   2 | project(racecar_model)
   3 | 
   4 | ## Compile as C++11, supported in ROS Kinetic and newer
   5 | # add_compile_options(-std=c++11)
   6 | 
   7 | ## Find catkin macros and libraries
   8 | ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
   9 | ## is used, also find other catkin packages
  10 | find_package(catkin REQUIRED)
  11 | 
  12 | ## System dependencies are found with CMake's conventions
  13 | # find_package(Boost REQUIRED COMPONENTS system)
  14 | 
  15 | 
  16 | ## Uncomment this if the package has a setup.py. This macro ensures
  17 | ## modules and global scripts declared therein get installed
  18 | ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
  19 | # catkin_python_setup()
  20 | 
  21 | ################################################
  22 | ## Declare ROS messages, services and actions ##
  23 | ################################################
  24 | 
  25 | ## To declare and build messages, services or actions from within this
  26 | ## package, follow these steps:
  27 | ## * Let MSG_DEP_SET be the set of packages whose message types you use in
  28 | ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
  29 | ## * In the file package.xml:
  30 | ##   * add a build_depend tag for "message_generation"
  31 | ##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
  32 | ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
  33 | ##     but can be declared for certainty nonetheless:
  34 | ##     * add a exec_depend tag for "message_runtime"
  35 | ## * In this file (CMakeLists.txt):
  36 | ##   * add "message_generation" and every package in MSG_DEP_SET to
  37 | ##     find_package(catkin REQUIRED COMPONENTS ...)
  38 | ##   * add "message_runtime" and every package in MSG_DEP_SET to
  39 | ##     catkin_package(CATKIN_DEPENDS ...)
  40 | ##   * uncomment the add_*_files sections below as needed
  41 | ##     and list every .msg/.srv/.action file to be processed
  42 | ##   * uncomment the generate_messages entry below
  43 | ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
  44 | 
  45 | ## Generate messages in the 'msg' folder
  46 | # add_message_files(
  47 | #   FILES
  48 | #   Message1.msg
  49 | #   Message2.msg
  50 | # )
  51 | 
  52 | ## Generate services in the 'srv' folder
  53 | # add_service_files(
  54 | #   FILES
  55 | #   Service1.srv
  56 | #   Service2.srv
  57 | # )
  58 | 
  59 | ## Generate actions in the 'action' folder
  60 | # add_action_files(
  61 | #   FILES
  62 | #   Action1.action
  63 | #   Action2.action
  64 | # )
  65 | 
  66 | ## Generate added messages and services with any dependencies listed here
  67 | # generate_messages(
  68 | #   DEPENDENCIES
  69 | #   std_msgs  # Or other packages containing msgs
  70 | # )
  71 | 
  72 | ################################################
  73 | ## Declare ROS dynamic reconfigure parameters ##
  74 | ################################################
  75 | 
  76 | ## To declare and build dynamic reconfigure parameters within this
  77 | ## package, follow these steps:
  78 | ## * In the file package.xml:
  79 | ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
  80 | ## * In this file (CMakeLists.txt):
  81 | ##   * add "dynamic_reconfigure" to
  82 | ##     find_package(catkin REQUIRED COMPONENTS ...)
  83 | ##   * uncomment the "generate_dynamic_reconfigure_options" section below
  84 | ##     and list every .cfg file to be processed
  85 | 
  86 | ## Generate dynamic reconfigure parameters in the 'cfg' folder
  87 | # generate_dynamic_reconfigure_options(
  88 | #   cfg/DynReconf1.cfg
  89 | #   cfg/DynReconf2.cfg
  90 | # )
  91 | 
  92 | ###################################
  93 | ## catkin specific configuration ##
  94 | ###################################
  95 | ## The catkin_package macro generates cmake config files for your package
  96 | ## Declare things to be passed to dependent projects
  97 | ## INCLUDE_DIRS: uncomment this if your package contains header files
  98 | ## LIBRARIES: libraries you create in this project that dependent projects also need
  99 | ## CATKIN_DEPENDS: catkin_packages dependent projects also need
 100 | ## DEPENDS: system dependencies of this project that dependent projects also need
 101 | catkin_package(
 102 | #  INCLUDE_DIRS include
 103 | #  LIBRARIES racecar_model
 104 | #  CATKIN_DEPENDS other_catkin_pkg
 105 | #  DEPENDS system_lib
 106 | )
 107 | 
 108 | ###########
 109 | ## Build ##
 110 | ###########
 111 | 
 112 | ## Specify additional locations of header files
 113 | ## Your package locations should be listed before other locations
 114 | include_directories(
 115 | # include
 116 | # ${catkin_INCLUDE_DIRS}
 117 | )
 118 | 
 119 | ## Declare a C++ library
 120 | # add_library(${PROJECT_NAME}
 121 | #   src/${PROJECT_NAME}/racecar_model.cpp
 122 | # )
 123 | 
 124 | ## Add cmake target dependencies of the library
 125 | ## as an example, code may need to be generated before libraries
 126 | ## either from message generation or dynamic reconfigure
 127 | # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 128 | 
 129 | ## Declare a C++ executable
 130 | ## With catkin_make all packages are built within a single CMake context
 131 | ## The recommended prefix ensures that target names across packages don't collide
 132 | # add_executable(${PROJECT_NAME}_node src/racecar_model_node.cpp)
 133 | 
 134 | ## Rename C++ executable without prefix
 135 | ## The above recommended prefix causes long target names, the following renames the
 136 | ## target back to the shorter version for ease of user use
 137 | ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
 138 | # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
 139 | 
 140 | ## Add cmake target dependencies of the executable
 141 | ## same as for the library above
 142 | # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 143 | 
 144 | ## Specify libraries to link a library or executable target against
 145 | # target_link_libraries(${PROJECT_NAME}_node
 146 | #   ${catkin_LIBRARIES}
 147 | # )
 148 | 
 149 | #############
 150 | ## Install ##
 151 | #############
 152 | 
 153 | # all install targets should use catkin DESTINATION variables
 154 | # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
 155 | 
 156 | ## Mark executable scripts (Python etc.) for installation
 157 | ## in contrast to setup.py, you can choose the destination
 158 | # catkin_install_python(PROGRAMS
 159 | #   scripts/my_python_script
 160 | #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 161 | # )
 162 | 
 163 | ## Mark executables for installation
 164 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 165 | # install(TARGETS ${PROJECT_NAME}_node
 166 | #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 167 | # )
 168 | 
 169 | ## Mark libraries for installation
 170 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 171 | # install(TARGETS ${PROJECT_NAME}
 172 | #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 173 | #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 174 | #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 175 | # )
 176 | 
 177 | ## Mark cpp header files for installation
 178 | # install(DIRECTORY include/${PROJECT_NAME}/
 179 | #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 180 | #   FILES_MATCHING PATTERN "*.h"
 181 | #   PATTERN ".svn" EXCLUDE
 182 | # )
 183 | 
 184 | ## Mark other files for installation (e.g. launch and bag files, etc.)
 185 | # install(FILES
 186 | #   # myfile1
 187 | #   # myfile2
 188 | #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 189 | # )
 190 | 
 191 | #############
 192 | ## Testing ##
 193 | #############
 194 | 
 195 | ## Add gtest based cpp test target and link libraries
 196 | # catkin_add_gtest(${PROJECT_NAME}-test test/test_racecar_model.cpp)
 197 | # if(TARGET ${PROJECT_NAME}-test)
 198 | #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 199 | # endif()
 200 | 
 201 | ## Add folders to be run by python nosetests
 202 | # catkin_add_nosetests(test)

```

`src\racecar_model\ego_racecar.xacro`:

```xacro
   1 | <?xml version="1.0"?>
   2 | 
   3 | <!-- A simple model of the racecar for rviz -->
   4 | 
   5 | <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="racecar">
   6 | 
   7 |   <xacro:property name="wheelbase" value="0.3302" />
   8 |   <xacro:property name="width" value="0.2032" />
   9 |   <xacro:property name="height" value="0.1" />
  10 |   <xacro:property name="ground_offset" value="0.04" />
  11 |   <xacro:property name="wheel_radius" value="0.0508" />
  12 |   <xacro:property name="wheel_length" value="0.0381" />
  13 |   <xacro:property name="laser_distance_from_base_link" value="0.275" />
  14 |   <xacro:property name="laser_height" value="0.05" />
  15 |   <xacro:property name="laser_radius" value="0.026" />
  16 | 
  17 |   <material name="black">
  18 |     <color rgba="0.2 0.2 0.2 1."/>
  19 |   </material>
  20 | 
  21 |   <material name="blue">
  22 |     <color rgba="0.3 0.57 1. 1."/>
  23 |   </material>
  24 | 
  25 |   <link name="base_link">
  26 |     <visual>
  27 |       <origin xyz="${wheelbase/2} 0 ${ground_offset+height/2}"/>
  28 |       <geometry>
  29 |         <box size="${wheelbase} ${width} ${height}"/>
  30 |       </geometry>
  31 |       <material name="blue"/>
  32 |     </visual>
  33 |   </link>
  34 | 
  35 |   <joint name="base_to_laser_model" type="fixed">
  36 |     <parent link="base_link"/>
  37 |     <child link="laser_model"/>
  38 |     <origin xyz="${laser_distance_from_base_link} 0 ${ground_offset+height+(laser_height/2)}"/>
  39 |   </joint>
  40 | 
  41 |   <link name="laser_model">
  42 |     <visual>
  43 |       <geometry>
  44 |         <cylinder radius="${laser_radius}" length="${laser_height}"/>
  45 |       </geometry>
  46 |       <material name="black"/>
  47 |     </visual>
  48 |   </link>
  49 | 
  50 |   <joint name="base_to_back_left_wheel" type="fixed">
  51 |     <parent link="base_link"/>
  52 |     <child link="back_left_wheel"/>
  53 |     <origin xyz="0 ${(wheel_length+width)/2} ${wheel_radius}"/>
  54 |   </joint>
  55 | 
  56 |   <link name="back_left_wheel">
  57 |     <visual>
  58 |       <geometry>
  59 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  60 |       </geometry>
  61 |       <material name="black"/>
  62 |       <origin rpy="${pi/2} 0 0"/>
  63 |     </visual>
  64 |   </link>
  65 | 
  66 |   <joint name="base_to_back_right_wheel" type="fixed">
  67 |     <parent link="base_link"/>
  68 |     <child link="back_right_wheel"/>
  69 |     <origin xyz="0 ${-(wheel_length+width)/2} ${wheel_radius}"/>
  70 |   </joint>
  71 | 
  72 |   <link name="back_right_wheel">
  73 |     <visual>
  74 |       <geometry>
  75 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  76 |       </geometry>
  77 |       <material name="black"/>
  78 |       <origin rpy="${pi/2} 0 0"/>
  79 |     </visual>
  80 |   </link>
  81 | 
  82 |   <joint name="base_to_front_left_hinge" type="fixed">
  83 |     <parent link="base_link"/>
  84 |     <child link="front_left_hinge"/>
  85 |     <origin xyz="${wheelbase} ${(wheel_length+width)/2} ${wheel_radius}"/>
  86 |   </joint>
  87 | 
  88 |   <link name="front_left_hinge"/>
  89 | 
  90 |   <joint name="front_left_hinge_to_wheel" type="continuous">
  91 |     <parent link="front_left_hinge"/>
  92 |     <child link="front_left_wheel"/>
  93 |   </joint>
  94 | 
  95 |   <link name="front_left_wheel">
  96 |     <visual>
  97 |       <geometry>
  98 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  99 |       </geometry>
 100 |       <material name="black"/>
 101 |       <origin rpy="${pi/2} 0 0"/>
 102 |     </visual>
 103 |   </link>
 104 | 
 105 |   <joint name="base_to_front_right_hinge" type="fixed">
 106 |     <parent link="base_link"/>
 107 |     <child link="front_right_hinge"/>
 108 |     <origin xyz="${wheelbase} ${-(wheel_length+width)/2} ${wheel_radius}"/>
 109 |   </joint>
 110 | 
 111 |   <link name="front_right_hinge"/>
 112 | 
 113 |   <joint name="front_right_hinge_to_wheel" type="continuous">
 114 |     <parent link="front_right_hinge"/>
 115 |     <child link="front_right_wheel"/>
 116 |   </joint>
 117 | 
 118 |   <link name="front_right_wheel">
 119 |     <visual>
 120 |       <geometry>
 121 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
 122 |       </geometry>
 123 |       <material name="black"/>
 124 |       <origin rpy="${pi/2} 0 0"/>
 125 |     </visual>
 126 |   </link>
 127 | 
 128 | </robot>

```

`src\racecar_model\launch\racecar_model.launch`:

```launch
   1 | <?xml version="1.0"?>
   2 | <launch>
   3 | 
   4 |   <!-- group for ego racecar -->
   5 |   <group ns="ego_racecar">
   6 |   <!-- Open the model file -->
   7 |   <arg name="racecar_xacro" default="$(find racecar_model)/ego_racecar.xacro"/>
   8 |   <param name="tf_prefix" value="ego_racecar"/>
   9 |   <param name="robot_description" command="xacro --inorder '$(arg racecar_xacro)'"/>
  10 |   </group>
  11 | 
  12 |   <!-- group for opponent racecar -->
  13 |   <group ns="opp_racecar">
  14 |   <!-- Open the model file -->
  15 |   <arg name="racecar_xacro" default="$(find racecar_model)/opp_racecar.xacro"/>
  16 |   <param name="tf_prefix" value="opp_racecar"/>
  17 |   <param name="robot_description" command="xacro --inorder '$(arg racecar_xacro)'"/>
  18 |   </group>
  19 | 
  20 | </launch>

```

`src\racecar_model\opp_racecar.xacro`:

```xacro
   1 | <?xml version="1.0"?>
   2 | 
   3 | <!-- A simple model of the racecar for rviz -->
   4 | 
   5 | <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="racecar">
   6 | 
   7 |   <xacro:property name="wheelbase" value="0.3302" />
   8 |   <xacro:property name="width" value="0.2032" />
   9 |   <xacro:property name="height" value="0.1" />
  10 |   <xacro:property name="ground_offset" value="0.04" />
  11 |   <xacro:property name="wheel_radius" value="0.0508" />
  12 |   <xacro:property name="wheel_length" value="0.0381" />
  13 |   <xacro:property name="laser_distance_from_base_link" value="0.275" />
  14 |   <xacro:property name="laser_height" value="0.05" />
  15 |   <xacro:property name="laser_radius" value="0.026" />
  16 | 
  17 |   <material name="black">
  18 |     <color rgba="0.2 0.2 0.2 1."/>
  19 |   </material>
  20 | 
  21 |   <material name="red">
  22 |     <color rgba="1. 0.57 0.1 1."/>
  23 |   </material>
  24 | 
  25 |   <link name="base_link">
  26 |     <visual>
  27 |       <origin xyz="${wheelbase/2} 0 ${ground_offset+height/2}"/>
  28 |       <geometry>
  29 |         <box size="${wheelbase} ${width} ${height}"/>
  30 |       </geometry>
  31 |       <material name="red"/>
  32 |     </visual>
  33 |   </link>
  34 | 
  35 |   <joint name="base_to_laser_model" type="fixed">
  36 |   <parent link="base_link"/>
  37 |   <child link="laser_model"/>
  38 |     <origin xyz="${laser_distance_from_base_link} 0 ${ground_offset+height+(laser_height/2)}"/>
  39 | </joint>
  40 | 
  41 |   <link name="laser_model">
  42 |     <visual>
  43 |       <geometry>
  44 |         <cylinder radius="${laser_radius}" length="${laser_height}"/>
  45 |       </geometry>
  46 |       <material name="black"/>
  47 |     </visual>
  48 |   </link>
  49 | 
  50 |   <joint name="base_to_back_left_wheel" type="fixed">
  51 |     <parent link="base_link"/>
  52 |     <child link="back_left_wheel"/>
  53 |     <origin xyz="0 ${(wheel_length+width)/2} ${wheel_radius}"/>
  54 |   </joint>
  55 | 
  56 |   <link name="back_left_wheel">
  57 |     <visual>
  58 |       <geometry>
  59 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  60 |       </geometry>
  61 |       <material name="black"/>
  62 |       <origin rpy="${pi/2} 0 0"/>
  63 |     </visual>
  64 |   </link>
  65 | 
  66 |   <joint name="base_to_back_right_wheel" type="fixed">
  67 |     <parent link="base_link"/>
  68 |     <child link="back_right_wheel"/>
  69 |     <origin xyz="0 ${-(wheel_length+width)/2} ${wheel_radius}"/>
  70 |   </joint>
  71 | 
  72 |   <link name="back_right_wheel">
  73 |     <visual>
  74 |       <geometry>
  75 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  76 |       </geometry>
  77 |       <material name="black"/>
  78 |       <origin rpy="${pi/2} 0 0"/>
  79 |     </visual>
  80 |   </link>
  81 | 
  82 |   <joint name="base_to_front_left_hinge" type="fixed">
  83 |     <parent link="base_link"/>
  84 |     <child link="front_left_hinge"/>
  85 |     <origin xyz="${wheelbase} ${(wheel_length+width)/2} ${wheel_radius}"/>
  86 |   </joint>
  87 | 
  88 |   <link name="front_left_hinge"/>
  89 | 
  90 |   <joint name="front_left_hinge_to_wheel" type="continuous">
  91 |     <parent link="front_left_hinge"/>
  92 |     <child link="front_left_wheel"/>
  93 |   </joint>
  94 | 
  95 |   <link name="front_left_wheel">
  96 |     <visual>
  97 |       <geometry>
  98 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  99 |       </geometry>
 100 |       <material name="black"/>
 101 |       <origin rpy="${pi/2} 0 0"/>
 102 |     </visual>
 103 |   </link>
 104 | 
 105 |   <joint name="base_to_front_right_hinge" type="fixed">
 106 |     <parent link="base_link"/>
 107 |     <child link="front_right_hinge"/>
 108 |     <origin xyz="${wheelbase} ${-(wheel_length+width)/2} ${wheel_radius}"/>
 109 |   </joint>
 110 | 
 111 |   <link name="front_right_hinge"/>
 112 | 
 113 |   <joint name="front_right_hinge_to_wheel" type="continuous">
 114 |     <parent link="front_right_hinge"/>
 115 |     <child link="front_right_wheel"/>
 116 |   </joint>
 117 | 
 118 |   <link name="front_right_wheel">
 119 |     <visual>
 120 |       <geometry>
 121 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
 122 |       </geometry>
 123 |       <material name="black"/>
 124 |       <origin rpy="${pi/2} 0 0"/>
 125 |     </visual>
 126 |   </link>
 127 | 
 128 | </robot>

```

`src\racecar_model\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>racecar_model</name>
   4 |   <version>0.0.0</version>
   5 |   <description>The racecar_model package</description>
   6 | 
   7 |   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8 |   <!-- Example:  -->
   9 |   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 |   <maintainer email="mizuho@todo.todo">mizuho</maintainer>
  11 | 
  12 | 
  13 |   <!-- One license tag required, multiple allowed, one license per tag -->
  14 |   <!-- Commonly used license strings: -->
  15 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  16 |   <license>TODO</license>
  17 | 
  18 | 
  19 |   <!-- Url tags are optional, but multiple are allowed, one per tag -->
  20 |   <!-- Optional attribute type can be: website, bugtracker, or repository -->
  21 |   <!-- Example: -->
  22 |   <!-- <url type="website">http://wiki.ros.org/racecar_model</url> -->
  23 | 
  24 | 
  25 |   <!-- Author tags are optional, multiple are allowed, one per tag -->
  26 |   <!-- Authors do not have to be maintainers, but could be -->
  27 |   <!-- Example: -->
  28 |   <!-- <author email="jane.doe@example.com">Jane Doe</author> -->
  29 | 
  30 | 
  31 |   <!-- The *depend tags are used to specify dependencies -->
  32 |   <!-- Dependencies can be catkin packages or system dependencies -->
  33 |   <!-- Examples: -->
  34 |   <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  35 |   <!--   <depend>roscpp</depend> -->
  36 |   <!--   Note that this is equivalent to the following: -->
  37 |   <!--   <build_depend>roscpp</build_depend> -->
  38 |   <!--   <exec_depend>roscpp</exec_depend> -->
  39 |   <!-- Use build_depend for packages you need at compile time: -->
  40 |   <!--   <build_depend>message_generation</build_depend> -->
  41 |   <!-- Use build_export_depend for packages you need in order to build against this package: -->
  42 |   <!--   <build_export_depend>message_generation</build_export_depend> -->
  43 |   <!-- Use buildtool_depend for build tool packages: -->
  44 |   <!--   <buildtool_depend>catkin</buildtool_depend> -->
  45 |   <!-- Use exec_depend for packages you need at runtime: -->
  46 |   <!--   <exec_depend>message_runtime</exec_depend> -->
  47 |   <!-- Use test_depend for packages you need only for testing: -->
  48 |   <!--   <test_depend>gtest</test_depend> -->
  49 |   <!-- Use doc_depend for packages you need only for building documentation: -->
  50 |   <!--   <doc_depend>doxygen</doc_depend> -->
  51 |   <buildtool_depend>catkin</buildtool_depend>
  52 | 
  53 | 
  54 |   <!-- The export tag contains other, unspecified, tags -->
  55 |   <export>
  56 |     <!-- Other tools can request additional information be placed here -->
  57 | 
  58 |   </export>
  59 | </package>

```

`src\reference_sdf_generator\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.13)
   2 | project(reference_sdf_generator)
   3 | 
   4 | add_compile_options(-std=c++17)
   5 | add_compile_options(-Wall -Wextra)
   6 | add_compile_options(-O3)
   7 | add_compile_options(-O3 -fopenmp)
   8 | # set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
   9 | if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  10 |   set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  11 |   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  12 | endif()
  13 | 
  14 | ## Find catkin macros and libraries
  15 | ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  16 | ## is used, also find other catkin packages
  17 | find_package(catkin REQUIRED COMPONENTS
  18 |   geometry_msgs
  19 |   std_msgs
  20 |   nav_msgs
  21 |   roscpp
  22 |   tf2_geometry_msgs
  23 |   tf2_ros
  24 |   grid_map_core
  25 |   grid_map_ros
  26 |   grid_map_filters
  27 |   grid_map_loader
  28 |   grid_map_msgs
  29 |   grid_map_rviz_plugin
  30 |   grid_map_visualization
  31 |   waypoint_msgs
  32 | )
  33 | 
  34 | find_package(Eigen3 REQUIRED)
  35 | 
  36 | # For OpenMP
  37 | # set(OpenMP_HOME "/usr/lib/llvm-10")
  38 | # set(OpenMP_omp_LIBRARY "${OpenMP_HOME}/lib/")
  39 | # set(OpenMP_C_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
  40 | # set(OpenMP_CXX_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
  41 | # set(OpenMP_C_LIB_NAMES "omp")
  42 | # set(OpenMP_CXX_LIB_NAMES "omp")
  43 | find_package(OpenMP REQUIRED)
  44 | if(OpenMP_FOUND)
  45 |     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  46 |     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  47 | endif()
  48 | 
  49 | catkin_package(
  50 |  INCLUDE_DIRS include
  51 | #  LIBRARIES mppi
  52 | #  CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros
  53 | #  DEPENDS system_lib
  54 | )
  55 | 
  56 | ###########
  57 | ## Build ##
  58 | ###########
  59 | add_executable(${PROJECT_NAME}_node
  60 |   src/reference_sdf_generator_node.cpp
  61 |   src/reference_sdf_generator.cpp
  62 | )
  63 | 
  64 | ## Specify additional locations of header files
  65 | ## Your package locations should be listed before other locations
  66 | 
  67 | target_include_directories(${PROJECT_NAME}_node PUBLIC
  68 |   include
  69 |   ${catkin_INCLUDE_DIRS}
  70 | )
  71 | 
  72 | ## Add cmake target dependencies of the executable
  73 | ## same as for the library above
  74 | add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  75 | 
  76 | ## Specify libraries to link a library or executable target against
  77 | target_link_libraries(${PROJECT_NAME}_node
  78 |   ${catkin_LIBRARIES}
  79 |  )
  80 | if (OPENMP_FOUND)
  81 |     if (TARGET OpenMP::OpenMP_CXX)
  82 |         target_link_libraries(${PROJECT_NAME}_node OpenMP::OpenMP_CXX)
  83 |     endif ()
  84 | endif ()
  85 | 
  86 |  install(
  87 |   TARGETS
  88 |     ${PROJECT_NAME}_node
  89 |   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  90 |   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  91 |   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  92 | )
  93 | 
  94 | install(
  95 |   DIRECTORY
  96 |     launch
  97 |     config
  98 |   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  99 | )

```

`src\reference_sdf_generator\README.md`:

```md
   1 | # reference_sdf_generator

```

`src\reference_sdf_generator\config\reference_sdf_generator.yaml`:

```yaml
   1 | out_sdf_topic: reference_sdf
   2 | in_waypoints_topic: reference_waypoint
   3 | robot_frame_id: ego_racecar/base_link
   4 | map_frame_id: map
   5 | backward_point_topic: backward_point
   6 | 
   7 | update_rate: 0.025 # Rate of publishing the reference sdf [s]
   8 | thread_num: 2 # Number of threads for parallel processing
   9 | ref_path_map_resolution: 0.1 # Resolution of the reference sdf grid map
  10 | ref_path_map_width: 20.0 # Width of the reference sdf grid map
  11 | ref_path_map_height: 20.0 # Height of the reference sdf grid map
  12 | 
  13 | num_waypoints: 80 # Number of used waypoints from robot current position
  14 | backward_margin_num: 0 # margin for switchback
  15 | waypoint_interval: 0.1 # Distance between two waypoints [m]
  16 | reference_speed_scale: 1.0 # Scale of the reference speed [0. inf)
  17 | max_speed: 100.0 # Maximum speed for clipping [m/s]
  18 | # The following is for reducing the size of the map
  19 | # But it is not used in the current version
  20 | # submap_center_ahead: 0.0
  21 | # submap_length: 5.0
  22 | # submap_width: 5.0

```

`src\reference_sdf_generator\include\reference_sdf_generator\reference_sdf_generator.hpp`:

```hpp
   1 | // Kohei Honda, 2023
   2 | 
   3 | #pragma once
   4 | #include <Eigen/Dense>
   5 | #include <string>
   6 | #include <vector>
   7 | 
   8 | #include <geometry_msgs/PoseStamped.h>
   9 | #include <geometry_msgs/TransformStamped.h>
  10 | #include <nav_msgs/Path.h>
  11 | #include <ros/ros.h>
  12 | #include <std_msgs/Float32.h>
  13 | #include <tf2/utils.h>
  14 | #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
  15 | #include <tf2_ros/transform_broadcaster.h>
  16 | #include <tf2_ros/transform_listener.h>
  17 | #include <visualization_msgs/Marker.h>
  18 | #include <visualization_msgs/MarkerArray.h>
  19 | #include <grid_map_core/GridMap.hpp>
  20 | #include <grid_map_core/iterators/GridMapIterator.hpp>
  21 | #include <grid_map_ros/grid_map_ros.hpp>
  22 | 
  23 | // custom message
  24 | #include <waypoint_msgs/Waypoint.h>
  25 | 
  26 | namespace reference_sdf_generator {
  27 | 
  28 | class ReferenceSDFGenerator {
  29 | public:
  30 |     ReferenceSDFGenerator();
  31 |     ~ReferenceSDFGenerator(){};
  32 | 
  33 | private:
  34 |     struct RobotState {
  35 |         double x = 0.0;
  36 |         double y = 0.0;
  37 |         double yaw = 0.0;
  38 | 
  39 |         // use only waypoints
  40 |         double vel = 0.0;
  41 |     };
  42 |     using Waypoints = std::vector<RobotState>;
  43 | 
  44 | private:
  45 |     /* ros system variables */
  46 |     ros::NodeHandle nh_;          //!< @brief ros public node handle
  47 |     ros::NodeHandle private_nh_;  //!< @brief ros private node handle
  48 |     tf2_ros::Buffer tf_buffer_;
  49 |     tf2_ros::TransformListener tf_listener_;
  50 | 
  51 |     /* pub sub */
  52 |     ros::Subscriber sub_ref_path_;       //!< @brief reference path subscriber
  53 |     ros::Timer timer_control_;           //!< @brief timer for control command commutation
  54 |     ros::Publisher pub_reference_sdf_;   //!< @brief distance field topic publisher
  55 |     ros::Publisher pub_backward_point_;  //!< @brief backward point topic publisher for switchback
  56 | 
  57 |     // debug
  58 |     ros::Publisher pub_waypoints_;  //!< @brief waypoints topic publisher (for debug)
  59 |     // ros::Publisher pub_calc_time_;
  60 | 
  61 |     /*Parameters*/
  62 |     std::string robot_frame_id_;
  63 |     std::string map_frame_id_;
  64 |     double update_rate_;  //!< @brief update interval [s]
  65 |     int thread_num_ = 4;
  66 |     const std::string distance_field_layer_name_ = "distance_field";
  67 |     const std::string angle_field_layer_name_ = "angle_field";
  68 |     const std::string speed_field_layer_name_ = "speed_field";
  69 | 
  70 |     // waypoints for path tracking parameters
  71 |     int backward_margin_num_ = 5;
  72 |     int num_waypoints_ = 30;
  73 |     double waypoint_interval_ = 0.1;        //!< @brief interval between waypoints [m]
  74 |     double ref_path_map_resolution_ = 0.1;  //!< @brief resolution of distance field [m]
  75 |     double ref_path_map_width_ = 50.0;      //!< @brief width of distance field [m]
  76 |     double ref_path_map_height_ = 50.0;     //!< @brief height of distance field [m]
  77 |     double reference_speed_scale_ = 1.0;    //!< @brief scale of reference speed
  78 |     double max_speed_ = 100.0;              //!< @brief maximum speed [m/s]
  79 |     // The submap size is important for calculation cost
  80 |     // double submap_center_ahead_ = 3.0; //!< @brief distance from robot to submap center [m]
  81 |     // double submap_length_ = 15.0;      //!< @brief width of submap [m]
  82 |     // double submap_width_ = 15.0;       //!< @brief height of submap [m]
  83 | 
  84 |     // Inner variables
  85 |     waypoint_msgs::Waypoint waypoints_msg_;
  86 | 
  87 |     bool is_waypoints_ok_ = false;
  88 |     grid_map::GridMap reference_sdf_;  // value = [0, inf) (distance field layer), value = [-pi/2, pi/2] (angle layer)
  89 | 
  90 |     /**
  91 |      * @brief main loop
  92 |      *
  93 |      */
  94 |     void timer_callback(const ros::TimerEvent&);
  95 | 
  96 |     void callback_waypoints(const waypoint_msgs::Waypoint& waypoint_msg);
  97 | 
  98 |     template <typename T1, typename T2>
  99 |     double distance(T1 pt1, T2 pt2) const {
 100 |         return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
 101 |     }
 102 | 
 103 |     int find_nearest_index(const std::vector<geometry_msgs::PoseStamped>& path, const RobotState& pose) const;
 104 | 
 105 |     int find_lookahead_index(const std::vector<geometry_msgs::PoseStamped>& path, const int& nearest_index, const double& lookahead_dist) const;
 106 | 
 107 |     Waypoints calc_waypoints(const waypoint_msgs::Waypoint& waypoints, const RobotState& current_state) const;
 108 | 
 109 |     void build_reference_sdf(const Waypoints& waypoints, const RobotState& current_state, grid_map::GridMap* distance_field) const;
 110 | 
 111 |     void publish_waypoints(const Waypoints& waypoints) const;
 112 | };
 113 | 
 114 | }  // namespace reference_sdf_generator

```

`src\reference_sdf_generator\launch\reference_sdf_generator.launch`:

```launch
   1 | <launch>
   2 |     <arg name="param_path" default="$(find reference_sdf_generator)/config/reference_sdf_generator.yaml"/>
   3 | 
   4 |     <node pkg="reference_sdf_generator" type="reference_sdf_generator_node" name="reference_sdf_generator" output="screen">
   5 |         <rosparam command="load" file="$(arg param_path)"/>
   6 |     </node>
   7 | </launch>

```

`src\reference_sdf_generator\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>reference_sdf_generator</name>
   4 |   <version>0.0.0</version>
   5 |   <description>Model Predictive Path Integral control</description>
   6 | 
   7 |   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8 |   <!-- Example:  -->
   9 |   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 |   <maintainer email="0905honda@gmail.com">Kohei Honda</maintainer>
  11 | 
  12 | 
  13 |   <!-- One license tag required, multiple allowed, one license per tag -->
  14 |   <!-- Commonly used license strings: -->
  15 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  16 |   <license>MIT</license>
  17 | 
  18 |   <buildtool_depend>catkin</buildtool_depend>
  19 |   <build_depend>geometry_msgs</build_depend>
  20 |   <build_depend>nav_msgs</build_depend>
  21 |   <build_depend>roscpp</build_depend>
  22 |   <build_depend>tf2_geometry_msgs</build_depend>
  23 |   <build_depend>tf2_ros</build_depend>
  24 |   <build_depend>grid_map</build_depend>
  25 |   <build_depend>grid_map_ros</build_depend>
  26 |   <build_depend>grid_map_msgs</build_depend>
  27 |   <build_depend>grid_map_rviz_plugin</build_depend>
  28 |   <build_depend>grid_map_visualization</build_depend>
  29 |   <build_depend>waypoint_msgs</build_depend>
  30 | 
  31 |   <build_export_depend>geometry_msgs</build_export_depend>
  32 |   <build_export_depend>nav_msgs</build_export_depend>
  33 |   <build_export_depend>roscpp</build_export_depend>
  34 |   <build_export_depend>tf2_geometry_msgs</build_export_depend>
  35 |   <build_export_depend>tf2_ros</build_export_depend>
  36 |   <build_export_depend>grid_map</build_export_depend>
  37 |   <build_export_depend>grid_map_ros</build_export_depend>
  38 |   <build_export_depend>grid_map_msgs</build_export_depend>
  39 |   <build_export_depend>grid_map_rviz_plugin</build_export_depend>
  40 |   <build_export_depend>grid_map_visualization</build_export_depend>
  41 |   <build_export_depend>waypoint_msgs</build_export_depend>
  42 |   
  43 |   <exec_depend>geometry_msgs</exec_depend>
  44 |   <exec_depend>nav_msgs</exec_depend>
  45 |   <exec_depend>roscpp</exec_depend>
  46 |   <exec_depend>tf2_geometry_msgs</exec_depend>
  47 |   <exec_depend>tf2_ros</exec_depend>
  48 |   <exec_depend>grid_map</exec_depend>
  49 |   <exec_depend>grid_map_ros</exec_depend>
  50 |   <exec_depend>grid_map_msgs</exec_depend>
  51 |   <exec_depend>grid_map_rviz_plugin</exec_depend>
  52 |   <exec_depend>grid_map_visualization</exec_depend>
  53 |   <exec_depend>waypoint_msgs</exec_depend>
  54 | 
  55 |   <!-- The export tag contains other, unspecified, tags -->
  56 |   <export>
  57 |     <!-- Other tools can request additional information be placed here -->
  58 | 
  59 |   </export>
  60 | </package>

```

`src\reference_sdf_generator\src\reference_sdf_generator.cpp`:

```cpp
   1 | #include "reference_sdf_generator/reference_sdf_generator.hpp"
   2 | 
   3 | namespace reference_sdf_generator {
   4 | 
   5 | ReferenceSDFGenerator::ReferenceSDFGenerator()
   6 |     : nh_(""),
   7 |       private_nh_("~"),
   8 |       tf_listener_(tf_buffer_),
   9 |       reference_sdf_(std::vector<std::string>({distance_field_layer_name_, angle_field_layer_name_, speed_field_layer_name_})) {
  10 |     // set parameters from ros parameter server
  11 |     private_nh_.param("update_rate", update_rate_, 0.05);
  12 |     private_nh_.param("thread_num", thread_num_, 4);
  13 |     private_nh_.param("num_waypoints", num_waypoints_, 30);
  14 |     private_nh_.param("backward_margin_num", backward_margin_num_, 5);
  15 |     private_nh_.param("waypoint_interval", waypoint_interval_, 0.1);
  16 |     private_nh_.param("ref_path_map_resolution", ref_path_map_resolution_, 0.1);
  17 |     private_nh_.param("ref_path_map_width", ref_path_map_width_, 50.0);
  18 |     private_nh_.param("ref_path_map_height", ref_path_map_height_, 50.0);
  19 |     private_nh_.param("max_speed", max_speed_, 100.0);
  20 |     private_nh_.param("reference_speed_scale", reference_speed_scale_, 1.0);
  21 |     // private_nh_.param("submap_center_ahead", submap_center_ahead_, 3.0);
  22 |     // private_nh_.param("submap_length", submap_length_, 15.0);
  23 |     // private_nh_.param("submap_width", submap_width_, 15.0);
  24 | 
  25 |     std::string out_sdf_topic;
  26 |     std::string in_waypoints_topic;
  27 |     std::string out_backward_point_topic;
  28 | 
  29 |     private_nh_.param("in_waypoints_topic", in_waypoints_topic, static_cast<std::string>("reference_waypoint"));
  30 |     private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
  31 |     private_nh_.param("map_frame_id", map_frame_id_, static_cast<std::string>("map"));
  32 |     private_nh_.param("out_sdf_topic", out_sdf_topic, static_cast<std::string>("reference_sdf"));
  33 |     private_nh_.param("backward_point_topic", out_backward_point_topic, static_cast<std::string>("backward_point"));
  34 | 
  35 |     // initialize reference path grid map
  36 |     reference_sdf_.setFrameId(map_frame_id_);
  37 |     reference_sdf_.setGeometry(grid_map::Length(ref_path_map_width_, ref_path_map_height_), ref_path_map_resolution_, grid_map::Position(0.0, 0.0));
  38 | 
  39 |     // set publishers and subscribers
  40 |     sub_ref_path_ = nh_.subscribe(in_waypoints_topic, 1, &ReferenceSDFGenerator::callback_waypoints, this);
  41 |     timer_control_ = nh_.createTimer(ros::Duration(update_rate_), &ReferenceSDFGenerator::timer_callback, this);
  42 |     pub_reference_sdf_ = nh_.advertise<grid_map_msgs::GridMap>(out_sdf_topic, 1, true);
  43 |     pub_backward_point_ = nh_.advertise<geometry_msgs::PointStamped>(out_backward_point_topic, 1, true);
  44 | 
  45 |     // For debug
  46 |     pub_waypoints_ = nh_.advertise<visualization_msgs::MarkerArray>("reference_sdf/waypoints", 1, true);
  47 |     // pub_calc_time_ = nh_.advertise<std_msgs::Float32>("reference_sdf/calc_time", 1, true);
  48 | }
  49 | 
  50 | void ReferenceSDFGenerator::callback_waypoints(const waypoint_msgs::Waypoint& waypoints_msg) {
  51 |     if (waypoints_msg.poses.size() == 0) {
  52 |         ROS_WARN_THROTTLE(5.0, "[ReferenceSDFGenerator]Received waypoints is empty");
  53 |         waypoints_msg_.poses.clear();
  54 |         waypoints_msg_.twists.clear();
  55 |         return;
  56 |     } else {
  57 |         ROS_INFO("[ReferenceSDFGenerator]New waypoints is received.");
  58 |     }
  59 |     waypoints_msg_ = waypoints_msg;
  60 |     is_waypoints_ok_ = true;
  61 | }
  62 | 
  63 | // main loop
  64 | void ReferenceSDFGenerator::timer_callback([[maybe_unused]] const ros::TimerEvent& te) {
  65 |     /* Status check */
  66 |     if (!is_waypoints_ok_) {
  67 |         ROS_WARN_THROTTLE(5.0, "[ReferenceSDFGenerator] path is not ready");
  68 |         return;
  69 |     }
  70 | 
  71 |     // get current robot state
  72 |     geometry_msgs::TransformStamped trans_form_stamped;
  73 |     try {
  74 |         trans_form_stamped = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
  75 |     } catch (const tf2::TransformException& ex) {
  76 |         ROS_WARN_THROTTLE(3.0, "[ReferenceSDFGenerator] %s", ex.what());
  77 |         return;
  78 |     };
  79 |     RobotState robot_state;
  80 |     robot_state.x = trans_form_stamped.transform.translation.x;
  81 |     robot_state.y = trans_form_stamped.transform.translation.y;
  82 |     const double _yaw = tf2::getYaw(trans_form_stamped.transform.rotation);
  83 |     robot_state.yaw = std::atan2(std::sin(_yaw), std::cos(_yaw));
  84 | 
  85 |     // Calculate waypoints
  86 |     const auto waypoints = calc_waypoints(waypoints_msg_, robot_state);
  87 | 
  88 |     // ======== time measurement ========
  89 |     // const auto start_time = std::chrono::system_clock::now();
  90 | 
  91 |     // Calculate reference SDF
  92 |     build_reference_sdf(waypoints, robot_state, &reference_sdf_);
  93 | 
  94 |     // ======== time measurement ========
  95 |     // const auto end_time = std::chrono::system_clock::now();
  96 |     // const auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000000.0;
  97 |     // std_msgs::Float32 calc_time_msg;
  98 |     // calc_time_msg.data = elapsed_time;
  99 |     // pub_calc_time_.publish(calc_time_msg);
 100 | 
 101 |     // Publish reference SDF
 102 |     grid_map_msgs::GridMap message;
 103 |     grid_map::GridMapRosConverter::toMessage(reference_sdf_, message);
 104 |     message.info.header.stamp = ros::Time::now();
 105 |     pub_reference_sdf_.publish(message);
 106 | 
 107 |     // Publish backward point
 108 |     geometry_msgs::PointStamped backward_point_msg;
 109 |     backward_point_msg.header.frame_id = map_frame_id_;
 110 |     backward_point_msg.header.stamp = ros::Time::now();
 111 |     backward_point_msg.point.x = waypoints[0].x;
 112 |     backward_point_msg.point.y = waypoints[0].y;
 113 |     backward_point_msg.point.z = 0;
 114 |     pub_backward_point_.publish(backward_point_msg);
 115 | 
 116 |     // Publish waypoints for debug
 117 |     publish_waypoints(waypoints);
 118 | }
 119 | 
 120 | int ReferenceSDFGenerator::find_nearest_index(const std::vector<geometry_msgs::PoseStamped>& path, const RobotState& current_state) const {
 121 |     int nearest_idx = 0;
 122 |     double min_dist = std::numeric_limits<double>::max();
 123 |     geometry_msgs::Point pos;
 124 |     for (size_t i = 0; i < path.size(); i++) {
 125 |         const double dist = distance(path[i].pose.position, current_state);
 126 |         if (dist < min_dist) {
 127 |             min_dist = dist;
 128 |             nearest_idx = i;
 129 |         }
 130 |     }
 131 |     return nearest_idx;
 132 | }
 133 | 
 134 | int ReferenceSDFGenerator::find_lookahead_index(const std::vector<geometry_msgs::PoseStamped>& path,
 135 |                                                 const int& nearest_index,
 136 |                                                 const double& lookahead_dist) const {
 137 |     // Find target waypoint index with loop path
 138 |     int target_waypoint_idx_ = nearest_index;
 139 |     bool is_found_target_waypoint_ = false;
 140 |     int index = nearest_index;
 141 |     while (!is_found_target_waypoint_) {
 142 |         index = (index + 1) % path.size();
 143 |         const double dist = distance(path[index].pose.position, path[nearest_index].pose.position);
 144 |         if (dist >= lookahead_dist) {
 145 |             target_waypoint_idx_ = index;
 146 |             is_found_target_waypoint_ = true;
 147 |         }
 148 |     }
 149 | 
 150 |     return target_waypoint_idx_;
 151 | }
 152 | 
 153 | ReferenceSDFGenerator::Waypoints ReferenceSDFGenerator::calc_waypoints(const waypoint_msgs::Waypoint& waypoints_msg,
 154 |                                                                        const RobotState& current_state) const {
 155 |     Waypoints waypoints;
 156 | 
 157 |     auto get_waypoint = [](const waypoint_msgs::Waypoint& waypoints_msgs, const int& index) {
 158 |         RobotState waypoint;
 159 |         waypoint.x = waypoints_msgs.poses[index].pose.position.x;
 160 |         waypoint.y = waypoints_msgs.poses[index].pose.position.y;
 161 | 
 162 |         // calculate yaw angle
 163 |         int next_index = 0;
 164 |         if (index == static_cast<int>(waypoints_msgs.poses.size()) - 1) {
 165 |             next_index = index - 1;
 166 |             waypoint.yaw = std::atan2(waypoints_msgs.poses[index].pose.position.y - waypoints_msgs.poses[next_index].pose.position.y,
 167 |                                       waypoints_msgs.poses[index].pose.position.x - waypoints_msgs.poses[next_index].pose.position.x);
 168 |         } else {
 169 |             next_index = index + 1;
 170 |             waypoint.yaw = std::atan2(waypoints_msgs.poses[next_index].pose.position.y - waypoints_msgs.poses[index].pose.position.y,
 171 |                                       waypoints_msgs.poses[next_index].pose.position.x - waypoints_msgs.poses[index].pose.position.x);
 172 |         }
 173 | 
 174 |         // vel
 175 |         waypoint.vel = waypoints_msgs.twists[index].twist.linear.x;
 176 | 
 177 |         return waypoint;
 178 |     };
 179 | 
 180 |     // find nearest point
 181 |     const int nearest_idx = find_nearest_index(waypoints_msg.poses, current_state);
 182 | 
 183 |     const int nearest_idx_with_margin = std::max(nearest_idx - backward_margin_num_, 0);
 184 |     const auto waypoint = get_waypoint(waypoints_msg, nearest_idx_with_margin);
 185 |     waypoints.push_back(waypoint);
 186 | 
 187 |     // find waypoints with lookahead distance
 188 |     int current_waypoint_idx = nearest_idx_with_margin;
 189 |     for (int i = 1; i < num_waypoints_; i++) {
 190 |         const int target_waypoint_idx = find_lookahead_index(waypoints_msg.poses, current_waypoint_idx, waypoint_interval_);
 191 | 
 192 |         const auto waypoint = get_waypoint(waypoints_msg, target_waypoint_idx);
 193 |         waypoints.push_back(waypoint);
 194 |         current_waypoint_idx = target_waypoint_idx;
 195 |     }
 196 | 
 197 |     return waypoints;
 198 | }
 199 | 
 200 | void ReferenceSDFGenerator::build_reference_sdf(const Waypoints& waypoints, const RobotState& current_state, grid_map::GridMap* reference_sdf) const {
 201 |     reference_sdf->setGeometry(grid_map::Length(ref_path_map_width_, ref_path_map_height_), ref_path_map_resolution_,
 202 |                                grid_map::Position(current_state.x, current_state.y));
 203 | 
 204 |     // Ellipse submap setting
 205 |     // const double ahead_distance = submap_center_ahead_;
 206 |     // const grid_map::Position ellipse_center(current_state.x + ahead_distance * cos(current_state.yaw), current_state.y + ahead_distance *
 207 |     // sin(current_state.yaw)); const grid_map::Length length(submap_length_, submap_width_);
 208 | 
 209 |     // iterate all cells for calculating distance field
 210 |     grid_map::Matrix& dist_data = reference_sdf->get(distance_field_layer_name_);
 211 |     grid_map::Matrix& angle_data = reference_sdf->get(angle_field_layer_name_);
 212 |     grid_map::Matrix& speed_data = reference_sdf->get(speed_field_layer_name_);
 213 | 
 214 |     const auto grid_map_size = reference_sdf->getSize();
 215 |     const unsigned int linear_grid_size = grid_map_size.prod();
 216 | 
 217 |     // The following are provided normal iterators, but cannot used open mp
 218 |     // for (grid_map::EllipseIterator iterator(*reference_sdf, ellipse_center, length, current_state.yaw); !iterator.isPastEnd(); ++iterator)
 219 |     // for (grid_map::GridMapIterator iterator(*reference_sdf); !iterator.isPastEnd(); ++iterator)
 220 | 
 221 | #pragma omp parallel for num_threads(thread_num_)
 222 |     for (unsigned int i = 0; i < linear_grid_size; ++i) {
 223 |         // const grid_map::Index index(*iterator);
 224 |         const grid_map::Index index(grid_map::getIndexFromLinearIndex(i, grid_map_size));
 225 | 
 226 |         grid_map::Position position;
 227 |         reference_sdf->getPosition(index, position);
 228 | 
 229 |         // calc distance to nearest waypoint
 230 |         double min_dist = std::numeric_limits<double>::max();
 231 |         int nearest_waypoint_idx = 0;
 232 |         for (size_t i = 0; i < waypoints.size(); i++) {
 233 |             const double dist = sqrt((waypoints[i].x - position[0]) * (waypoints[i].x - position[0]) +
 234 |                                      (waypoints[i].y - position[1]) * (waypoints[i].y - position[1]));
 235 |             if (dist < min_dist) {
 236 |                 min_dist = dist;
 237 |                 nearest_waypoint_idx = i;
 238 |             }
 239 |         }
 240 | 
 241 |         // set each field value
 242 |         dist_data(index[0], index[1]) = min_dist;
 243 |         angle_data(index[0], index[1]) = waypoints[nearest_waypoint_idx].yaw;
 244 |         speed_data(index[0], index[1]) = std::min(waypoints[nearest_waypoint_idx].vel * reference_speed_scale_, max_speed_);
 245 |     }
 246 | }
 247 | 
 248 | void ReferenceSDFGenerator::publish_waypoints(const Waypoints& waypoints) const {
 249 |     visualization_msgs::MarkerArray marker_array;
 250 |     visualization_msgs::Marker arrow;
 251 |     arrow.header.frame_id = map_frame_id_;
 252 |     arrow.header.stamp = ros::Time::now();
 253 |     arrow.ns = "waypoints";
 254 |     arrow.type = visualization_msgs::Marker::ARROW;
 255 |     arrow.action = visualization_msgs::Marker::ADD;
 256 |     geometry_msgs::Vector3 arrow_scale;
 257 |     arrow_scale.x = 0.02;
 258 |     arrow_scale.y = 0.04;
 259 |     arrow_scale.z = 0.1;
 260 |     arrow.scale = arrow_scale;
 261 |     arrow.pose.position.x = 0.0;
 262 |     arrow.pose.position.y = 0.0;
 263 |     arrow.pose.position.z = 0.0;
 264 |     arrow.pose.orientation.x = 0.0;
 265 |     arrow.pose.orientation.y = 0.0;
 266 |     arrow.pose.orientation.z = 0.0;
 267 |     arrow.pose.orientation.w = 1.0;
 268 |     arrow.color.a = 1.0;
 269 |     arrow.color.r = 0.0;
 270 |     arrow.color.g = 1.0;
 271 |     arrow.color.b = 0.0;
 272 |     arrow.lifetime = ros::Duration(0.1);
 273 |     arrow.points.resize(2);
 274 | 
 275 |     for (size_t i = 0; i < waypoints.size(); i++) {
 276 |         arrow.id = i;
 277 |         const double length = 0.3;
 278 |         geometry_msgs::Point start;
 279 |         start.x = waypoints[i].x;
 280 |         start.y = waypoints[i].y;
 281 |         start.z = 0.1;
 282 | 
 283 |         geometry_msgs::Point end;
 284 |         end.x = waypoints[i].x + length * cos(waypoints[i].yaw);
 285 |         end.y = waypoints[i].y + length * sin(waypoints[i].yaw);
 286 |         end.z = 0.1;
 287 | 
 288 |         arrow.points[0] = start;
 289 |         arrow.points[1] = end;
 290 | 
 291 |         marker_array.markers.push_back(arrow);
 292 |     }
 293 | 
 294 |     pub_waypoints_.publish(marker_array);
 295 | }
 296 | 
 297 | }  // namespace reference_sdf_generator

```

`src\reference_sdf_generator\src\reference_sdf_generator_node.cpp`:

```cpp
   1 | #include "reference_sdf_generator/reference_sdf_generator.hpp"
   2 | 
   3 | int main(int argc, char **argv)
   4 | {
   5 |     ros::init(argc, argv, "reference_sdf_generator");
   6 |     reference_sdf_generator::ReferenceSDFGenerator reference_sdf_generator;
   7 |     ros::spin();
   8 |     return 0;
   9 | };

```

`src\reference_waypoint_loader\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 3.13)
   2 | project(reference_waypoint_loader)
   3 | 
   4 | add_compile_options(-std=c++17)
   5 | # set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
   6 | if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
   7 |   set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
   8 |   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
   9 | endif()
  10 | 
  11 | #### Options
  12 | option(BUILD_TESTS "Build test" OFF)
  13 | option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
  14 | 
  15 | find_package(catkin REQUIRED COMPONENTS 
  16 |   tf2_ros 
  17 |   roslib 
  18 |   rosbag
  19 |   waypoint_msgs
  20 |   nav_msgs
  21 |   visualization_msgs
  22 | )
  23 | 
  24 | ###################################
  25 | ## catkin specific configuration ##
  26 | ###################################
  27 | catkin_package(
  28 | #  INCLUDE_DIRS include
  29 | #  LIBRARIES reference_waypoint_loader
  30 | #  CATKIN_DEPENDS other_catkin_pkg
  31 | #  DEPENDS system_lib
  32 | )
  33 | 
  34 | ###########
  35 | ## Build ##
  36 | ###########
  37 | 
  38 | add_executable(reference_waypoint_loader_node
  39 |   src/reference_waypoint_loader.cpp
  40 |   src/reference_waypoint_loader_node.cpp
  41 | )
  42 | 
  43 | add_dependencies(reference_waypoint_loader_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  44 | 
  45 | target_include_directories(reference_waypoint_loader_node PUBLIC
  46 |   include
  47 |   ${catkin_INCLUDE_DIRS}
  48 | )
  49 | 
  50 | target_link_libraries(reference_waypoint_loader_node
  51 |   ${catkin_LIBRARIES}
  52 | )
  53 | 
  54 | install(
  55 |   TARGETS
  56 |     reference_waypoint_loader_node
  57 |   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  58 |   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  59 |   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  60 | )
  61 | 
  62 | install(
  63 |   DIRECTORY
  64 |     launch
  65 |   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  66 | )

```

`src\reference_waypoint_loader\README.md`:

```md
   1 | # Reference waypoint loader
   2 | 
   3 | - 開発のためにとりあえずreference pathをcsvからloadして流すだけ
   4 | - -> added reference velocity information

```

`src\reference_waypoint_loader\include\reference_waypoint_loader\rapidcsv.h`:

```h
   1 | /*
   2 |  * rapidcsv.h
   3 |  *
   4 |  * URL:      https://github.com/d99kris/rapidcsv
   5 |  * Version:  8.49
   6 |  *
   7 |  * Copyright (C) 2017-2021 Kristofer Berggren
   8 |  * All rights reserved.
   9 |  *
  10 |  * rapidcsv is distributed under the BSD 3-Clause license, see LICENSE for details.
  11 |  *
  12 |  */
  13 | 
  14 | #pragma once
  15 | 
  16 | #include <algorithm>
  17 | #include <cassert>
  18 | #include <cmath>
  19 | #ifdef HAS_CODECVT
  20 | #include <codecvt>
  21 | #endif
  22 | #include <fstream>
  23 | #include <functional>
  24 | #include <iostream>
  25 | #include <map>
  26 | #include <sstream>
  27 | #include <string>
  28 | #include <typeinfo>
  29 | #include <vector>
  30 | 
  31 | #if defined(_MSC_VER)
  32 | #include <BaseTsd.h>
  33 | typedef SSIZE_T ssize_t;
  34 | #endif
  35 | 
  36 | namespace rapidcsv {
  37 | #if defined(_MSC_VER)
  38 | static const bool sPlatformHasCR = true;
  39 | #else
  40 | static const bool sPlatformHasCR = false;
  41 | #endif
  42 | 
  43 | /**
  44 |  * @brief     Datastructure holding parameters controlling how invalid numbers (including
  45 |  *            empty strings) should be handled.
  46 |  */
  47 | struct ConverterParams {
  48 |     /**
  49 |      * @brief   Constructor
  50 |      * @param   pHasDefaultConverter  specifies if conversion of non-numerical strings shall be
  51 |      *                                converted to a default numerical value, instead of causing
  52 |      *                                an exception to be thrown (default).
  53 |      * @param   pDefaultFloat         floating-point default value to represent invalid numbers.
  54 |      * @param   pDefaultInteger       integer default value to represent invalid numbers.
  55 |      */
  56 |     explicit ConverterParams(const bool pHasDefaultConverter = false,
  57 |                              const long double pDefaultFloat = std::numeric_limits<long double>::signaling_NaN(),
  58 |                              const long long pDefaultInteger = 0)
  59 |         : mHasDefaultConverter(pHasDefaultConverter), mDefaultFloat(pDefaultFloat), mDefaultInteger(pDefaultInteger) {}
  60 | 
  61 |     /**
  62 |      * @brief   specifies if conversion of non-numerical strings shall be converted to a default
  63 |      *          numerical value, instead of causing an exception to be thrown (default).
  64 |      */
  65 |     bool mHasDefaultConverter;
  66 | 
  67 |     /**
  68 |      * @brief   floating-point default value to represent invalid numbers.
  69 |      */
  70 |     long double mDefaultFloat;
  71 | 
  72 |     /**
  73 |      * @brief   integer default value to represent invalid numbers.
  74 |      */
  75 |     long long mDefaultInteger;
  76 | };
  77 | 
  78 | /**
  79 |  * @brief     Exception thrown when attempting to access Document data in a datatype which
  80 |  *            is not supported by the Converter class.
  81 |  */
  82 | class no_converter : public std::exception {
  83 |     /**
  84 |      * @brief   Provides details about the exception
  85 |      * @returns an explanatory string
  86 |      */
  87 |     virtual const char* what() const throw() { return "unsupported conversion datatype"; }
  88 | };
  89 | 
  90 | /**
  91 |  * @brief     Class providing conversion to/from numerical datatypes and strings. Only
  92 |  *            intended for rapidcsv internal usage, but exposed externally to allow
  93 |  *            specialization for custom datatype conversions.
  94 |  */
  95 | template <typename T>
  96 | class Converter {
  97 | public:
  98 |     /**
  99 |      * @brief   Constructor
 100 |      * @param   pConverterParams      specifies how conversion of non-numerical values to
 101 |      *                                numerical datatype shall be handled.
 102 |      */
 103 |     Converter(const ConverterParams& pConverterParams) : mConverterParams(pConverterParams) {}
 104 | 
 105 |     /**
 106 |      * @brief   Converts numerical value to string representation.
 107 |      * @param   pVal                  numerical value
 108 |      * @param   pStr                  output string
 109 |      */
 110 |     void ToStr(const T& pVal, std::string& pStr) const {
 111 |         if (typeid(T) == typeid(int) || typeid(T) == typeid(long) || typeid(T) == typeid(long long) || typeid(T) == typeid(unsigned) ||
 112 |             typeid(T) == typeid(unsigned long) || typeid(T) == typeid(unsigned long long) || typeid(T) == typeid(float) ||
 113 |             typeid(T) == typeid(double) || typeid(T) == typeid(long double) || typeid(T) == typeid(char)) {
 114 |             std::ostringstream out;
 115 |             out << pVal;
 116 |             pStr = out.str();
 117 |         } else {
 118 |             throw no_converter();
 119 |         }
 120 |     }
 121 | 
 122 |     /**
 123 |      * @brief   Converts string holding a numerical value to numerical datatype representation.
 124 |      * @param   pVal                  numerical value
 125 |      * @param   pStr                  output string
 126 |      */
 127 |     void ToVal(const std::string& pStr, T& pVal) const {
 128 |         try {
 129 |             if (typeid(T) == typeid(int)) {
 130 |                 pVal = static_cast<T>(std::stoi(pStr));
 131 |                 return;
 132 |             } else if (typeid(T) == typeid(long)) {
 133 |                 pVal = static_cast<T>(std::stol(pStr));
 134 |                 return;
 135 |             } else if (typeid(T) == typeid(long long)) {
 136 |                 pVal = static_cast<T>(std::stoll(pStr));
 137 |                 return;
 138 |             } else if (typeid(T) == typeid(unsigned)) {
 139 |                 pVal = static_cast<T>(std::stoul(pStr));
 140 |                 return;
 141 |             } else if (typeid(T) == typeid(unsigned long)) {
 142 |                 pVal = static_cast<T>(std::stoul(pStr));
 143 |                 return;
 144 |             } else if (typeid(T) == typeid(unsigned long long)) {
 145 |                 pVal = static_cast<T>(std::stoull(pStr));
 146 |                 return;
 147 |             }
 148 |         } catch (...) {
 149 |             if (!mConverterParams.mHasDefaultConverter) {
 150 |                 throw;
 151 |             } else {
 152 |                 pVal = static_cast<T>(mConverterParams.mDefaultInteger);
 153 |                 return;
 154 |             }
 155 |         }
 156 | 
 157 |         try {
 158 |             if (typeid(T) == typeid(float)) {
 159 |                 pVal = static_cast<T>(std::stof(pStr));
 160 |                 return;
 161 |             } else if (typeid(T) == typeid(double)) {
 162 |                 pVal = static_cast<T>(std::stod(pStr));
 163 |                 return;
 164 |             } else if (typeid(T) == typeid(long double)) {
 165 |                 pVal = static_cast<T>(std::stold(pStr));
 166 |                 return;
 167 |             }
 168 |         } catch (...) {
 169 |             if (!mConverterParams.mHasDefaultConverter) {
 170 |                 throw;
 171 |             } else {
 172 |                 pVal = static_cast<T>(mConverterParams.mDefaultFloat);
 173 |                 return;
 174 |             }
 175 |         }
 176 | 
 177 |         if (typeid(T) == typeid(char)) {
 178 |             pVal = static_cast<T>(pStr[0]);
 179 |             return;
 180 |         } else {
 181 |             throw no_converter();
 182 |         }
 183 |     }
 184 | 
 185 | private:
 186 |     const ConverterParams& mConverterParams;
 187 | };
 188 | 
 189 | /**
 190 |  * @brief     Specialized implementation handling string to string conversion.
 191 |  * @param     pVal                  string
 192 |  * @param     pStr                  string
 193 |  */
 194 | template <>
 195 | inline void Converter<std::string>::ToStr(const std::string& pVal, std::string& pStr) const {
 196 |     pStr = pVal;
 197 | }
 198 | 
 199 | /**
 200 |  * @brief     Specialized implementation handling string to string conversion.
 201 |  * @param     pVal                  string
 202 |  * @param     pStr                  string
 203 |  */
 204 | template <>
 205 | inline void Converter<std::string>::ToVal(const std::string& pStr, std::string& pVal) const {
 206 |     pVal = pStr;
 207 | }
 208 | 
 209 | template <typename T>
 210 | using ConvFunc = std::function<void(const std::string& pStr, T& pVal)>;
 211 | 
 212 | /**
 213 |  * @brief     Datastructure holding parameters controlling which row and column should be
 214 |  *            treated as labels.
 215 |  */
 216 | struct LabelParams {
 217 |     /**
 218 |      * @brief   Constructor
 219 |      * @param   pColumnNameIdx        specifies the zero-based row index of the column labels, setting
 220 |      *                                it to -1 prevents column lookup by label name, and gives access
 221 |      *                                to all rows as document data. Default: 0
 222 |      * @param   pRowNameIdx           specifies the zero-based column index of the row labels, setting
 223 |      *                                it to -1 prevents row lookup by label name, and gives access
 224 |      *                                to all columns as document data. Default: -1
 225 |      */
 226 |     explicit LabelParams(const int pColumnNameIdx = 0, const int pRowNameIdx = -1) : mColumnNameIdx(pColumnNameIdx), mRowNameIdx(pRowNameIdx) {}
 227 | 
 228 |     /**
 229 |      * @brief   specifies the zero-based row index of the column labels.
 230 |      */
 231 |     int mColumnNameIdx;
 232 | 
 233 |     /**
 234 |      * @brief   specifies the zero-based column index of the row labels.
 235 |      */
 236 |     int mRowNameIdx;
 237 | };
 238 | 
 239 | /**
 240 |  * @brief     Datastructure holding parameters controlling how the CSV data fields are separated.
 241 |  */
 242 | struct SeparatorParams {
 243 |     /**
 244 |      * @brief   Constructor
 245 |      * @param   pSeparator            specifies the column separator (default ',').
 246 |      * @param   pTrim                 specifies whether to trim leading and trailing spaces from
 247 |      *                                cells read (default false).
 248 |      * @param   pHasCR                specifies whether a new document (i.e. not an existing document read)
 249 |      *                                should use CR/LF instead of only LF (default is to use standard
 250 |      *                                behavior of underlying platforms - CR/LF for Win, and LF for others).
 251 |      * @param   pQuotedLinebreaks     specifies whether to allow line breaks in quoted text (default false)
 252 |      * @param   pAutoQuote            specifies whether to automatically dequote data during read, and add
 253 |      *                                quotes during write (default true).
 254 |      */
 255 |     explicit SeparatorParams(const char pSeparator = ',',
 256 |                              const bool pTrim = false,
 257 |                              const bool pHasCR = sPlatformHasCR,
 258 |                              const bool pQuotedLinebreaks = false,
 259 |                              const bool pAutoQuote = true)
 260 |         : mSeparator(pSeparator), mTrim(pTrim), mHasCR(pHasCR), mQuotedLinebreaks(pQuotedLinebreaks), mAutoQuote(pAutoQuote) {}
 261 | 
 262 |     /**
 263 |      * @brief   specifies the column separator.
 264 |      */
 265 |     char mSeparator;
 266 | 
 267 |     /**
 268 |      * @brief   specifies whether to trim leading and trailing spaces from cells read.
 269 |      */
 270 |     bool mTrim;
 271 | 
 272 |     /**
 273 |      * @brief   specifies whether new documents should use CR/LF instead of LF.
 274 |      */
 275 |     bool mHasCR;
 276 | 
 277 |     /**
 278 |      * @brief   specifies whether to allow line breaks in quoted text.
 279 |      */
 280 |     bool mQuotedLinebreaks;
 281 | 
 282 |     /**
 283 |      * @brief   specifies whether to automatically dequote cell data.
 284 |      */
 285 |     bool mAutoQuote;
 286 | };
 287 | 
 288 | /**
 289 |  * @brief     Class representing a CSV document.
 290 |  */
 291 | class Document {
 292 | public:
 293 |     /**
 294 |      * @brief   Constructor
 295 |      * @param   pPath                 specifies the path of an existing CSV-file to populate the Document
 296 |      *                                data with.
 297 |      * @param   pLabelParams          specifies which row and column should be treated as labels.
 298 |      * @param   pSeparatorParams      specifies which field and row separators should be used.
 299 |      * @param   pConverterParams      specifies how invalid numbers (including empty strings) should be
 300 |      *                                handled.
 301 |      */
 302 |     explicit Document(const std::string& pPath = std::string(),
 303 |                       const LabelParams& pLabelParams = LabelParams(),
 304 |                       const SeparatorParams& pSeparatorParams = SeparatorParams(),
 305 |                       const ConverterParams& pConverterParams = ConverterParams())
 306 |         : mPath(pPath), mLabelParams(pLabelParams), mSeparatorParams(pSeparatorParams), mConverterParams(pConverterParams) {
 307 |         if (!mPath.empty()) {
 308 |             ReadCsv();
 309 |         }
 310 |     }
 311 | 
 312 |     /**
 313 |      * @brief   Constructor
 314 |      * @param   pStream               specifies an input stream to read CSV data from.
 315 |      * @param   pLabelParams          specifies which row and column should be treated as labels.
 316 |      * @param   pSeparatorParams      specifies which field and row separators should be used.
 317 |      * @param   pConverterParams      specifies how invalid numbers (including empty strings) should be
 318 |      *                                handled.
 319 |      */
 320 |     explicit Document(std::istream& pStream,
 321 |                       const LabelParams& pLabelParams = LabelParams(),
 322 |                       const SeparatorParams& pSeparatorParams = SeparatorParams(),
 323 |                       const ConverterParams& pConverterParams = ConverterParams())
 324 |         : mPath(), mLabelParams(pLabelParams), mSeparatorParams(pSeparatorParams), mConverterParams(pConverterParams) {
 325 |         ReadCsv(pStream);
 326 |     }
 327 | 
 328 |     /**
 329 |      * @brief   Read Document data from file.
 330 |      * @param   pPath                 specifies the path of an existing CSV-file to populate the Document
 331 |      *                                data with.
 332 |      * @param   pLabelParams          specifies which row and column should be treated as labels.
 333 |      * @param   pSeparatorParams      specifies which field and row separators should be used.
 334 |      * @param   pConverterParams      specifies how invalid numbers (including empty strings) should be
 335 |      *                                handled.
 336 |      */
 337 |     void Load(const std::string& pPath,
 338 |               const LabelParams& pLabelParams = LabelParams(),
 339 |               const SeparatorParams& pSeparatorParams = SeparatorParams(),
 340 |               const ConverterParams& pConverterParams = ConverterParams()) {
 341 |         mPath = pPath;
 342 |         mLabelParams = pLabelParams;
 343 |         mSeparatorParams = pSeparatorParams;
 344 |         mConverterParams = pConverterParams;
 345 |         ReadCsv();
 346 |     }
 347 | 
 348 |     /**
 349 |      * @brief   Read Document data from stream.
 350 |      * @param   pStream               specifies an input stream to read CSV data from.
 351 |      * @param   pLabelParams          specifies which row and column should be treated as labels.
 352 |      * @param   pSeparatorParams      specifies which field and row separators should be used.
 353 |      * @param   pConverterParams      specifies how invalid numbers (including empty strings) should be
 354 |      *                                handled.
 355 |      */
 356 |     void Load(std::istream& pStream,
 357 |               const LabelParams& pLabelParams = LabelParams(),
 358 |               const SeparatorParams& pSeparatorParams = SeparatorParams(),
 359 |               const ConverterParams& pConverterParams = ConverterParams()) {
 360 |         mPath = "";
 361 |         mLabelParams = pLabelParams;
 362 |         mSeparatorParams = pSeparatorParams;
 363 |         mConverterParams = pConverterParams;
 364 |         ReadCsv(pStream);
 365 |     }
 366 | 
 367 |     /**
 368 |      * @brief   Write Document data to file.
 369 |      * @param   pPath                 optionally specifies the path where the CSV-file will be created
 370 |      *                                (if not specified, the original path provided when creating or
 371 |      *                                loading the Document data will be used).
 372 |      */
 373 |     void Save(const std::string& pPath = std::string()) {
 374 |         if (!pPath.empty()) {
 375 |             mPath = pPath;
 376 |         }
 377 |         WriteCsv();
 378 |     }
 379 | 
 380 |     /**
 381 |      * @brief   Write Document data to stream.
 382 |      * @param   pStream               specifies an output stream to write the data to.
 383 |      */
 384 |     void Save(std::ostream& pStream) { WriteCsv(pStream); }
 385 | 
 386 |     /**
 387 |      * @brief   Clears loaded Document data.
 388 |      *
 389 |      */
 390 |     void Clear() {
 391 |         mData.clear();
 392 |         mColumnNames.clear();
 393 |         mRowNames.clear();
 394 | #ifdef HAS_CODECVT
 395 |         mIsUtf16 = false;
 396 |         mIsLE = false;
 397 | #endif
 398 |     }
 399 | 
 400 |     /**
 401 |      * @brief   Get column index by name.
 402 |      * @param   pColumnName           column label name.
 403 |      * @returns zero-based column index.
 404 |      */
 405 |     ssize_t GetColumnIdx(const std::string& pColumnName) const {
 406 |         if (mLabelParams.mColumnNameIdx >= 0) {
 407 |             if (mColumnNames.find(pColumnName) != mColumnNames.end()) {
 408 |                 return mColumnNames.at(pColumnName) - (mLabelParams.mRowNameIdx + 1);
 409 |             }
 410 |         }
 411 |         return -1;
 412 |     }
 413 | 
 414 |     /**
 415 |      * @brief   Get column by index.
 416 |      * @param   pColumnIdx            zero-based column index.
 417 |      * @returns vector of column data.
 418 |      */
 419 |     template <typename T>
 420 |     std::vector<T> GetColumn(const size_t pColumnIdx) const {
 421 |         const ssize_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 422 |         std::vector<T> column;
 423 |         Converter<T> converter(mConverterParams);
 424 |         for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
 425 |             if (std::distance(mData.begin(), itRow) > mLabelParams.mColumnNameIdx) {
 426 |                 T val;
 427 |                 converter.ToVal(itRow->at(columnIdx), val);
 428 |                 column.push_back(val);
 429 |             }
 430 |         }
 431 |         return column;
 432 |     }
 433 | 
 434 |     /**
 435 |      * @brief   Get column by index.
 436 |      * @param   pColumnIdx            zero-based column index.
 437 |      * @param   pToVal                conversion function.
 438 |      * @returns vector of column data.
 439 |      */
 440 |     template <typename T>
 441 |     std::vector<T> GetColumn(const size_t pColumnIdx, ConvFunc<T> pToVal) const {
 442 |         const ssize_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 443 |         std::vector<T> column;
 444 |         for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
 445 |             if (std::distance(mData.begin(), itRow) > mLabelParams.mColumnNameIdx) {
 446 |                 T val;
 447 |                 pToVal(itRow->at(columnIdx), val);
 448 |                 column.push_back(val);
 449 |             }
 450 |         }
 451 |         return column;
 452 |     }
 453 | 
 454 |     /**
 455 |      * @brief   Get column by name.
 456 |      * @param   pColumnName           column label name.
 457 |      * @returns vector of column data.
 458 |      */
 459 |     template <typename T>
 460 |     std::vector<T> GetColumn(const std::string& pColumnName) const {
 461 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 462 |         if (columnIdx < 0) {
 463 |             throw std::out_of_range("column not found: " + pColumnName);
 464 |         }
 465 |         return GetColumn<T>(columnIdx);
 466 |     }
 467 | 
 468 |     /**
 469 |      * @brief   Get column by name.
 470 |      * @param   pColumnName           column label name.
 471 |      * @param   pToVal                conversion function.
 472 |      * @returns vector of column data.
 473 |      */
 474 |     template <typename T>
 475 |     std::vector<T> GetColumn(const std::string& pColumnName, ConvFunc<T> pToVal) const {
 476 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 477 |         if (columnIdx < 0) {
 478 |             throw std::out_of_range("column not found: " + pColumnName);
 479 |         }
 480 |         return GetColumn<T>(columnIdx, pToVal);
 481 |     }
 482 | 
 483 |     /**
 484 |      * @brief   Set column by index.
 485 |      * @param   pColumnIdx            zero-based column index.
 486 |      * @param   pColumn               vector of column data.
 487 |      */
 488 |     template <typename T>
 489 |     void SetColumn(const size_t pColumnIdx, const std::vector<T>& pColumn) {
 490 |         const size_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 491 | 
 492 |         while (pColumn.size() + (mLabelParams.mColumnNameIdx + 1) > GetDataRowCount()) {
 493 |             std::vector<std::string> row;
 494 |             row.resize(GetDataColumnCount());
 495 |             mData.push_back(row);
 496 |         }
 497 | 
 498 |         if ((columnIdx + 1) > GetDataColumnCount()) {
 499 |             for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
 500 |                 itRow->resize(columnIdx + 1 + (mLabelParams.mRowNameIdx + 1));
 501 |             }
 502 |         }
 503 | 
 504 |         Converter<T> converter(mConverterParams);
 505 |         for (auto itRow = pColumn.begin(); itRow != pColumn.end(); ++itRow) {
 506 |             std::string str;
 507 |             converter.ToStr(*itRow, str);
 508 |             mData.at(std::distance(pColumn.begin(), itRow) + (mLabelParams.mColumnNameIdx + 1)).at(columnIdx) = str;
 509 |         }
 510 |     }
 511 | 
 512 |     /**
 513 |      * @brief   Set column by name.
 514 |      * @param   pColumnName           column label name.
 515 |      * @param   pColumn               vector of column data.
 516 |      */
 517 |     template <typename T>
 518 |     void SetColumn(const std::string& pColumnName, const std::vector<T>& pColumn) {
 519 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 520 |         if (columnIdx < 0) {
 521 |             throw std::out_of_range("column not found: " + pColumnName);
 522 |         }
 523 |         SetColumn<T>(columnIdx, pColumn);
 524 |     }
 525 | 
 526 |     /**
 527 |      * @brief   Remove column by index.
 528 |      * @param   pColumnIdx            zero-based column index.
 529 |      */
 530 |     void RemoveColumn(const size_t pColumnIdx) {
 531 |         const ssize_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 532 |         for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
 533 |             itRow->erase(itRow->begin() + columnIdx);
 534 |         }
 535 |     }
 536 | 
 537 |     /**
 538 |      * @brief   Remove column by name.
 539 |      * @param   pColumnName           column label name.
 540 |      */
 541 |     void RemoveColumn(const std::string& pColumnName) {
 542 |         ssize_t columnIdx = GetColumnIdx(pColumnName);
 543 |         if (columnIdx < 0) {
 544 |             throw std::out_of_range("column not found: " + pColumnName);
 545 |         }
 546 | 
 547 |         RemoveColumn(columnIdx);
 548 |     }
 549 | 
 550 |     /**
 551 |      * @brief   Insert column at specified index.
 552 |      * @param   pColumnIdx            zero-based column index.
 553 |      * @param   pColumn               vector of column data (optional argument).
 554 |      * @param   pColumnName           column label name (optional argument).
 555 |      */
 556 |     template <typename T>
 557 |     void InsertColumn(const size_t pColumnIdx, const std::vector<T>& pColumn = std::vector<T>(), const std::string& pColumnName = std::string()) {
 558 |         const size_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 559 | 
 560 |         std::vector<std::string> column;
 561 |         if (pColumn.empty()) {
 562 |             column.resize(GetDataRowCount());
 563 |         } else {
 564 |             column.resize(pColumn.size() + (mLabelParams.mColumnNameIdx + 1));
 565 |             Converter<T> converter(mConverterParams);
 566 |             for (auto itRow = pColumn.begin(); itRow != pColumn.end(); ++itRow) {
 567 |                 std::string str;
 568 |                 converter.ToStr(*itRow, str);
 569 |                 const size_t rowIdx = std::distance(pColumn.begin(), itRow) + (mLabelParams.mColumnNameIdx + 1);
 570 |                 column.at(rowIdx) = str;
 571 |             }
 572 |         }
 573 | 
 574 |         while (column.size() > GetDataRowCount()) {
 575 |             std::vector<std::string> row;
 576 |             const size_t columnCount = std::max(static_cast<size_t>(mLabelParams.mColumnNameIdx + 1), GetDataColumnCount());
 577 |             row.resize(columnCount);
 578 |             mData.push_back(row);
 579 |         }
 580 | 
 581 |         for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
 582 |             const size_t rowIdx = std::distance(mData.begin(), itRow);
 583 |             itRow->insert(itRow->begin() + columnIdx, column.at(rowIdx));
 584 |         }
 585 | 
 586 |         if (!pColumnName.empty()) {
 587 |             SetColumnName(pColumnIdx, pColumnName);
 588 |         }
 589 |     }
 590 | 
 591 |     /**
 592 |      * @brief   Get number of data columns (excluding label columns).
 593 |      * @returns column count.
 594 |      */
 595 |     size_t GetColumnCount() const {
 596 |         const ssize_t count = static_cast<ssize_t>((mData.size() > 0) ? mData.at(0).size() : 0) - (mLabelParams.mRowNameIdx + 1);
 597 |         return (count >= 0) ? count : 0;
 598 |     }
 599 | 
 600 |     /**
 601 |      * @brief   Get row index by name.
 602 |      * @param   pRowName              row label name.
 603 |      * @returns zero-based row index.
 604 |      */
 605 |     ssize_t GetRowIdx(const std::string& pRowName) const {
 606 |         if (mLabelParams.mRowNameIdx >= 0) {
 607 |             if (mRowNames.find(pRowName) != mRowNames.end()) {
 608 |                 return mRowNames.at(pRowName) - (mLabelParams.mColumnNameIdx + 1);
 609 |             }
 610 |         }
 611 |         return -1;
 612 |     }
 613 | 
 614 |     /**
 615 |      * @brief   Get row by index.
 616 |      * @param   pRowIdx               zero-based row index.
 617 |      * @returns vector of row data.
 618 |      */
 619 |     template <typename T>
 620 |     std::vector<T> GetRow(const size_t pRowIdx) const {
 621 |         const ssize_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 622 |         std::vector<T> row;
 623 |         Converter<T> converter(mConverterParams);
 624 |         for (auto itCol = mData.at(rowIdx).begin(); itCol != mData.at(rowIdx).end(); ++itCol) {
 625 |             if (std::distance(mData.at(rowIdx).begin(), itCol) > mLabelParams.mRowNameIdx) {
 626 |                 T val;
 627 |                 converter.ToVal(*itCol, val);
 628 |                 row.push_back(val);
 629 |             }
 630 |         }
 631 |         return row;
 632 |     }
 633 | 
 634 |     /**
 635 |      * @brief   Get row by index.
 636 |      * @param   pRowIdx               zero-based row index.
 637 |      * @param   pToVal                conversion function.
 638 |      * @returns vector of row data.
 639 |      */
 640 |     template <typename T>
 641 |     std::vector<T> GetRow(const size_t pRowIdx, ConvFunc<T> pToVal) const {
 642 |         const ssize_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 643 |         std::vector<T> row;
 644 |         Converter<T> converter(mConverterParams);
 645 |         for (auto itCol = mData.at(rowIdx).begin(); itCol != mData.at(rowIdx).end(); ++itCol) {
 646 |             if (std::distance(mData.at(rowIdx).begin(), itCol) > mLabelParams.mRowNameIdx) {
 647 |                 T val;
 648 |                 pToVal(*itCol, val);
 649 |                 row.push_back(val);
 650 |             }
 651 |         }
 652 |         return row;
 653 |     }
 654 | 
 655 |     /**
 656 |      * @brief   Get row by name.
 657 |      * @param   pRowName              row label name.
 658 |      * @returns vector of row data.
 659 |      */
 660 |     template <typename T>
 661 |     std::vector<T> GetRow(const std::string& pRowName) const {
 662 |         ssize_t rowIdx = GetRowIdx(pRowName);
 663 |         if (rowIdx < 0) {
 664 |             throw std::out_of_range("row not found: " + pRowName);
 665 |         }
 666 |         return GetRow<T>(rowIdx);
 667 |     }
 668 | 
 669 |     /**
 670 |      * @brief   Get row by name.
 671 |      * @param   pRowName              row label name.
 672 |      * @param   pToVal                conversion function.
 673 |      * @returns vector of row data.
 674 |      */
 675 |     template <typename T>
 676 |     std::vector<T> GetRow(const std::string& pRowName, ConvFunc<T> pToVal) const {
 677 |         ssize_t rowIdx = GetRowIdx(pRowName);
 678 |         if (rowIdx < 0) {
 679 |             throw std::out_of_range("row not found: " + pRowName);
 680 |         }
 681 |         return GetRow<T>(rowIdx, pToVal);
 682 |     }
 683 | 
 684 |     /**
 685 |      * @brief   Set row by index.
 686 |      * @param   pRowIdx               zero-based row index.
 687 |      * @param   pRow                  vector of row data.
 688 |      */
 689 |     template <typename T>
 690 |     void SetRow(const size_t pRowIdx, const std::vector<T>& pRow) {
 691 |         const size_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 692 | 
 693 |         while ((rowIdx + 1) > GetDataRowCount()) {
 694 |             std::vector<std::string> row;
 695 |             row.resize(GetDataColumnCount());
 696 |             mData.push_back(row);
 697 |         }
 698 | 
 699 |         if (pRow.size() > GetDataColumnCount()) {
 700 |             for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
 701 |                 itRow->resize(pRow.size() + (mLabelParams.mRowNameIdx + 1));
 702 |             }
 703 |         }
 704 | 
 705 |         Converter<T> converter(mConverterParams);
 706 |         for (auto itCol = pRow.begin(); itCol != pRow.end(); ++itCol) {
 707 |             std::string str;
 708 |             converter.ToStr(*itCol, str);
 709 |             mData.at(rowIdx).at(std::distance(pRow.begin(), itCol) + (mLabelParams.mRowNameIdx + 1)) = str;
 710 |         }
 711 |     }
 712 | 
 713 |     /**
 714 |      * @brief   Set row by name.
 715 |      * @param   pRowName              row label name.
 716 |      * @param   pRow                  vector of row data.
 717 |      */
 718 |     template <typename T>
 719 |     void SetRow(const std::string& pRowName, const std::vector<T>& pRow) {
 720 |         ssize_t rowIdx = GetRowIdx(pRowName);
 721 |         if (rowIdx < 0) {
 722 |             throw std::out_of_range("row not found: " + pRowName);
 723 |         }
 724 |         return SetRow<T>(rowIdx, pRow);
 725 |     }
 726 | 
 727 |     /**
 728 |      * @brief   Remove row by index.
 729 |      * @param   pRowIdx               zero-based row index.
 730 |      */
 731 |     void RemoveRow(const size_t pRowIdx) {
 732 |         const ssize_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 733 |         mData.erase(mData.begin() + rowIdx);
 734 |     }
 735 | 
 736 |     /**
 737 |      * @brief   Remove row by name.
 738 |      * @param   pRowName              row label name.
 739 |      */
 740 |     void RemoveRow(const std::string& pRowName) {
 741 |         ssize_t rowIdx = GetRowIdx(pRowName);
 742 |         if (rowIdx < 0) {
 743 |             throw std::out_of_range("row not found: " + pRowName);
 744 |         }
 745 | 
 746 |         RemoveRow(rowIdx);
 747 |     }
 748 | 
 749 |     /**
 750 |      * @brief   Insert row at specified index.
 751 |      * @param   pRowIdx               zero-based row index.
 752 |      * @param   pRow                  vector of row data (optional argument).
 753 |      * @param   pRowName              row label name (optional argument).
 754 |      */
 755 |     template <typename T>
 756 |     void InsertRow(const size_t pRowIdx, const std::vector<T>& pRow = std::vector<T>(), const std::string& pRowName = std::string()) {
 757 |         const size_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 758 | 
 759 |         std::vector<std::string> row;
 760 |         if (pRow.empty()) {
 761 |             row.resize(GetDataColumnCount());
 762 |         } else {
 763 |             row.resize(pRow.size() + (mLabelParams.mRowNameIdx + 1));
 764 |             Converter<T> converter(mConverterParams);
 765 |             for (auto itCol = pRow.begin(); itCol != pRow.end(); ++itCol) {
 766 |                 std::string str;
 767 |                 converter.ToStr(*itCol, str);
 768 |                 row.at(std::distance(pRow.begin(), itCol) + (mLabelParams.mRowNameIdx + 1)) = str;
 769 |             }
 770 |         }
 771 | 
 772 |         while (rowIdx > GetDataRowCount()) {
 773 |             std::vector<std::string> tempRow;
 774 |             tempRow.resize(GetDataColumnCount());
 775 |             mData.push_back(tempRow);
 776 |         }
 777 | 
 778 |         mData.insert(mData.begin() + rowIdx, row);
 779 | 
 780 |         if (!pRowName.empty()) {
 781 |             SetRowName(pRowIdx, pRowName);
 782 |         }
 783 |     }
 784 | 
 785 |     /**
 786 |      * @brief   Get number of data rows (excluding label rows).
 787 |      * @returns row count.
 788 |      */
 789 |     size_t GetRowCount() const {
 790 |         const ssize_t count = static_cast<ssize_t>(mData.size()) - (mLabelParams.mColumnNameIdx + 1);
 791 |         return (count >= 0) ? count : 0;
 792 |     }
 793 | 
 794 |     /**
 795 |      * @brief   Get cell by index.
 796 |      * @param   pColumnIdx            zero-based column index.
 797 |      * @param   pRowIdx               zero-based row index.
 798 |      * @returns cell data.
 799 |      */
 800 |     template <typename T>
 801 |     T GetCell(const size_t pColumnIdx, const size_t pRowIdx) const {
 802 |         const ssize_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 803 |         const ssize_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 804 | 
 805 |         T val;
 806 |         Converter<T> converter(mConverterParams);
 807 |         converter.ToVal(mData.at(rowIdx).at(columnIdx), val);
 808 |         return val;
 809 |     }
 810 | 
 811 |     /**
 812 |      * @brief   Get cell by index.
 813 |      * @param   pColumnIdx            zero-based column index.
 814 |      * @param   pRowIdx               zero-based row index.
 815 |      * @param   pToVal                conversion function.
 816 |      * @returns cell data.
 817 |      */
 818 |     template <typename T>
 819 |     T GetCell(const size_t pColumnIdx, const size_t pRowIdx, ConvFunc<T> pToVal) const {
 820 |         const ssize_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 821 |         const ssize_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 822 | 
 823 |         T val;
 824 |         pToVal(mData.at(rowIdx).at(columnIdx), val);
 825 |         return val;
 826 |     }
 827 | 
 828 |     /**
 829 |      * @brief   Get cell by name.
 830 |      * @param   pColumnName           column label name.
 831 |      * @param   pRowName              row label name.
 832 |      * @returns cell data.
 833 |      */
 834 |     template <typename T>
 835 |     T GetCell(const std::string& pColumnName, const std::string& pRowName) const {
 836 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 837 |         if (columnIdx < 0) {
 838 |             throw std::out_of_range("column not found: " + pColumnName);
 839 |         }
 840 | 
 841 |         const ssize_t rowIdx = GetRowIdx(pRowName);
 842 |         if (rowIdx < 0) {
 843 |             throw std::out_of_range("row not found: " + pRowName);
 844 |         }
 845 | 
 846 |         return GetCell<T>(columnIdx, rowIdx);
 847 |     }
 848 | 
 849 |     /**
 850 |      * @brief   Get cell by name.
 851 |      * @param   pColumnName           column label name.
 852 |      * @param   pRowName              row label name.
 853 |      * @param   pToVal                conversion function.
 854 |      * @returns cell data.
 855 |      */
 856 |     template <typename T>
 857 |     T GetCell(const std::string& pColumnName, const std::string& pRowName, ConvFunc<T> pToVal) const {
 858 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 859 |         if (columnIdx < 0) {
 860 |             throw std::out_of_range("column not found: " + pColumnName);
 861 |         }
 862 | 
 863 |         const ssize_t rowIdx = GetRowIdx(pRowName);
 864 |         if (rowIdx < 0) {
 865 |             throw std::out_of_range("row not found: " + pRowName);
 866 |         }
 867 | 
 868 |         return GetCell<T>(columnIdx, rowIdx, pToVal);
 869 |     }
 870 | 
 871 |     /**
 872 |      * @brief   Get cell by column name and row index.
 873 |      * @param   pColumnName           column label name.
 874 |      * @param   pRowIdx               zero-based row index.
 875 |      * @returns cell data.
 876 |      */
 877 |     template <typename T>
 878 |     T GetCell(const std::string& pColumnName, const size_t pRowIdx) const {
 879 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 880 |         if (columnIdx < 0) {
 881 |             throw std::out_of_range("column not found: " + pColumnName);
 882 |         }
 883 | 
 884 |         return GetCell<T>(columnIdx, pRowIdx);
 885 |     }
 886 | 
 887 |     /**
 888 |      * @brief   Get cell by column name and row index.
 889 |      * @param   pColumnName           column label name.
 890 |      * @param   pRowIdx               zero-based row index.
 891 |      * @param   pToVal                conversion function.
 892 |      * @returns cell data.
 893 |      */
 894 |     template <typename T>
 895 |     T GetCell(const std::string& pColumnName, const size_t pRowIdx, ConvFunc<T> pToVal) const {
 896 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 897 |         if (columnIdx < 0) {
 898 |             throw std::out_of_range("column not found: " + pColumnName);
 899 |         }
 900 | 
 901 |         return GetCell<T>(columnIdx, pRowIdx, pToVal);
 902 |     }
 903 | 
 904 |     /**
 905 |      * @brief   Get cell by column index and row name.
 906 |      * @param   pColumnIdx            zero-based column index.
 907 |      * @param   pRowName              row label name.
 908 |      * @returns cell data.
 909 |      */
 910 |     template <typename T>
 911 |     T GetCell(const size_t pColumnIdx, const std::string& pRowName) const {
 912 |         const ssize_t rowIdx = GetRowIdx(pRowName);
 913 |         if (rowIdx < 0) {
 914 |             throw std::out_of_range("row not found: " + pRowName);
 915 |         }
 916 | 
 917 |         return GetCell<T>(pColumnIdx, rowIdx);
 918 |     }
 919 | 
 920 |     /**
 921 |      * @brief   Get cell by column index and row name.
 922 |      * @param   pColumnIdx            zero-based column index.
 923 |      * @param   pRowName              row label name.
 924 |      * @param   pToVal                conversion function.
 925 |      * @returns cell data.
 926 |      */
 927 |     template <typename T>
 928 |     T GetCell(const size_t pColumnIdx, const std::string& pRowName, ConvFunc<T> pToVal) const {
 929 |         const ssize_t rowIdx = GetRowIdx(pRowName);
 930 |         if (rowIdx < 0) {
 931 |             throw std::out_of_range("row not found: " + pRowName);
 932 |         }
 933 | 
 934 |         return GetCell<T>(pColumnIdx, rowIdx, pToVal);
 935 |     }
 936 | 
 937 |     /**
 938 |      * @brief   Set cell by index.
 939 |      * @param   pRowIdx               zero-based row index.
 940 |      * @param   pColumnIdx            zero-based column index.
 941 |      * @param   pCell                 cell data.
 942 |      */
 943 |     template <typename T>
 944 |     void SetCell(const size_t pColumnIdx, const size_t pRowIdx, const T& pCell) {
 945 |         const size_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 946 |         const size_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
 947 | 
 948 |         while ((rowIdx + 1) > GetDataRowCount()) {
 949 |             std::vector<std::string> row;
 950 |             row.resize(GetDataColumnCount());
 951 |             mData.push_back(row);
 952 |         }
 953 | 
 954 |         if ((columnIdx + 1) > GetDataColumnCount()) {
 955 |             for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
 956 |                 itRow->resize(columnIdx + 1);
 957 |             }
 958 |         }
 959 | 
 960 |         std::string str;
 961 |         Converter<T> converter(mConverterParams);
 962 |         converter.ToStr(pCell, str);
 963 |         mData.at(rowIdx).at(columnIdx) = str;
 964 |     }
 965 | 
 966 |     /**
 967 |      * @brief   Set cell by name.
 968 |      * @param   pColumnName           column label name.
 969 |      * @param   pRowName              row label name.
 970 |      * @param   pCell                 cell data.
 971 |      */
 972 |     template <typename T>
 973 |     void SetCell(const std::string& pColumnName, const std::string& pRowName, const T& pCell) {
 974 |         const ssize_t columnIdx = GetColumnIdx(pColumnName);
 975 |         if (columnIdx < 0) {
 976 |             throw std::out_of_range("column not found: " + pColumnName);
 977 |         }
 978 | 
 979 |         const ssize_t rowIdx = GetRowIdx(pRowName);
 980 |         if (rowIdx < 0) {
 981 |             throw std::out_of_range("row not found: " + pRowName);
 982 |         }
 983 | 
 984 |         SetCell<T>(columnIdx, rowIdx, pCell);
 985 |     }
 986 | 
 987 |     /**
 988 |      * @brief   Get column name
 989 |      * @param   pColumnIdx            zero-based column index.
 990 |      * @returns column name.
 991 |      */
 992 |     std::string GetColumnName(const ssize_t pColumnIdx) {
 993 |         const ssize_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
 994 |         if (mLabelParams.mColumnNameIdx < 0) {
 995 |             throw std::out_of_range("column name row index < 0: " + std::to_string(mLabelParams.mColumnNameIdx));
 996 |         }
 997 | 
 998 |         return mData.at(mLabelParams.mColumnNameIdx).at(columnIdx);
 999 |     }
1000 | 
1001 |     /**
1002 |      * @brief   Set column name
1003 |      * @param   pColumnIdx            zero-based column index.
1004 |      * @param   pColumnName           column name.
1005 |      */
1006 |     void SetColumnName(size_t pColumnIdx, const std::string& pColumnName) {
1007 |         const ssize_t columnIdx = pColumnIdx + (mLabelParams.mRowNameIdx + 1);
1008 |         mColumnNames[pColumnName] = columnIdx;
1009 |         if (mLabelParams.mColumnNameIdx < 0) {
1010 |             throw std::out_of_range("column name row index < 0: " + std::to_string(mLabelParams.mColumnNameIdx));
1011 |         }
1012 | 
1013 |         // increase table size if necessary:
1014 |         const int rowIdx = mLabelParams.mColumnNameIdx;
1015 |         if (rowIdx >= static_cast<int>(mData.size())) {
1016 |             mData.resize(rowIdx + 1);
1017 |         }
1018 |         auto& row = mData[rowIdx];
1019 |         if (columnIdx >= static_cast<int>(row.size())) {
1020 |             row.resize(columnIdx + 1);
1021 |         }
1022 | 
1023 |         mData.at(mLabelParams.mColumnNameIdx).at(columnIdx) = pColumnName;
1024 |     }
1025 | 
1026 |     /**
1027 |      * @brief   Get column names
1028 |      * @returns vector of column names.
1029 |      */
1030 |     std::vector<std::string> GetColumnNames() {
1031 |         if (mLabelParams.mColumnNameIdx >= 0) {
1032 |             return std::vector<std::string>(mData.at(mLabelParams.mColumnNameIdx).begin() + (mLabelParams.mRowNameIdx + 1),
1033 |                                             mData.at(mLabelParams.mColumnNameIdx).end());
1034 |         }
1035 | 
1036 |         return std::vector<std::string>();
1037 |     }
1038 | 
1039 |     /**
1040 |      * @brief   Get row name
1041 |      * @param   pRowIdx               zero-based column index.
1042 |      * @returns row name.
1043 |      */
1044 |     std::string GetRowName(const ssize_t pRowIdx) {
1045 |         const ssize_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
1046 |         if (mLabelParams.mRowNameIdx < 0) {
1047 |             throw std::out_of_range("row name column index < 0: " + std::to_string(mLabelParams.mRowNameIdx));
1048 |         }
1049 | 
1050 |         return mData.at(rowIdx).at(mLabelParams.mRowNameIdx);
1051 |     }
1052 | 
1053 |     /**
1054 |      * @brief   Set row name
1055 |      * @param   pRowIdx               zero-based row index.
1056 |      * @param   pRowName              row name.
1057 |      */
1058 |     void SetRowName(size_t pRowIdx, const std::string& pRowName) {
1059 |         const ssize_t rowIdx = pRowIdx + (mLabelParams.mColumnNameIdx + 1);
1060 |         mRowNames[pRowName] = rowIdx;
1061 |         if (mLabelParams.mRowNameIdx < 0) {
1062 |             throw std::out_of_range("row name column index < 0: " + std::to_string(mLabelParams.mRowNameIdx));
1063 |         }
1064 | 
1065 |         // increase table size if necessary:
1066 |         if (rowIdx >= static_cast<int>(mData.size())) {
1067 |             mData.resize(rowIdx + 1);
1068 |         }
1069 |         auto& row = mData[rowIdx];
1070 |         if (mLabelParams.mRowNameIdx >= static_cast<int>(row.size())) {
1071 |             row.resize(mLabelParams.mRowNameIdx + 1);
1072 |         }
1073 | 
1074 |         mData.at(rowIdx).at(mLabelParams.mRowNameIdx) = pRowName;
1075 |     }
1076 | 
1077 |     /**
1078 |      * @brief   Get row names
1079 |      * @returns vector of row names.
1080 |      */
1081 |     std::vector<std::string> GetRowNames() {
1082 |         std::vector<std::string> rownames;
1083 |         if (mLabelParams.mRowNameIdx >= 0) {
1084 |             for (auto itRow = mData.begin(); itRow != mData.end(); ++itRow) {
1085 |                 if (std::distance(mData.begin(), itRow) > mLabelParams.mColumnNameIdx) {
1086 |                     rownames.push_back(itRow->at(mLabelParams.mRowNameIdx));
1087 |                 }
1088 |             }
1089 |         }
1090 |         return rownames;
1091 |     }
1092 | 
1093 | private:
1094 |     void ReadCsv() {
1095 |         std::ifstream stream;
1096 |         stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
1097 |         stream.open(mPath, std::ios::binary);
1098 |         ReadCsv(stream);
1099 |     }
1100 | 
1101 |     void ReadCsv(std::istream& pStream) {
1102 |         Clear();
1103 |         pStream.seekg(0, std::ios::end);
1104 |         std::streamsize length = pStream.tellg();
1105 |         pStream.seekg(0, std::ios::beg);
1106 | 
1107 | #ifdef HAS_CODECVT
1108 |         std::vector<char> bom2b(2, '\0');
1109 |         if (length >= 2) {
1110 |             pStream.read(bom2b.data(), 2);
1111 |             pStream.seekg(0, std::ios::beg);
1112 |         }
1113 | 
1114 |         static const std::vector<char> bomU16le = {'\xff', '\xfe'};
1115 |         static const std::vector<char> bomU16be = {'\xfe', '\xff'};
1116 |         if ((bom2b == bomU16le) || (bom2b == bomU16be)) {
1117 |             mIsUtf16 = true;
1118 |             mIsLE = (bom2b == bomU16le);
1119 | 
1120 |             std::wifstream wstream;
1121 |             wstream.exceptions(std::wifstream::failbit | std::wifstream::badbit);
1122 |             wstream.open(mPath, std::ios::binary);
1123 |             if (mIsLE) {
1124 |                 wstream.imbue(
1125 |                     std::locale(wstream.getloc(),
1126 |                                 new std::codecvt_utf16<wchar_t, 0x10ffff, static_cast<std::codecvt_mode>(std::consume_header | std::little_endian)>));
1127 |             } else {
1128 |                 wstream.imbue(std::locale(wstream.getloc(), new std::codecvt_utf16<wchar_t, 0x10ffff, std::consume_header>));
1129 |             }
1130 |             std::wstringstream wss;
1131 |             wss << wstream.rdbuf();
1132 |             std::string utf8 = ToString(wss.str());
1133 |             std::stringstream ss(utf8);
1134 |             ParseCsv(ss, utf8.size());
1135 |         } else
1136 | #endif
1137 |         {
1138 |             // check for UTF-8 Byte order mark and skip it when found
1139 |             if (length >= 3) {
1140 |                 std::vector<char> bom3b(3, '\0');
1141 |                 pStream.read(bom3b.data(), 3);
1142 |                 static const std::vector<char> bomU8 = {'\xef', '\xbb', '\xbf'};
1143 |                 if (bom3b != bomU8) {
1144 |                     // file does not start with a UTF-8 Byte order mark
1145 |                     pStream.seekg(0, std::ios::beg);
1146 |                 } else {
1147 |                     // file did start with a UTF-8 Byte order mark, simply skip it
1148 |                     length -= 3;
1149 |                 }
1150 |             }
1151 | 
1152 |             ParseCsv(pStream, length);
1153 |         }
1154 |     }
1155 | 
1156 |     void ParseCsv(std::istream& pStream, std::streamsize p_FileLength) {
1157 |         const std::streamsize bufLength = 64 * 1024;
1158 |         std::vector<char> buffer(bufLength);
1159 |         std::vector<std::string> row;
1160 |         std::string cell;
1161 |         bool quoted = false;
1162 |         int cr = 0;
1163 |         int lf = 0;
1164 | 
1165 |         while (p_FileLength > 0) {
1166 |             std::streamsize readLength = std::min<std::streamsize>(p_FileLength, bufLength);
1167 |             pStream.read(buffer.data(), readLength);
1168 |             for (int i = 0; i < readLength; ++i) {
1169 |                 if (buffer[i] == '"') {
1170 |                     if (cell.empty() || cell[0] == '"') {
1171 |                         quoted = !quoted;
1172 |                     }
1173 |                     cell += buffer[i];
1174 |                 } else if (buffer[i] == mSeparatorParams.mSeparator) {
1175 |                     if (!quoted) {
1176 |                         row.push_back(Unquote(Trim(cell)));
1177 |                         cell.clear();
1178 |                     } else {
1179 |                         cell += buffer[i];
1180 |                     }
1181 |                 } else if (buffer[i] == '\r') {
1182 |                     if (mSeparatorParams.mQuotedLinebreaks && quoted) {
1183 |                         cell += buffer[i];
1184 |                     } else {
1185 |                         ++cr;
1186 |                     }
1187 |                 } else if (buffer[i] == '\n') {
1188 |                     if (mSeparatorParams.mQuotedLinebreaks && quoted) {
1189 |                         cell += buffer[i];
1190 |                     } else {
1191 |                         ++lf;
1192 |                         row.push_back(Unquote(Trim(cell)));
1193 |                         cell.clear();
1194 |                         mData.push_back(row);
1195 |                         row.clear();
1196 |                         quoted = false;
1197 |                     }
1198 |                 } else {
1199 |                     cell += buffer[i];
1200 |                 }
1201 |             }
1202 |             p_FileLength -= readLength;
1203 |         }
1204 | 
1205 |         // Handle last line without linebreak
1206 |         if (!cell.empty() || !row.empty()) {
1207 |             row.push_back(Unquote(Trim(cell)));
1208 |             cell.clear();
1209 |             mData.push_back(row);
1210 |             row.clear();
1211 |         }
1212 | 
1213 |         // Assume CR/LF if at least half the linebreaks have CR
1214 |         mSeparatorParams.mHasCR = (cr > (lf / 2));
1215 | 
1216 |         // Set up column labels
1217 |         if ((mLabelParams.mColumnNameIdx >= 0) && (static_cast<ssize_t>(mData.size()) > mLabelParams.mColumnNameIdx)) {
1218 |             int i = 0;
1219 |             for (auto& columnName : mData[mLabelParams.mColumnNameIdx]) {
1220 |                 mColumnNames[columnName] = i++;
1221 |             }
1222 |         }
1223 | 
1224 |         // Set up row labels
1225 |         if ((mLabelParams.mRowNameIdx >= 0) && (static_cast<ssize_t>(mData.size()) > (mLabelParams.mColumnNameIdx + 1))) {
1226 |             int i = 0;
1227 |             for (auto& dataRow : mData) {
1228 |                 if (static_cast<ssize_t>(dataRow.size()) > mLabelParams.mRowNameIdx) {
1229 |                     mRowNames[dataRow[mLabelParams.mRowNameIdx]] = i++;
1230 |                 }
1231 |             }
1232 |         }
1233 |     }
1234 | 
1235 |     void WriteCsv() const {
1236 | #ifdef HAS_CODECVT
1237 |         if (mIsUtf16) {
1238 |             std::stringstream ss;
1239 |             WriteCsv(ss);
1240 |             std::string utf8 = ss.str();
1241 |             std::wstring wstr = ToWString(utf8);
1242 | 
1243 |             std::wofstream wstream;
1244 |             wstream.exceptions(std::wofstream::failbit | std::wofstream::badbit);
1245 |             wstream.open(mPath, std::ios::binary | std::ios::trunc);
1246 | 
1247 |             if (mIsLE) {
1248 |                 wstream.imbue(
1249 |                     std::locale(wstream.getloc(), new std::codecvt_utf16<wchar_t, 0x10ffff, static_cast<std::codecvt_mode>(std::little_endian)>));
1250 |             } else {
1251 |                 wstream.imbue(std::locale(wstream.getloc(), new std::codecvt_utf16<wchar_t, 0x10ffff>));
1252 |             }
1253 | 
1254 |             wstream << static_cast<wchar_t>(0xfeff);
1255 |             wstream << wstr;
1256 |         } else
1257 | #endif
1258 |         {
1259 |             std::ofstream stream;
1260 |             stream.exceptions(std::ofstream::failbit | std::ofstream::badbit);
1261 |             stream.open(mPath, std::ios::binary | std::ios::trunc);
1262 |             WriteCsv(stream);
1263 |         }
1264 |     }
1265 | 
1266 |     void WriteCsv(std::ostream& pStream) const {
1267 |         for (auto itr = mData.begin(); itr != mData.end(); ++itr) {
1268 |             for (auto itc = itr->begin(); itc != itr->end(); ++itc) {
1269 |                 if (mSeparatorParams.mAutoQuote &&
1270 |                     ((itc->find(mSeparatorParams.mSeparator) != std::string::npos) || (itc->find(' ') != std::string::npos))) {
1271 |                     // escape quotes in string
1272 |                     std::string str = *itc;
1273 |                     ReplaceString(str, "\"", "\"\"");
1274 | 
1275 |                     pStream << "\"" << str << "\"";
1276 |                 } else {
1277 |                     pStream << *itc;
1278 |                 }
1279 | 
1280 |                 if (std::distance(itc, itr->end()) > 1) {
1281 |                     pStream << mSeparatorParams.mSeparator;
1282 |                 }
1283 |             }
1284 |             pStream << (mSeparatorParams.mHasCR ? "\r\n" : "\n");
1285 |         }
1286 |     }
1287 | 
1288 |     size_t GetDataRowCount() const { return mData.size(); }
1289 | 
1290 |     size_t GetDataColumnCount() const { return (mData.size() > 0) ? mData.at(0).size() : 0; }
1291 | 
1292 |     std::string Trim(const std::string& pStr) {
1293 |         if (mSeparatorParams.mTrim) {
1294 |             std::string str = pStr;
1295 | 
1296 |             // ltrim
1297 |             str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](int ch) { return !isspace(ch); }));
1298 | 
1299 |             // rtrim
1300 |             str.erase(std::find_if(str.rbegin(), str.rend(), [](int ch) { return !isspace(ch); }).base(), str.end());
1301 | 
1302 |             return str;
1303 |         } else {
1304 |             return pStr;
1305 |         }
1306 |     }
1307 | 
1308 |     std::string Unquote(const std::string& pStr) {
1309 |         if (mSeparatorParams.mAutoQuote && (pStr.size() >= 2) && (pStr.front() == '"') && (pStr.back() == '"')) {
1310 |             // remove start/end quotes
1311 |             std::string str = pStr.substr(1, pStr.size() - 2);
1312 | 
1313 |             // unescape quotes in string
1314 |             ReplaceString(str, "\"\"", "\"");
1315 | 
1316 |             return str;
1317 |         } else {
1318 |             return pStr;
1319 |         }
1320 |     }
1321 | 
1322 | #ifdef HAS_CODECVT
1323 | #if defined(_MSC_VER)
1324 | #pragma warning(disable : 4996)
1325 | #endif
1326 |     static std::string ToString(const std::wstring& pWStr) {
1327 |         size_t len = std::wcstombs(nullptr, pWStr.c_str(), 0) + 1;
1328 |         char* cstr = new char[len];
1329 |         std::wcstombs(cstr, pWStr.c_str(), len);
1330 |         std::string str(cstr);
1331 |         delete[] cstr;
1332 |         return str;
1333 |     }
1334 | 
1335 |     static std::wstring ToWString(const std::string& pStr) {
1336 |         size_t len = 1 + mbstowcs(nullptr, pStr.c_str(), 0);
1337 |         wchar_t* wcstr = new wchar_t[len];
1338 |         std::mbstowcs(wcstr, pStr.c_str(), len);
1339 |         std::wstring wstr(wcstr);
1340 |         delete[] wcstr;
1341 |         return wstr;
1342 |     }
1343 | #if defined(_MSC_VER)
1344 | #pragma warning(default : 4996)
1345 | #endif
1346 | #endif
1347 | 
1348 |     static void ReplaceString(std::string& pStr, const std::string& pSearch, const std::string& pReplace) {
1349 |         size_t pos = 0;
1350 | 
1351 |         while ((pos = pStr.find(pSearch, pos)) != std::string::npos) {
1352 |             pStr.replace(pos, pSearch.size(), pReplace);
1353 |             pos += pReplace.size();
1354 |         }
1355 |     }
1356 | 
1357 | private:
1358 |     std::string mPath;
1359 |     LabelParams mLabelParams;
1360 |     SeparatorParams mSeparatorParams;
1361 |     ConverterParams mConverterParams;
1362 |     std::vector<std::vector<std::string>> mData;
1363 |     std::map<std::string, size_t> mColumnNames;
1364 |     std::map<std::string, size_t> mRowNames;
1365 | #ifdef HAS_CODECVT
1366 |     bool mIsUtf16 = false;
1367 |     bool mIsLE = false;
1368 | #endif
1369 | };
1370 | }  // namespace rapidcsv

```

`src\reference_waypoint_loader\include\reference_waypoint_loader\reference_waypoint_loader.hpp`:

```hpp
   1 | #pragma once
   2 | 
   3 | #include <algorithm>
   4 | #include <filesystem>
   5 | #include <vector>
   6 | 
   7 | #include <ros/console.h>
   8 | #include <ros/ros.h>
   9 | 
  10 | #include <nav_msgs/Path.h>
  11 | #include <visualization_msgs/MarkerArray.h>
  12 | #include <waypoint_msgs/Waypoint.h>  // custom msg
  13 | 
  14 | #include "reference_waypoint_loader/rapidcsv.h"
  15 | #include "reference_waypoint_loader/tinycolormap.hpp"
  16 | 
  17 | namespace planning {
  18 | 
  19 | class ReferenceWaypointLoader {
  20 | public:
  21 |     ReferenceWaypointLoader();
  22 |     ~ReferenceWaypointLoader(){};
  23 | 
  24 | private:
  25 |     ros::NodeHandle nh_;
  26 |     ros::NodeHandle private_nh_;
  27 | 
  28 |     /* publisher */
  29 |     ros::Publisher reference_waypoint_pub_;
  30 |     ros::Publisher reference_path_pub_;
  31 |     ros::Publisher reference_rvizmarker_pub_;
  32 | 
  33 |     /* system params */
  34 |     std::string csv_waypoint_;
  35 |     std::string map_frame_;
  36 |     std::string topic_name_waypoint_;
  37 |     std::string topic_name_path_;
  38 |     std::string topic_name_rviz_waypoint_marker_;
  39 |     std::string ref_x_label_;
  40 |     std::string ref_y_label_;
  41 |     std::string ref_v_label_;
  42 |     float ref_v_scale_;
  43 |     float scaled_v_min_;
  44 |     float scaled_v_max_;
  45 | };
  46 | 
  47 | }  // namespace planning

```

`src\reference_waypoint_loader\include\reference_waypoint_loader\tinycolormap.hpp`:

```hpp
   1 | /*
   2 |  MIT License
   3 | 
   4 |  Copyright (c) 2018-2020 Yuki Koyama
   5 | 
   6 |  Permission is hereby granted, free of charge, to any person obtaining a copy
   7 |  of this software and associated documentation files (the "Software"), to deal
   8 |  in the Software without restriction, including without limitation the rights
   9 |  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  10 |  copies of the Software, and to permit persons to whom the Software is
  11 |  furnished to do so, subject to the following conditions:
  12 | 
  13 |  The above copyright notice and this permission notice shall be included in all
  14 |  copies or substantial portions of the Software.
  15 | 
  16 |  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  17 |  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  18 |  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  19 |  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  20 |  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  21 |  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  22 |  SOFTWARE.
  23 | 
  24 |  -------------------------------------------------------------------------------
  25 | 
  26 |  The lookup table for Turbo is derived by Shot511 in his PR,
  27 |  https://github.com/yuki-koyama/tinycolormap/pull/27 , from
  28 |  https://gist.github.com/mikhailov-work/6a308c20e494d9e0ccc29036b28faa7a , which
  29 |  is released by Anton Mikhailov, copyrighted by Google LLC, and licensed under
  30 |  the Apache 2.0 license. To the best of our knowledge, the Apache 2.0 license is
  31 |  compatible with the MIT license, and thus we release the merged entire code
  32 |  under the MIT license. The license notice for Anton's code is posted here:
  33 | 
  34 |  Copyright 2019 Google LLC.
  35 | 
  36 |  Licensed under the Apache License, Version 2.0 (the "License");
  37 |  you may not use this file except in compliance with the License.
  38 |  You may obtain a copy of the License at
  39 |      http://www.apache.org/licenses/LICENSE-2.0
  40 |  Unless required by applicable law or agreed to in writing, software
  41 |  distributed under the License is distributed on an "AS IS" BASIS,
  42 |  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  43 |  See the License for the specific language governing permissions and
  44 |  limitations under the License.
  45 | 
  46 |  */
  47 | 
  48 | #ifndef TINYCOLORMAP_HPP_
  49 | #define TINYCOLORMAP_HPP_
  50 | 
  51 | #include <algorithm>
  52 | #include <cmath>
  53 | #include <cstdint>
  54 | 
  55 | #if defined(TINYCOLORMAP_WITH_EIGEN)
  56 | #include <Eigen/Core>
  57 | #endif
  58 | 
  59 | #if defined(TINYCOLORMAP_WITH_QT5)
  60 | #include <QColor>
  61 | #endif
  62 | 
  63 | #if defined(TINYCOLORMAP_WITH_QT5) && defined(TINYCOLORMAP_WITH_EIGEN)
  64 | #include <QImage>
  65 | #include <QString>
  66 | #endif
  67 | 
  68 | #if defined(TINYCOLORMAP_WITH_GLM)
  69 | #include <glm/vec3.hpp>
  70 | #endif
  71 | 
  72 | namespace tinycolormap {
  73 | //////////////////////////////////////////////////////////////////////////////////
  74 | // Interface
  75 | //////////////////////////////////////////////////////////////////////////////////
  76 | 
  77 | enum class ColormapType { Parula, Heat, Jet, Turbo, Hot, Gray, Magma, Inferno, Plasma, Viridis, Cividis, Github, Cubehelix, HSV };
  78 | 
  79 | struct Color {
  80 |     explicit constexpr Color(double gray) noexcept : data{gray, gray, gray} {}
  81 |     constexpr Color(double r, double g, double b) noexcept : data{r, g, b} {}
  82 | 
  83 |     double data[3];
  84 | 
  85 |     double& r() noexcept { return data[0]; }
  86 |     double& g() noexcept { return data[1]; }
  87 |     double& b() noexcept { return data[2]; }
  88 |     constexpr double r() const noexcept { return data[0]; }
  89 |     constexpr double g() const noexcept { return data[1]; }
  90 |     constexpr double b() const noexcept { return data[2]; }
  91 | 
  92 |     constexpr uint8_t ri() const noexcept { return static_cast<uint8_t>(data[0] * 255.0); }
  93 |     constexpr uint8_t gi() const noexcept { return static_cast<uint8_t>(data[1] * 255.0); }
  94 |     constexpr uint8_t bi() const noexcept { return static_cast<uint8_t>(data[2] * 255.0); }
  95 | 
  96 |     double& operator[](std::size_t n) noexcept { return data[n]; }
  97 |     constexpr double operator[](std::size_t n) const noexcept { return data[n]; }
  98 |     double& operator()(std::size_t n) noexcept { return data[n]; }
  99 |     constexpr double operator()(std::size_t n) const noexcept { return data[n]; }
 100 | 
 101 |     friend constexpr Color operator+(const Color& c0, const Color& c1) noexcept { return {c0.r() + c1.r(), c0.g() + c1.g(), c0.b() + c1.b()}; }
 102 | 
 103 |     friend constexpr Color operator*(double s, const Color& c) noexcept { return {s * c.r(), s * c.g(), s * c.b()}; }
 104 | 
 105 | #if defined(TINYCOLORMAP_WITH_QT5)
 106 |     QColor ConvertToQColor() const { return QColor(data[0] * 255.0, data[1] * 255.0, data[2] * 255.0); }
 107 | #endif
 108 | #if defined(TINYCOLORMAP_WITH_EIGEN)
 109 |     Eigen::Vector3d ConvertToEigen() const { return Eigen::Vector3d(data[0], data[1], data[2]); }
 110 | #endif
 111 | #if defined(TINYCOLORMAP_WITH_GLM)
 112 |     glm::vec3 ConvertToGLM() const { return glm::vec3(data[0], data[1], data[2]); }
 113 | #endif
 114 | };
 115 | 
 116 | inline Color GetColor(double x, ColormapType type = ColormapType::Viridis);
 117 | inline Color GetQuantizedColor(double x, unsigned int num_levels, ColormapType type = ColormapType::Viridis);
 118 | inline Color GetParulaColor(double x);
 119 | inline Color GetHeatColor(double x);
 120 | inline Color GetJetColor(double x);
 121 | inline Color GetTurboColor(double x);
 122 | inline Color GetHotColor(double x);
 123 | inline constexpr Color GetGrayColor(double x) noexcept;
 124 | inline Color GetMagmaColor(double x);
 125 | inline Color GetInfernoColor(double x);
 126 | inline Color GetPlasmaColor(double x);
 127 | inline Color GetViridisColor(double x);
 128 | inline Color GetCividisColor(double x);
 129 | inline Color GetGithubColor(double x);
 130 | inline Color GetCubehelixColor(double x);
 131 | inline Color GetHSVColor(double x);
 132 | 
 133 | #if defined(TINYCOLORMAP_WITH_QT5) && defined(TINYCOLORMAP_WITH_EIGEN)
 134 | inline QImage CreateMatrixVisualization(const Eigen::MatrixXd& matrix);
 135 | inline void ExportMatrixVisualization(const Eigen::MatrixXd& matrix, const std::string& path);
 136 | #endif
 137 | 
 138 | //////////////////////////////////////////////////////////////////////////////////
 139 | // Private Implementation - public usage is not intended
 140 | //////////////////////////////////////////////////////////////////////////////////
 141 | 
 142 | namespace internal {
 143 |     inline constexpr double Clamp01(double x) noexcept { return (x < 0.0) ? 0.0 : (x > 1.0) ? 1.0 : x; }
 144 | 
 145 |     // A helper function to calculate linear interpolation
 146 |     template <std::size_t N>
 147 |     Color CalcLerp(double x, const Color (&data)[N]) {
 148 |         const double a = Clamp01(x) * (N - 1);
 149 |         const double i = std::floor(a);
 150 |         const double t = a - i;
 151 |         const Color& c0 = data[static_cast<std::size_t>(i)];
 152 |         const Color& c1 = data[static_cast<std::size_t>(std::ceil(a))];
 153 | 
 154 |         return (1.0 - t) * c0 + t * c1;
 155 |     }
 156 | 
 157 |     inline double QuantizeArgument(double x, unsigned int num_levels) {
 158 |         // Clamp num_classes to range [1, 255].
 159 |         num_levels = (std::max)(1u, (std::min)(num_levels, 255u));
 160 | 
 161 |         const double interval_length = 255.0 / num_levels;
 162 | 
 163 |         // Calculate index of the interval to which the given x belongs to.
 164 |         // Substracting eps prevents getting out of bounds index.
 165 |         constexpr double eps = 0.0005;
 166 |         const unsigned int index = static_cast<unsigned int>((x * 255.0 - eps) / interval_length);
 167 | 
 168 |         // Calculate upper and lower bounds of the given interval.
 169 |         const unsigned int upper_boundary = static_cast<unsigned int>(index * interval_length + interval_length);
 170 |         const unsigned int lower_boundary = static_cast<unsigned int>(upper_boundary - interval_length);
 171 | 
 172 |         // Get middle "coordinate" of the given interval and move it back to [0.0, 1.0] interval.
 173 |         const double xx = static_cast<double>(upper_boundary + lower_boundary) * 0.5 / 255.0;
 174 | 
 175 |         return xx;
 176 |     }
 177 | }  // namespace internal
 178 | 
 179 | //////////////////////////////////////////////////////////////////////////////////
 180 | // Public Implementation
 181 | //////////////////////////////////////////////////////////////////////////////////
 182 | 
 183 | inline Color GetColor(double x, ColormapType type) {
 184 |     switch (type) {
 185 |         case ColormapType::Parula:
 186 |             return GetParulaColor(x);
 187 |         case ColormapType::Heat:
 188 |             return GetHeatColor(x);
 189 |         case ColormapType::Jet:
 190 |             return GetJetColor(x);
 191 |         case ColormapType::Turbo:
 192 |             return GetTurboColor(x);
 193 |         case ColormapType::Hot:
 194 |             return GetHotColor(x);
 195 |         case ColormapType::Gray:
 196 |             return GetGrayColor(x);
 197 |         case ColormapType::Magma:
 198 |             return GetMagmaColor(x);
 199 |         case ColormapType::Inferno:
 200 |             return GetInfernoColor(x);
 201 |         case ColormapType::Plasma:
 202 |             return GetPlasmaColor(x);
 203 |         case ColormapType::Viridis:
 204 |             return GetViridisColor(x);
 205 |         case ColormapType::Cividis:
 206 |             return GetCividisColor(x);
 207 |         case ColormapType::Github:
 208 |             return GetGithubColor(x);
 209 |         case ColormapType::Cubehelix:
 210 |             return GetCubehelixColor(x);
 211 |         case ColormapType::HSV:
 212 |             return GetHSVColor(x);
 213 |         default:
 214 |             break;
 215 |     }
 216 | 
 217 |     return GetViridisColor(x);
 218 | }
 219 | 
 220 | inline Color GetQuantizedColor(double x, unsigned int num_levels, ColormapType type) {
 221 |     return GetColor(internal::QuantizeArgument(x, num_levels), type);
 222 | }
 223 | 
 224 | inline Color GetParulaColor(double x) {
 225 |     constexpr Color data[] = {{0.2081, 0.1663, 0.5292},
 226 |                               {0.2091, 0.1721, 0.5411},
 227 |                               {0.2101, 0.1779, 0.553},
 228 |                               {0.2109, 0.1837, 0.565},
 229 |                               {0.2116, 0.1895, 0.5771},
 230 |                               {0.2121, 0.1954, 0.5892},
 231 |                               {0.2124, 0.2013, 0.6013},
 232 |                               {0.2125, 0.2072, 0.6135},
 233 |                               {0.2123, 0.2132, 0.6258},
 234 |                               {0.2118, 0.2192, 0.6381},
 235 |                               {0.2111, 0.2253, 0.6505},
 236 |                               {0.2099, 0.2315, 0.6629},
 237 |                               {0.2084, 0.2377, 0.6753},
 238 |                               {0.2063, 0.244, 0.6878},
 239 |                               {0.2038, 0.2503, 0.7003},
 240 |                               {0.2006, 0.2568, 0.7129},
 241 |                               {0.1968, 0.2632, 0.7255},
 242 |                               {0.1921, 0.2698, 0.7381},
 243 |                               {0.1867, 0.2764, 0.7507},
 244 |                               {0.1802, 0.2832, 0.7634},
 245 |                               {0.1728, 0.2902, 0.7762},
 246 |                               {0.1641, 0.2975, 0.789},
 247 |                               {0.1541, 0.3052, 0.8017},
 248 |                               {0.1427, 0.3132, 0.8145},
 249 |                               {0.1295, 0.3217, 0.8269},
 250 |                               {0.1147, 0.3306, 0.8387},
 251 |                               {0.0986, 0.3397, 0.8495},
 252 |                               {0.0816, 0.3486, 0.8588},
 253 |                               {0.0646, 0.3572, 0.8664},
 254 |                               {0.0482, 0.3651, 0.8722},
 255 |                               {0.0329, 0.3724, 0.8765},
 256 |                               {0.0213, 0.3792, 0.8796},
 257 |                               {0.0136, 0.3853, 0.8815},
 258 |                               {0.0086, 0.3911, 0.8827},
 259 |                               {0.006, 0.3965, 0.8833},
 260 |                               {0.0051, 0.4017, 0.8834},
 261 |                               {0.0054, 0.4066, 0.8831},
 262 |                               {0.0067, 0.4113, 0.8825},
 263 |                               {0.0089, 0.4159, 0.8816},
 264 |                               {0.0116, 0.4203, 0.8805},
 265 |                               {0.0148, 0.4246, 0.8793},
 266 |                               {0.0184, 0.4288, 0.8779},
 267 |                               {0.0223, 0.4329, 0.8763},
 268 |                               {0.0264, 0.437, 0.8747},
 269 |                               {0.0306, 0.441, 0.8729},
 270 |                               {0.0349, 0.4449, 0.8711},
 271 |                               {0.0394, 0.4488, 0.8692},
 272 |                               {0.0437, 0.4526, 0.8672},
 273 |                               {0.0477, 0.4564, 0.8652},
 274 |                               {0.0514, 0.4602, 0.8632},
 275 |                               {0.0549, 0.464, 0.8611},
 276 |                               {0.0582, 0.4677, 0.8589},
 277 |                               {0.0612, 0.4714, 0.8568},
 278 |                               {0.064, 0.4751, 0.8546},
 279 |                               {0.0666, 0.4788, 0.8525},
 280 |                               {0.0689, 0.4825, 0.8503},
 281 |                               {0.071, 0.4862, 0.8481},
 282 |                               {0.0729, 0.4899, 0.846},
 283 |                               {0.0746, 0.4937, 0.8439},
 284 |                               {0.0761, 0.4974, 0.8418},
 285 |                               {0.0773, 0.5012, 0.8398},
 286 |                               {0.0782, 0.5051, 0.8378},
 287 |                               {0.0789, 0.5089, 0.8359},
 288 |                               {0.0794, 0.5129, 0.8341},
 289 |                               {0.0795, 0.5169, 0.8324},
 290 |                               {0.0793, 0.521, 0.8308},
 291 |                               {0.0788, 0.5251, 0.8293},
 292 |                               {0.0778, 0.5295, 0.828},
 293 |                               {0.0764, 0.5339, 0.827},
 294 |                               {0.0746, 0.5384, 0.8261},
 295 |                               {0.0724, 0.5431, 0.8253},
 296 |                               {0.0698, 0.5479, 0.8247},
 297 |                               {0.0668, 0.5527, 0.8243},
 298 |                               {0.0636, 0.5577, 0.8239},
 299 |                               {0.06, 0.5627, 0.8237},
 300 |                               {0.0562, 0.5677, 0.8234},
 301 |                               {0.0523, 0.5727, 0.8231},
 302 |                               {0.0484, 0.5777, 0.8228},
 303 |                               {0.0445, 0.5826, 0.8223},
 304 |                               {0.0408, 0.5874, 0.8217},
 305 |                               {0.0372, 0.5922, 0.8209},
 306 |                               {0.0342, 0.5968, 0.8198},
 307 |                               {0.0317, 0.6012, 0.8186},
 308 |                               {0.0296, 0.6055, 0.8171},
 309 |                               {0.0279, 0.6097, 0.8154},
 310 |                               {0.0265, 0.6137, 0.8135},
 311 |                               {0.0255, 0.6176, 0.8114},
 312 |                               {0.0248, 0.6214, 0.8091},
 313 |                               {0.0243, 0.625, 0.8066},
 314 |                               {0.0239, 0.6285, 0.8039},
 315 |                               {0.0237, 0.6319, 0.801},
 316 |                               {0.0235, 0.6352, 0.798},
 317 |                               {0.0233, 0.6384, 0.7948},
 318 |                               {0.0231, 0.6415, 0.7916},
 319 |                               {0.023, 0.6445, 0.7881},
 320 |                               {0.0229, 0.6474, 0.7846},
 321 |                               {
 322 |                                   0.0227,
 323 |                                   0.6503,
 324 |                                   0.781,
 325 |                               },
 326 |                               {0.0227, 0.6531, 0.7773},
 327 |                               {0.0232, 0.6558, 0.7735},
 328 |                               {0.0238, 0.6585, 0.7696},
 329 |                               {0.0246, 0.6611, 0.7656},
 330 |                               {0.0263, 0.6637, 0.7615},
 331 |                               {0.0282, 0.6663, 0.7574},
 332 |                               {0.0306, 0.6688, 0.7532},
 333 |                               {0.0338, 0.6712, 0.749},
 334 |                               {0.0373, 0.6737, 0.7446},
 335 |                               {0.0418, 0.6761, 0.7402},
 336 |                               {0.0467, 0.6784, 0.7358},
 337 |                               {0.0516, 0.6808, 0.7313},
 338 |                               {0.0574, 0.6831, 0.7267},
 339 |                               {0.0629, 0.6854, 0.7221},
 340 |                               {0.0692, 0.6877, 0.7173},
 341 |                               {0.0755, 0.6899, 0.7126},
 342 |                               {0.082, 0.6921, 0.7078},
 343 |                               {0.0889, 0.6943, 0.7029},
 344 |                               {0.0956, 0.6965, 0.6979},
 345 |                               {0.1031, 0.6986, 0.6929},
 346 |                               {0.1104, 0.7007, 0.6878},
 347 |                               {0.118, 0.7028, 0.6827},
 348 |                               {0.1258, 0.7049, 0.6775},
 349 |                               {0.1335, 0.7069, 0.6723},
 350 |                               {0.1418, 0.7089, 0.6669},
 351 |                               {0.1499, 0.7109, 0.6616},
 352 |                               {0.1585, 0.7129, 0.6561},
 353 |                               {0.1671, 0.7148, 0.6507},
 354 |                               {0.1758, 0.7168, 0.6451},
 355 |                               {0.1849, 0.7186, 0.6395},
 356 |                               {0.1938, 0.7205, 0.6338},
 357 |                               {0.2033, 0.7223, 0.6281},
 358 |                               {0.2128, 0.7241, 0.6223},
 359 |                               {0.2224, 0.7259, 0.6165},
 360 |                               {0.2324, 0.7275, 0.6107},
 361 |                               {0.2423, 0.7292, 0.6048},
 362 |                               {0.2527, 0.7308, 0.5988},
 363 |                               {0.2631, 0.7324, 0.5929},
 364 |                               {0.2735, 0.7339, 0.5869},
 365 |                               {0.2845, 0.7354, 0.5809},
 366 |                               {0.2953, 0.7368, 0.5749},
 367 |                               {0.3064, 0.7381, 0.5689},
 368 |                               {0.3177, 0.7394, 0.563},
 369 |                               {0.3289, 0.7406, 0.557},
 370 |                               {0.3405, 0.7417, 0.5512},
 371 |                               {0.352, 0.7428, 0.5453},
 372 |                               {0.3635, 0.7438, 0.5396},
 373 |                               {0.3753, 0.7446, 0.5339},
 374 |                               {0.3869, 0.7454, 0.5283},
 375 |                               {0.3986, 0.7461, 0.5229},
 376 |                               {0.4103, 0.7467, 0.5175},
 377 |                               {0.4218, 0.7473, 0.5123},
 378 |                               {0.4334, 0.7477, 0.5072},
 379 |                               {0.4447, 0.7482, 0.5021},
 380 |                               {0.4561, 0.7485, 0.4972},
 381 |                               {0.4672, 0.7487, 0.4924},
 382 |                               {0.4783, 0.7489, 0.4877},
 383 |                               {0.4892, 0.7491, 0.4831},
 384 |                               {0.5, 0.7491, 0.4786},
 385 |                               {0.5106, 0.7492, 0.4741},
 386 |                               {0.5212, 0.7492, 0.4698},
 387 |                               {0.5315, 0.7491, 0.4655},
 388 |                               {0.5418, 0.749, 0.4613},
 389 |                               {0.5519, 0.7489, 0.4571},
 390 |                               {0.5619, 0.7487, 0.4531},
 391 |                               {0.5718, 0.7485, 0.449},
 392 |                               {0.5816, 0.7482, 0.4451},
 393 |                               {0.5913, 0.7479, 0.4412},
 394 |                               {0.6009, 0.7476, 0.4374},
 395 |                               {0.6103, 0.7473, 0.4335},
 396 |                               {0.6197, 0.7469, 0.4298},
 397 |                               {0.629, 0.7465, 0.4261},
 398 |                               {0.6382, 0.746, 0.4224},
 399 |                               {0.6473, 0.7456, 0.4188},
 400 |                               {0.6564, 0.7451, 0.4152},
 401 |                               {0.6653, 0.7446, 0.4116},
 402 |                               {0.6742, 0.7441, 0.4081},
 403 |                               {0.683, 0.7435, 0.4046},
 404 |                               {0.6918, 0.743, 0.4011},
 405 |                               {0.7004, 0.7424, 0.3976},
 406 |                               {0.7091, 0.7418, 0.3942},
 407 |                               {0.7176, 0.7412, 0.3908},
 408 |                               {0.7261, 0.7405, 0.3874},
 409 |                               {0.7346, 0.7399, 0.384},
 410 |                               {0.743, 0.7392, 0.3806},
 411 |                               {0.7513, 0.7385, 0.3773},
 412 |                               {0.7596, 0.7378, 0.3739},
 413 |                               {0.7679, 0.7372, 0.3706},
 414 |                               {0.7761, 0.7364, 0.3673},
 415 |                               {0.7843, 0.7357, 0.3639},
 416 |                               {0.7924, 0.735, 0.3606},
 417 |                               {0.8005, 0.7343, 0.3573},
 418 |                               {0.8085, 0.7336, 0.3539},
 419 |                               {0.8166, 0.7329, 0.3506},
 420 |                               {0.8246, 0.7322, 0.3472},
 421 |                               {0.8325, 0.7315, 0.3438},
 422 |                               {0.8405, 0.7308, 0.3404},
 423 |                               {0.8484, 0.7301, 0.337},
 424 |                               {0.8563, 0.7294, 0.3336},
 425 |                               {0.8642, 0.7288, 0.33},
 426 |                               {0.872, 0.7282, 0.3265},
 427 |                               {0.8798, 0.7276, 0.3229},
 428 |                               {0.8877, 0.7271, 0.3193},
 429 |                               {0.8954, 0.7266, 0.3156},
 430 |                               {0.9032, 0.7262, 0.3117},
 431 |                               {0.911, 0.7259, 0.3078},
 432 |                               {0.9187, 0.7256, 0.3038},
 433 |                               {0.9264, 0.7256, 0.2996},
 434 |                               {0.9341, 0.7256, 0.2953},
 435 |                               {0.9417, 0.7259, 0.2907},
 436 |                               {0.9493, 0.7264, 0.2859},
 437 |                               {0.9567, 0.7273, 0.2808},
 438 |                               {0.9639, 0.7285, 0.2754},
 439 |                               {0.9708, 0.7303, 0.2696},
 440 |                               {0.9773, 0.7326, 0.2634},
 441 |                               {0.9831, 0.7355, 0.257},
 442 |                               {0.9882, 0.739, 0.2504},
 443 |                               {0.9922, 0.7431, 0.2437},
 444 |                               {0.9952, 0.7476, 0.2373},
 445 |                               {0.9973, 0.7524, 0.231},
 446 |                               {0.9986, 0.7573, 0.2251},
 447 |                               {0.9991, 0.7624, 0.2195},
 448 |                               {0.999, 0.7675, 0.2141},
 449 |                               {0.9985, 0.7726, 0.209},
 450 |                               {0.9976, 0.7778, 0.2042},
 451 |                               {0.9964, 0.7829, 0.1995},
 452 |                               {0.995, 0.788, 0.1949},
 453 |                               {0.9933, 0.7931, 0.1905},
 454 |                               {0.9914, 0.7981, 0.1863},
 455 |                               {0.9894, 0.8032, 0.1821},
 456 |                               {0.9873, 0.8083, 0.178},
 457 |                               {0.9851, 0.8133, 0.174},
 458 |                               {0.9828, 0.8184, 0.17},
 459 |                               {0.9805, 0.8235, 0.1661},
 460 |                               {0.9782, 0.8286, 0.1622},
 461 |                               {0.9759, 0.8337, 0.1583},
 462 |                               {0.9736, 0.8389, 0.1544},
 463 |                               {0.9713, 0.8441, 0.1505},
 464 |                               {0.9692, 0.8494, 0.1465},
 465 |                               {0.9672, 0.8548, 0.1425},
 466 |                               {0.9654, 0.8603, 0.1385},
 467 |                               {0.9638, 0.8659, 0.1343},
 468 |                               {0.9623, 0.8716, 0.1301},
 469 |                               {0.9611, 0.8774, 0.1258},
 470 |                               {0.96, 0.8834, 0.1215},
 471 |                               {0.9593, 0.8895, 0.1171},
 472 |                               {0.9588, 0.8958, 0.1126},
 473 |                               {0.9586, 0.9022, 0.1082},
 474 |                               {0.9587, 0.9088, 0.1036},
 475 |                               {0.9591, 0.9155, 0.099},
 476 |                               {0.9599, 0.9225, 0.0944},
 477 |                               {0.961, 0.9296, 0.0897},
 478 |                               {0.9624, 0.9368, 0.085},
 479 |                               {0.9641, 0.9443, 0.0802},
 480 |                               {0.9662, 0.9518, 0.0753},
 481 |                               {0.9685, 0.9595, 0.0703},
 482 |                               {0.971, 0.9673, 0.0651},
 483 |                               {0.9736, 0.9752, 0.0597},
 484 |                               {0.9763, 0.9831, 0.0538}};
 485 | 
 486 |     return internal::CalcLerp(x, data);
 487 | }
 488 | 
 489 | inline Color GetHeatColor(double x) {
 490 |     constexpr Color data[] = {{0.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 0.0}};
 491 | 
 492 |     return internal::CalcLerp(x, data);
 493 | }
 494 | 
 495 | inline Color GetJetColor(double x) {
 496 |     constexpr Color data[] = {{0.0, 0.0, 0.5}, {0.0, 0.0, 1.0}, {0.0, 0.5, 1.0}, {0.0, 1.0, 1.0}, {0.5, 1.0, 0.5},
 497 |                               {1.0, 1.0, 0.0}, {1.0, 0.5, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
 498 | 
 499 |     return internal::CalcLerp(x, data);
 500 | }
 501 | 
 502 | inline Color GetTurboColor(double x) {
 503 |     constexpr Color data[] = {{0.18995, 0.07176, 0.23217}, {0.19483, 0.08339, 0.26149}, {0.19956, 0.09498, 0.29024}, {0.20415, 0.10652, 0.31844},
 504 |                               {0.20860, 0.11802, 0.34607}, {0.21291, 0.12947, 0.37314}, {0.21708, 0.14087, 0.39964}, {0.22111, 0.15223, 0.42558},
 505 |                               {0.22500, 0.16354, 0.45096}, {0.22875, 0.17481, 0.47578}, {0.23236, 0.18603, 0.50004}, {0.23582, 0.19720, 0.52373},
 506 |                               {0.23915, 0.20833, 0.54686}, {0.24234, 0.21941, 0.56942}, {0.24539, 0.23044, 0.59142}, {0.24830, 0.24143, 0.61286},
 507 |                               {0.25107, 0.25237, 0.63374}, {0.25369, 0.26327, 0.65406}, {0.25618, 0.27412, 0.67381}, {0.25853, 0.28492, 0.69300},
 508 |                               {0.26074, 0.29568, 0.71162}, {0.26280, 0.30639, 0.72968}, {0.26473, 0.31706, 0.74718}, {0.26652, 0.32768, 0.76412},
 509 |                               {0.26816, 0.33825, 0.78050}, {0.26967, 0.34878, 0.79631}, {0.27103, 0.35926, 0.81156}, {0.27226, 0.36970, 0.82624},
 510 |                               {0.27334, 0.38008, 0.84037}, {0.27429, 0.39043, 0.85393}, {0.27509, 0.40072, 0.86692}, {0.27576, 0.41097, 0.87936},
 511 |                               {0.27628, 0.42118, 0.89123}, {0.27667, 0.43134, 0.90254}, {0.27691, 0.44145, 0.91328}, {0.27701, 0.45152, 0.92347},
 512 |                               {0.27698, 0.46153, 0.93309}, {0.27680, 0.47151, 0.94214}, {0.27648, 0.48144, 0.95064}, {0.27603, 0.49132, 0.95857},
 513 |                               {0.27543, 0.50115, 0.96594}, {0.27469, 0.51094, 0.97275}, {0.27381, 0.52069, 0.97899}, {0.27273, 0.53040, 0.98461},
 514 |                               {0.27106, 0.54015, 0.98930}, {0.26878, 0.54995, 0.99303}, {0.26592, 0.55979, 0.99583}, {0.26252, 0.56967, 0.99773},
 515 |                               {0.25862, 0.57958, 0.99876}, {0.25425, 0.58950, 0.99896}, {0.24946, 0.59943, 0.99835}, {0.24427, 0.60937, 0.99697},
 516 |                               {0.23874, 0.61931, 0.99485}, {0.23288, 0.62923, 0.99202}, {0.22676, 0.63913, 0.98851}, {0.22039, 0.64901, 0.98436},
 517 |                               {0.21382, 0.65886, 0.97959}, {0.20708, 0.66866, 0.97423}, {0.20021, 0.67842, 0.96833}, {0.19326, 0.68812, 0.96190},
 518 |                               {0.18625, 0.69775, 0.95498}, {0.17923, 0.70732, 0.94761}, {0.17223, 0.71680, 0.93981}, {0.16529, 0.72620, 0.93161},
 519 |                               {0.15844, 0.73551, 0.92305}, {0.15173, 0.74472, 0.91416}, {0.14519, 0.75381, 0.90496}, {0.13886, 0.76279, 0.89550},
 520 |                               {0.13278, 0.77165, 0.88580}, {0.12698, 0.78037, 0.87590}, {0.12151, 0.78896, 0.86581}, {0.11639, 0.79740, 0.85559},
 521 |                               {0.11167, 0.80569, 0.84525}, {0.10738, 0.81381, 0.83484}, {0.10357, 0.82177, 0.82437}, {0.10026, 0.82955, 0.81389},
 522 |                               {0.09750, 0.83714, 0.80342}, {0.09532, 0.84455, 0.79299}, {0.09377, 0.85175, 0.78264}, {0.09287, 0.85875, 0.77240},
 523 |                               {0.09267, 0.86554, 0.76230}, {0.09320, 0.87211, 0.75237}, {0.09451, 0.87844, 0.74265}, {0.09662, 0.88454, 0.73316},
 524 |                               {0.09958, 0.89040, 0.72393}, {0.10342, 0.89600, 0.71500}, {0.10815, 0.90142, 0.70599}, {0.11374, 0.90673, 0.69651},
 525 |                               {0.12014, 0.91193, 0.68660}, {0.12733, 0.91701, 0.67627}, {0.13526, 0.92197, 0.66556}, {0.14391, 0.92680, 0.65448},
 526 |                               {0.15323, 0.93151, 0.64308}, {0.16319, 0.93609, 0.63137}, {0.17377, 0.94053, 0.61938}, {0.18491, 0.94484, 0.60713},
 527 |                               {0.19659, 0.94901, 0.59466}, {0.20877, 0.95304, 0.58199}, {0.22142, 0.95692, 0.56914}, {0.23449, 0.96065, 0.55614},
 528 |                               {0.24797, 0.96423, 0.54303}, {0.26180, 0.96765, 0.52981}, {0.27597, 0.97092, 0.51653}, {0.29042, 0.97403, 0.50321},
 529 |                               {0.30513, 0.97697, 0.48987}, {0.32006, 0.97974, 0.47654}, {0.33517, 0.98234, 0.46325}, {0.35043, 0.98477, 0.45002},
 530 |                               {0.36581, 0.98702, 0.43688}, {0.38127, 0.98909, 0.42386}, {0.39678, 0.99098, 0.41098}, {0.41229, 0.99268, 0.39826},
 531 |                               {0.42778, 0.99419, 0.38575}, {0.44321, 0.99551, 0.37345}, {0.45854, 0.99663, 0.36140}, {0.47375, 0.99755, 0.34963},
 532 |                               {0.48879, 0.99828, 0.33816}, {0.50362, 0.99879, 0.32701}, {0.51822, 0.99910, 0.31622}, {0.53255, 0.99919, 0.30581},
 533 |                               {0.54658, 0.99907, 0.29581}, {0.56026, 0.99873, 0.28623}, {0.57357, 0.99817, 0.27712}, {0.58646, 0.99739, 0.26849},
 534 |                               {0.59891, 0.99638, 0.26038}, {0.61088, 0.99514, 0.25280}, {0.62233, 0.99366, 0.24579}, {0.63323, 0.99195, 0.23937},
 535 |                               {0.64362, 0.98999, 0.23356}, {0.65394, 0.98775, 0.22835}, {0.66428, 0.98524, 0.22370}, {0.67462, 0.98246, 0.21960},
 536 |                               {0.68494, 0.97941, 0.21602}, {0.69525, 0.97610, 0.21294}, {0.70553, 0.97255, 0.21032}, {0.71577, 0.96875, 0.20815},
 537 |                               {0.72596, 0.96470, 0.20640}, {0.73610, 0.96043, 0.20504}, {0.74617, 0.95593, 0.20406}, {0.75617, 0.95121, 0.20343},
 538 |                               {0.76608, 0.94627, 0.20311}, {0.77591, 0.94113, 0.20310}, {0.78563, 0.93579, 0.20336}, {0.79524, 0.93025, 0.20386},
 539 |                               {0.80473, 0.92452, 0.20459}, {0.81410, 0.91861, 0.20552}, {0.82333, 0.91253, 0.20663}, {0.83241, 0.90627, 0.20788},
 540 |                               {0.84133, 0.89986, 0.20926}, {0.85010, 0.89328, 0.21074}, {0.85868, 0.88655, 0.21230}, {0.86709, 0.87968, 0.21391},
 541 |                               {0.87530, 0.87267, 0.21555}, {0.88331, 0.86553, 0.21719}, {0.89112, 0.85826, 0.21880}, {0.89870, 0.85087, 0.22038},
 542 |                               {0.90605, 0.84337, 0.22188}, {0.91317, 0.83576, 0.22328}, {0.92004, 0.82806, 0.22456}, {0.92666, 0.82025, 0.22570},
 543 |                               {0.93301, 0.81236, 0.22667}, {0.93909, 0.80439, 0.22744}, {0.94489, 0.79634, 0.22800}, {0.95039, 0.78823, 0.22831},
 544 |                               {0.95560, 0.78005, 0.22836}, {0.96049, 0.77181, 0.22811}, {0.96507, 0.76352, 0.22754}, {0.96931, 0.75519, 0.22663},
 545 |                               {0.97323, 0.74682, 0.22536}, {0.97679, 0.73842, 0.22369}, {0.98000, 0.73000, 0.22161}, {0.98289, 0.72140, 0.21918},
 546 |                               {0.98549, 0.71250, 0.21650}, {0.98781, 0.70330, 0.21358}, {0.98986, 0.69382, 0.21043}, {0.99163, 0.68408, 0.20706},
 547 |                               {0.99314, 0.67408, 0.20348}, {0.99438, 0.66386, 0.19971}, {0.99535, 0.65341, 0.19577}, {0.99607, 0.64277, 0.19165},
 548 |                               {0.99654, 0.63193, 0.18738}, {0.99675, 0.62093, 0.18297}, {0.99672, 0.60977, 0.17842}, {0.99644, 0.59846, 0.17376},
 549 |                               {0.99593, 0.58703, 0.16899}, {0.99517, 0.57549, 0.16412}, {0.99419, 0.56386, 0.15918}, {0.99297, 0.55214, 0.15417},
 550 |                               {0.99153, 0.54036, 0.14910}, {0.98987, 0.52854, 0.14398}, {0.98799, 0.51667, 0.13883}, {0.98590, 0.50479, 0.13367},
 551 |                               {0.98360, 0.49291, 0.12849}, {0.98108, 0.48104, 0.12332}, {0.97837, 0.46920, 0.11817}, {0.97545, 0.45740, 0.11305},
 552 |                               {0.97234, 0.44565, 0.10797}, {0.96904, 0.43399, 0.10294}, {0.96555, 0.42241, 0.09798}, {0.96187, 0.41093, 0.09310},
 553 |                               {0.95801, 0.39958, 0.08831}, {0.95398, 0.38836, 0.08362}, {0.94977, 0.37729, 0.07905}, {0.94538, 0.36638, 0.07461},
 554 |                               {0.94084, 0.35566, 0.07031}, {0.93612, 0.34513, 0.06616}, {0.93125, 0.33482, 0.06218}, {0.92623, 0.32473, 0.05837},
 555 |                               {0.92105, 0.31489, 0.05475}, {0.91572, 0.30530, 0.05134}, {0.91024, 0.29599, 0.04814}, {0.90463, 0.28696, 0.04516},
 556 |                               {0.89888, 0.27824, 0.04243}, {0.89298, 0.26981, 0.03993}, {0.88691, 0.26152, 0.03753}, {0.88066, 0.25334, 0.03521},
 557 |                               {0.87422, 0.24526, 0.03297}, {0.86760, 0.23730, 0.03082}, {0.86079, 0.22945, 0.02875}, {0.85380, 0.22170, 0.02677},
 558 |                               {0.84662, 0.21407, 0.02487}, {0.83926, 0.20654, 0.02305}, {0.83172, 0.19912, 0.02131}, {0.82399, 0.19182, 0.01966},
 559 |                               {0.81608, 0.18462, 0.01809}, {0.80799, 0.17753, 0.01660}, {0.79971, 0.17055, 0.01520}, {0.79125, 0.16368, 0.01387},
 560 |                               {0.78260, 0.15693, 0.01264}, {0.77377, 0.15028, 0.01148}, {0.76476, 0.14374, 0.01041}, {0.75556, 0.13731, 0.00942},
 561 |                               {0.74617, 0.13098, 0.00851}, {0.73661, 0.12477, 0.00769}, {0.72686, 0.11867, 0.00695}, {0.71692, 0.11268, 0.00629},
 562 |                               {0.70680, 0.10680, 0.00571}, {0.69650, 0.10102, 0.00522}, {0.68602, 0.09536, 0.00481}, {0.67535, 0.08980, 0.00449},
 563 |                               {0.66449, 0.08436, 0.00424}, {0.65345, 0.07902, 0.00408}, {0.64223, 0.07380, 0.00401}, {0.63082, 0.06868, 0.00401},
 564 |                               {0.61923, 0.06367, 0.00410}, {0.60746, 0.05878, 0.00427}, {0.59550, 0.05399, 0.00453}, {0.58336, 0.04931, 0.00486},
 565 |                               {0.57103, 0.04474, 0.00529}, {0.55852, 0.04028, 0.00579}, {0.54583, 0.03593, 0.00638}, {0.53295, 0.03169, 0.00705},
 566 |                               {0.51989, 0.02756, 0.00780}, {0.50664, 0.02354, 0.00863}, {0.49321, 0.01963, 0.00955}, {0.47960, 0.01583, 0.01055}};
 567 | 
 568 |     return internal::CalcLerp(x, data);
 569 | }
 570 | 
 571 | inline Color GetHotColor(double x) {
 572 |     x = internal::Clamp01(x);
 573 | 
 574 |     constexpr Color r{1.0, 0.0, 0.0};
 575 |     constexpr Color g{0.0, 1.0, 0.0};
 576 |     constexpr Color b{0.0, 0.0, 1.0};
 577 | 
 578 |     if (x < 0.4) {
 579 |         const double t = x / 0.4;
 580 |         return t * r;
 581 |     } else if (x < 0.8) {
 582 |         const double t = (x - 0.4) / (0.8 - 0.4);
 583 |         return r + t * g;
 584 |     } else {
 585 |         const double t = (x - 0.8) / (1.0 - 0.8);
 586 |         return r + g + t * b;
 587 |     }
 588 | }
 589 | 
 590 | inline constexpr Color GetGrayColor(double x) noexcept {
 591 |     return Color{1.0 - internal::Clamp01(x)};
 592 | }
 593 | 
 594 | inline Color GetMagmaColor(double x) {
 595 |     constexpr Color data[] = {
 596 |         {0.001462, 0.000466, 0.013866}, {0.002258, 0.001295, 0.018331}, {0.003279, 0.002305, 0.023708}, {0.004512, 0.003490, 0.029965},
 597 |         {0.005950, 0.004843, 0.037130}, {0.007588, 0.006356, 0.044973}, {0.009426, 0.008022, 0.052844}, {0.011465, 0.009828, 0.060750},
 598 |         {0.013708, 0.011771, 0.068667}, {0.016156, 0.013840, 0.076603}, {0.018815, 0.016026, 0.084584}, {0.021692, 0.018320, 0.092610},
 599 |         {0.024792, 0.020715, 0.100676}, {0.028123, 0.023201, 0.108787}, {0.031696, 0.025765, 0.116965}, {0.035520, 0.028397, 0.125209},
 600 |         {0.039608, 0.031090, 0.133515}, {0.043830, 0.033830, 0.141886}, {0.048062, 0.036607, 0.150327}, {0.052320, 0.039407, 0.158841},
 601 |         {0.056615, 0.042160, 0.167446}, {0.060949, 0.044794, 0.176129}, {0.065330, 0.047318, 0.184892}, {0.069764, 0.049726, 0.193735},
 602 |         {0.074257, 0.052017, 0.202660}, {0.078815, 0.054184, 0.211667}, {0.083446, 0.056225, 0.220755}, {0.088155, 0.058133, 0.229922},
 603 |         {0.092949, 0.059904, 0.239164}, {0.097833, 0.061531, 0.248477}, {0.102815, 0.063010, 0.257854}, {0.107899, 0.064335, 0.267289},
 604 |         {0.113094, 0.065492, 0.276784}, {0.118405, 0.066479, 0.286321}, {0.123833, 0.067295, 0.295879}, {0.129380, 0.067935, 0.305443},
 605 |         {0.135053, 0.068391, 0.315000}, {0.140858, 0.068654, 0.324538}, {0.146785, 0.068738, 0.334011}, {0.152839, 0.068637, 0.343404},
 606 |         {0.159018, 0.068354, 0.352688}, {0.165308, 0.067911, 0.361816}, {0.171713, 0.067305, 0.370771}, {0.178212, 0.066576, 0.379497},
 607 |         {0.184801, 0.065732, 0.387973}, {0.191460, 0.064818, 0.396152}, {0.198177, 0.063862, 0.404009}, {0.204935, 0.062907, 0.411514},
 608 |         {0.211718, 0.061992, 0.418647}, {0.218512, 0.061158, 0.425392}, {0.225302, 0.060445, 0.431742}, {0.232077, 0.059889, 0.437695},
 609 |         {0.238826, 0.059517, 0.443256}, {0.245543, 0.059352, 0.448436}, {0.252220, 0.059415, 0.453248}, {0.258857, 0.059706, 0.457710},
 610 |         {0.265447, 0.060237, 0.461840}, {0.271994, 0.060994, 0.465660}, {0.278493, 0.061978, 0.469190}, {0.284951, 0.063168, 0.472451},
 611 |         {0.291366, 0.064553, 0.475462}, {0.297740, 0.066117, 0.478243}, {0.304081, 0.067835, 0.480812}, {0.310382, 0.069702, 0.483186},
 612 |         {0.316654, 0.071690, 0.485380}, {0.322899, 0.073782, 0.487408}, {0.329114, 0.075972, 0.489287}, {0.335308, 0.078236, 0.491024},
 613 |         {0.341482, 0.080564, 0.492631}, {0.347636, 0.082946, 0.494121}, {0.353773, 0.085373, 0.495501}, {0.359898, 0.087831, 0.496778},
 614 |         {0.366012, 0.090314, 0.497960}, {0.372116, 0.092816, 0.499053}, {0.378211, 0.095332, 0.500067}, {0.384299, 0.097855, 0.501002},
 615 |         {0.390384, 0.100379, 0.501864}, {0.396467, 0.102902, 0.502658}, {0.402548, 0.105420, 0.503386}, {0.408629, 0.107930, 0.504052},
 616 |         {0.414709, 0.110431, 0.504662}, {0.420791, 0.112920, 0.505215}, {0.426877, 0.115395, 0.505714}, {0.432967, 0.117855, 0.506160},
 617 |         {0.439062, 0.120298, 0.506555}, {0.445163, 0.122724, 0.506901}, {0.451271, 0.125132, 0.507198}, {0.457386, 0.127522, 0.507448},
 618 |         {0.463508, 0.129893, 0.507652}, {0.469640, 0.132245, 0.507809}, {0.475780, 0.134577, 0.507921}, {0.481929, 0.136891, 0.507989},
 619 |         {0.488088, 0.139186, 0.508011}, {0.494258, 0.141462, 0.507988}, {0.500438, 0.143719, 0.507920}, {0.506629, 0.145958, 0.507806},
 620 |         {0.512831, 0.148179, 0.507648}, {0.519045, 0.150383, 0.507443}, {0.525270, 0.152569, 0.507192}, {0.531507, 0.154739, 0.506895},
 621 |         {0.537755, 0.156894, 0.506551}, {0.544015, 0.159033, 0.506159}, {0.550287, 0.161158, 0.505719}, {0.556571, 0.163269, 0.505230},
 622 |         {0.562866, 0.165368, 0.504692}, {0.569172, 0.167454, 0.504105}, {0.575490, 0.169530, 0.503466}, {0.581819, 0.171596, 0.502777},
 623 |         {0.588158, 0.173652, 0.502035}, {0.594508, 0.175701, 0.501241}, {0.600868, 0.177743, 0.500394}, {0.607238, 0.179779, 0.499492},
 624 |         {0.613617, 0.181811, 0.498536}, {0.620005, 0.183840, 0.497524}, {0.626401, 0.185867, 0.496456}, {0.632805, 0.187893, 0.495332},
 625 |         {0.639216, 0.189921, 0.494150}, {0.645633, 0.191952, 0.492910}, {0.652056, 0.193986, 0.491611}, {0.658483, 0.196027, 0.490253},
 626 |         {0.664915, 0.198075, 0.488836}, {0.671349, 0.200133, 0.487358}, {0.677786, 0.202203, 0.485819}, {0.684224, 0.204286, 0.484219},
 627 |         {0.690661, 0.206384, 0.482558}, {0.697098, 0.208501, 0.480835}, {0.703532, 0.210638, 0.479049}, {0.709962, 0.212797, 0.477201},
 628 |         {0.716387, 0.214982, 0.475290}, {0.722805, 0.217194, 0.473316}, {0.729216, 0.219437, 0.471279}, {0.735616, 0.221713, 0.469180},
 629 |         {0.742004, 0.224025, 0.467018}, {0.748378, 0.226377, 0.464794}, {0.754737, 0.228772, 0.462509}, {0.761077, 0.231214, 0.460162},
 630 |         {0.767398, 0.233705, 0.457755}, {0.773695, 0.236249, 0.455289}, {0.779968, 0.238851, 0.452765}, {0.786212, 0.241514, 0.450184},
 631 |         {0.792427, 0.244242, 0.447543}, {0.798608, 0.247040, 0.444848}, {0.804752, 0.249911, 0.442102}, {0.810855, 0.252861, 0.439305},
 632 |         {0.816914, 0.255895, 0.436461}, {0.822926, 0.259016, 0.433573}, {0.828886, 0.262229, 0.430644}, {0.834791, 0.265540, 0.427671},
 633 |         {0.840636, 0.268953, 0.424666}, {0.846416, 0.272473, 0.421631}, {0.852126, 0.276106, 0.418573}, {0.857763, 0.279857, 0.415496},
 634 |         {0.863320, 0.283729, 0.412403}, {0.868793, 0.287728, 0.409303}, {0.874176, 0.291859, 0.406205}, {0.879464, 0.296125, 0.403118},
 635 |         {0.884651, 0.300530, 0.400047}, {0.889731, 0.305079, 0.397002}, {0.894700, 0.309773, 0.393995}, {0.899552, 0.314616, 0.391037},
 636 |         {0.904281, 0.319610, 0.388137}, {0.908884, 0.324755, 0.385308}, {0.913354, 0.330052, 0.382563}, {0.917689, 0.335500, 0.379915},
 637 |         {0.921884, 0.341098, 0.377376}, {0.925937, 0.346844, 0.374959}, {0.929845, 0.352734, 0.372677}, {0.933606, 0.358764, 0.370541},
 638 |         {0.937221, 0.364929, 0.368567}, {0.940687, 0.371224, 0.366762}, {0.944006, 0.377643, 0.365136}, {0.947180, 0.384178, 0.363701},
 639 |         {0.950210, 0.390820, 0.362468}, {0.953099, 0.397563, 0.361438}, {0.955849, 0.404400, 0.360619}, {0.958464, 0.411324, 0.360014},
 640 |         {0.960949, 0.418323, 0.359630}, {0.963310, 0.425390, 0.359469}, {0.965549, 0.432519, 0.359529}, {0.967671, 0.439703, 0.359810},
 641 |         {0.969680, 0.446936, 0.360311}, {0.971582, 0.454210, 0.361030}, {0.973381, 0.461520, 0.361965}, {0.975082, 0.468861, 0.363111},
 642 |         {0.976690, 0.476226, 0.364466}, {0.978210, 0.483612, 0.366025}, {0.979645, 0.491014, 0.367783}, {0.981000, 0.498428, 0.369734},
 643 |         {0.982279, 0.505851, 0.371874}, {0.983485, 0.513280, 0.374198}, {0.984622, 0.520713, 0.376698}, {0.985693, 0.528148, 0.379371},
 644 |         {0.986700, 0.535582, 0.382210}, {0.987646, 0.543015, 0.385210}, {0.988533, 0.550446, 0.388365}, {0.989363, 0.557873, 0.391671},
 645 |         {0.990138, 0.565296, 0.395122}, {0.990871, 0.572706, 0.398714}, {0.991558, 0.580107, 0.402441}, {0.992196, 0.587502, 0.406299},
 646 |         {0.992785, 0.594891, 0.410283}, {0.993326, 0.602275, 0.414390}, {0.993834, 0.609644, 0.418613}, {0.994309, 0.616999, 0.422950},
 647 |         {0.994738, 0.624350, 0.427397}, {0.995122, 0.631696, 0.431951}, {0.995480, 0.639027, 0.436607}, {0.995810, 0.646344, 0.441361},
 648 |         {0.996096, 0.653659, 0.446213}, {0.996341, 0.660969, 0.451160}, {0.996580, 0.668256, 0.456192}, {0.996775, 0.675541, 0.461314},
 649 |         {0.996925, 0.682828, 0.466526}, {0.997077, 0.690088, 0.471811}, {0.997186, 0.697349, 0.477182}, {0.997254, 0.704611, 0.482635},
 650 |         {0.997325, 0.711848, 0.488154}, {0.997351, 0.719089, 0.493755}, {0.997351, 0.726324, 0.499428}, {0.997341, 0.733545, 0.505167},
 651 |         {0.997285, 0.740772, 0.510983}, {0.997228, 0.747981, 0.516859}, {0.997138, 0.755190, 0.522806}, {0.997019, 0.762398, 0.528821},
 652 |         {0.996898, 0.769591, 0.534892}, {0.996727, 0.776795, 0.541039}, {0.996571, 0.783977, 0.547233}, {0.996369, 0.791167, 0.553499},
 653 |         {0.996162, 0.798348, 0.559820}, {0.995932, 0.805527, 0.566202}, {0.995680, 0.812706, 0.572645}, {0.995424, 0.819875, 0.579140},
 654 |         {0.995131, 0.827052, 0.585701}, {0.994851, 0.834213, 0.592307}, {0.994524, 0.841387, 0.598983}, {0.994222, 0.848540, 0.605696},
 655 |         {0.993866, 0.855711, 0.612482}, {0.993545, 0.862859, 0.619299}, {0.993170, 0.870024, 0.626189}, {0.992831, 0.877168, 0.633109},
 656 |         {0.992440, 0.884330, 0.640099}, {0.992089, 0.891470, 0.647116}, {0.991688, 0.898627, 0.654202}, {0.991332, 0.905763, 0.661309},
 657 |         {0.990930, 0.912915, 0.668481}, {0.990570, 0.920049, 0.675675}, {0.990175, 0.927196, 0.682926}, {0.989815, 0.934329, 0.690198},
 658 |         {0.989434, 0.941470, 0.697519}, {0.989077, 0.948604, 0.704863}, {0.988717, 0.955742, 0.712242}, {0.988367, 0.962878, 0.719649},
 659 |         {0.988033, 0.970012, 0.727077}, {0.987691, 0.977154, 0.734536}, {0.987387, 0.984288, 0.742002}, {0.987053, 0.991438, 0.749504}};
 660 | 
 661 |     return internal::CalcLerp(x, data);
 662 | }
 663 | 
 664 | inline Color GetInfernoColor(double x) {
 665 |     constexpr Color data[] = {
 666 |         {0.001462, 0.000466, 0.013866}, {0.002267, 0.001270, 0.018570}, {0.003299, 0.002249, 0.024239}, {0.004547, 0.003392, 0.030909},
 667 |         {0.006006, 0.004692, 0.038558}, {0.007676, 0.006136, 0.046836}, {0.009561, 0.007713, 0.055143}, {0.011663, 0.009417, 0.063460},
 668 |         {0.013995, 0.011225, 0.071862}, {0.016561, 0.013136, 0.080282}, {0.019373, 0.015133, 0.088767}, {0.022447, 0.017199, 0.097327},
 669 |         {0.025793, 0.019331, 0.105930}, {0.029432, 0.021503, 0.114621}, {0.033385, 0.023702, 0.123397}, {0.037668, 0.025921, 0.132232},
 670 |         {0.042253, 0.028139, 0.141141}, {0.046915, 0.030324, 0.150164}, {0.051644, 0.032474, 0.159254}, {0.056449, 0.034569, 0.168414},
 671 |         {0.061340, 0.036590, 0.177642}, {0.066331, 0.038504, 0.186962}, {0.071429, 0.040294, 0.196354}, {0.076637, 0.041905, 0.205799},
 672 |         {0.081962, 0.043328, 0.215289}, {0.087411, 0.044556, 0.224813}, {0.092990, 0.045583, 0.234358}, {0.098702, 0.046402, 0.243904},
 673 |         {0.104551, 0.047008, 0.253430}, {0.110536, 0.047399, 0.262912}, {0.116656, 0.047574, 0.272321}, {0.122908, 0.047536, 0.281624},
 674 |         {0.129285, 0.047293, 0.290788}, {0.135778, 0.046856, 0.299776}, {0.142378, 0.046242, 0.308553}, {0.149073, 0.045468, 0.317085},
 675 |         {0.155850, 0.044559, 0.325338}, {0.162689, 0.043554, 0.333277}, {0.169575, 0.042489, 0.340874}, {0.176493, 0.041402, 0.348111},
 676 |         {0.183429, 0.040329, 0.354971}, {0.190367, 0.039309, 0.361447}, {0.197297, 0.038400, 0.367535}, {0.204209, 0.037632, 0.373238},
 677 |         {0.211095, 0.037030, 0.378563}, {0.217949, 0.036615, 0.383522}, {0.224763, 0.036405, 0.388129}, {0.231538, 0.036405, 0.392400},
 678 |         {0.238273, 0.036621, 0.396353}, {0.244967, 0.037055, 0.400007}, {0.251620, 0.037705, 0.403378}, {0.258234, 0.038571, 0.406485},
 679 |         {0.264810, 0.039647, 0.409345}, {0.271347, 0.040922, 0.411976}, {0.277850, 0.042353, 0.414392}, {0.284321, 0.043933, 0.416608},
 680 |         {0.290763, 0.045644, 0.418637}, {0.297178, 0.047470, 0.420491}, {0.303568, 0.049396, 0.422182}, {0.309935, 0.051407, 0.423721},
 681 |         {0.316282, 0.053490, 0.425116}, {0.322610, 0.055634, 0.426377}, {0.328921, 0.057827, 0.427511}, {0.335217, 0.060060, 0.428524},
 682 |         {0.341500, 0.062325, 0.429425}, {0.347771, 0.064616, 0.430217}, {0.354032, 0.066925, 0.430906}, {0.360284, 0.069247, 0.431497},
 683 |         {0.366529, 0.071579, 0.431994}, {0.372768, 0.073915, 0.432400}, {0.379001, 0.076253, 0.432719}, {0.385228, 0.078591, 0.432955},
 684 |         {0.391453, 0.080927, 0.433109}, {0.397674, 0.083257, 0.433183}, {0.403894, 0.085580, 0.433179}, {0.410113, 0.087896, 0.433098},
 685 |         {0.416331, 0.090203, 0.432943}, {0.422549, 0.092501, 0.432714}, {0.428768, 0.094790, 0.432412}, {0.434987, 0.097069, 0.432039},
 686 |         {0.441207, 0.099338, 0.431594}, {0.447428, 0.101597, 0.431080}, {0.453651, 0.103848, 0.430498}, {0.459875, 0.106089, 0.429846},
 687 |         {0.466100, 0.108322, 0.429125}, {0.472328, 0.110547, 0.428334}, {0.478558, 0.112764, 0.427475}, {0.484789, 0.114974, 0.426548},
 688 |         {0.491022, 0.117179, 0.425552}, {0.497257, 0.119379, 0.424488}, {0.503493, 0.121575, 0.423356}, {0.509730, 0.123769, 0.422156},
 689 |         {0.515967, 0.125960, 0.420887}, {0.522206, 0.128150, 0.419549}, {0.528444, 0.130341, 0.418142}, {0.534683, 0.132534, 0.416667},
 690 |         {0.540920, 0.134729, 0.415123}, {0.547157, 0.136929, 0.413511}, {0.553392, 0.139134, 0.411829}, {0.559624, 0.141346, 0.410078},
 691 |         {0.565854, 0.143567, 0.408258}, {0.572081, 0.145797, 0.406369}, {0.578304, 0.148039, 0.404411}, {0.584521, 0.150294, 0.402385},
 692 |         {0.590734, 0.152563, 0.400290}, {0.596940, 0.154848, 0.398125}, {0.603139, 0.157151, 0.395891}, {0.609330, 0.159474, 0.393589},
 693 |         {0.615513, 0.161817, 0.391219}, {0.621685, 0.164184, 0.388781}, {0.627847, 0.166575, 0.386276}, {0.633998, 0.168992, 0.383704},
 694 |         {0.640135, 0.171438, 0.381065}, {0.646260, 0.173914, 0.378359}, {0.652369, 0.176421, 0.375586}, {0.658463, 0.178962, 0.372748},
 695 |         {0.664540, 0.181539, 0.369846}, {0.670599, 0.184153, 0.366879}, {0.676638, 0.186807, 0.363849}, {0.682656, 0.189501, 0.360757},
 696 |         {0.688653, 0.192239, 0.357603}, {0.694627, 0.195021, 0.354388}, {0.700576, 0.197851, 0.351113}, {0.706500, 0.200728, 0.347777},
 697 |         {0.712396, 0.203656, 0.344383}, {0.718264, 0.206636, 0.340931}, {0.724103, 0.209670, 0.337424}, {0.729909, 0.212759, 0.333861},
 698 |         {0.735683, 0.215906, 0.330245}, {0.741423, 0.219112, 0.326576}, {0.747127, 0.222378, 0.322856}, {0.752794, 0.225706, 0.319085},
 699 |         {0.758422, 0.229097, 0.315266}, {0.764010, 0.232554, 0.311399}, {0.769556, 0.236077, 0.307485}, {0.775059, 0.239667, 0.303526},
 700 |         {0.780517, 0.243327, 0.299523}, {0.785929, 0.247056, 0.295477}, {0.791293, 0.250856, 0.291390}, {0.796607, 0.254728, 0.287264},
 701 |         {0.801871, 0.258674, 0.283099}, {0.807082, 0.262692, 0.278898}, {0.812239, 0.266786, 0.274661}, {0.817341, 0.270954, 0.270390},
 702 |         {0.822386, 0.275197, 0.266085}, {0.827372, 0.279517, 0.261750}, {0.832299, 0.283913, 0.257383}, {0.837165, 0.288385, 0.252988},
 703 |         {0.841969, 0.292933, 0.248564}, {0.846709, 0.297559, 0.244113}, {0.851384, 0.302260, 0.239636}, {0.855992, 0.307038, 0.235133},
 704 |         {0.860533, 0.311892, 0.230606}, {0.865006, 0.316822, 0.226055}, {0.869409, 0.321827, 0.221482}, {0.873741, 0.326906, 0.216886},
 705 |         {0.878001, 0.332060, 0.212268}, {0.882188, 0.337287, 0.207628}, {0.886302, 0.342586, 0.202968}, {0.890341, 0.347957, 0.198286},
 706 |         {0.894305, 0.353399, 0.193584}, {0.898192, 0.358911, 0.188860}, {0.902003, 0.364492, 0.184116}, {0.905735, 0.370140, 0.179350},
 707 |         {0.909390, 0.375856, 0.174563}, {0.912966, 0.381636, 0.169755}, {0.916462, 0.387481, 0.164924}, {0.919879, 0.393389, 0.160070},
 708 |         {0.923215, 0.399359, 0.155193}, {0.926470, 0.405389, 0.150292}, {0.929644, 0.411479, 0.145367}, {0.932737, 0.417627, 0.140417},
 709 |         {0.935747, 0.423831, 0.135440}, {0.938675, 0.430091, 0.130438}, {0.941521, 0.436405, 0.125409}, {0.944285, 0.442772, 0.120354},
 710 |         {0.946965, 0.449191, 0.115272}, {0.949562, 0.455660, 0.110164}, {0.952075, 0.462178, 0.105031}, {0.954506, 0.468744, 0.099874},
 711 |         {0.956852, 0.475356, 0.094695}, {0.959114, 0.482014, 0.089499}, {0.961293, 0.488716, 0.084289}, {0.963387, 0.495462, 0.079073},
 712 |         {0.965397, 0.502249, 0.073859}, {0.967322, 0.509078, 0.068659}, {0.969163, 0.515946, 0.063488}, {0.970919, 0.522853, 0.058367},
 713 |         {0.972590, 0.529798, 0.053324}, {0.974176, 0.536780, 0.048392}, {0.975677, 0.543798, 0.043618}, {0.977092, 0.550850, 0.039050},
 714 |         {0.978422, 0.557937, 0.034931}, {0.979666, 0.565057, 0.031409}, {0.980824, 0.572209, 0.028508}, {0.981895, 0.579392, 0.026250},
 715 |         {0.982881, 0.586606, 0.024661}, {0.983779, 0.593849, 0.023770}, {0.984591, 0.601122, 0.023606}, {0.985315, 0.608422, 0.024202},
 716 |         {0.985952, 0.615750, 0.025592}, {0.986502, 0.623105, 0.027814}, {0.986964, 0.630485, 0.030908}, {0.987337, 0.637890, 0.034916},
 717 |         {0.987622, 0.645320, 0.039886}, {0.987819, 0.652773, 0.045581}, {0.987926, 0.660250, 0.051750}, {0.987945, 0.667748, 0.058329},
 718 |         {0.987874, 0.675267, 0.065257}, {0.987714, 0.682807, 0.072489}, {0.987464, 0.690366, 0.079990}, {0.987124, 0.697944, 0.087731},
 719 |         {0.986694, 0.705540, 0.095694}, {0.986175, 0.713153, 0.103863}, {0.985566, 0.720782, 0.112229}, {0.984865, 0.728427, 0.120785},
 720 |         {0.984075, 0.736087, 0.129527}, {0.983196, 0.743758, 0.138453}, {0.982228, 0.751442, 0.147565}, {0.981173, 0.759135, 0.156863},
 721 |         {0.980032, 0.766837, 0.166353}, {0.978806, 0.774545, 0.176037}, {0.977497, 0.782258, 0.185923}, {0.976108, 0.789974, 0.196018},
 722 |         {0.974638, 0.797692, 0.206332}, {0.973088, 0.805409, 0.216877}, {0.971468, 0.813122, 0.227658}, {0.969783, 0.820825, 0.238686},
 723 |         {0.968041, 0.828515, 0.249972}, {0.966243, 0.836191, 0.261534}, {0.964394, 0.843848, 0.273391}, {0.962517, 0.851476, 0.285546},
 724 |         {0.960626, 0.859069, 0.298010}, {0.958720, 0.866624, 0.310820}, {0.956834, 0.874129, 0.323974}, {0.954997, 0.881569, 0.337475},
 725 |         {0.953215, 0.888942, 0.351369}, {0.951546, 0.896226, 0.365627}, {0.950018, 0.903409, 0.380271}, {0.948683, 0.910473, 0.395289},
 726 |         {0.947594, 0.917399, 0.410665}, {0.946809, 0.924168, 0.426373}, {0.946392, 0.930761, 0.442367}, {0.946403, 0.937159, 0.458592},
 727 |         {0.946903, 0.943348, 0.474970}, {0.947937, 0.949318, 0.491426}, {0.949545, 0.955063, 0.507860}, {0.951740, 0.960587, 0.524203},
 728 |         {0.954529, 0.965896, 0.540361}, {0.957896, 0.971003, 0.556275}, {0.961812, 0.975924, 0.571925}, {0.966249, 0.980678, 0.587206},
 729 |         {0.971162, 0.985282, 0.602154}, {0.976511, 0.989753, 0.616760}, {0.982257, 0.994109, 0.631017}, {0.988362, 0.998364, 0.644924}};
 730 | 
 731 |     return internal::CalcLerp(x, data);
 732 | }
 733 | 
 734 | inline Color GetPlasmaColor(double x) {
 735 |     constexpr Color data[] = {
 736 |         {0.050383, 0.029803, 0.527975}, {0.063536, 0.028426, 0.533124}, {0.075353, 0.027206, 0.538007}, {0.086222, 0.026125, 0.542658},
 737 |         {0.096379, 0.025165, 0.547103}, {0.105980, 0.024309, 0.551368}, {0.115124, 0.023556, 0.555468}, {0.123903, 0.022878, 0.559423},
 738 |         {0.132381, 0.022258, 0.563250}, {0.140603, 0.021687, 0.566959}, {0.148607, 0.021154, 0.570562}, {0.156421, 0.020651, 0.574065},
 739 |         {0.164070, 0.020171, 0.577478}, {0.171574, 0.019706, 0.580806}, {0.178950, 0.019252, 0.584054}, {0.186213, 0.018803, 0.587228},
 740 |         {0.193374, 0.018354, 0.590330}, {0.200445, 0.017902, 0.593364}, {0.207435, 0.017442, 0.596333}, {0.214350, 0.016973, 0.599239},
 741 |         {0.221197, 0.016497, 0.602083}, {0.227983, 0.016007, 0.604867}, {0.234715, 0.015502, 0.607592}, {0.241396, 0.014979, 0.610259},
 742 |         {0.248032, 0.014439, 0.612868}, {0.254627, 0.013882, 0.615419}, {0.261183, 0.013308, 0.617911}, {0.267703, 0.012716, 0.620346},
 743 |         {0.274191, 0.012109, 0.622722}, {0.280648, 0.011488, 0.625038}, {0.287076, 0.010855, 0.627295}, {0.293478, 0.010213, 0.629490},
 744 |         {0.299855, 0.009561, 0.631624}, {0.306210, 0.008902, 0.633694}, {0.312543, 0.008239, 0.635700}, {0.318856, 0.007576, 0.637640},
 745 |         {0.325150, 0.006915, 0.639512}, {0.331426, 0.006261, 0.641316}, {0.337683, 0.005618, 0.643049}, {0.343925, 0.004991, 0.644710},
 746 |         {0.350150, 0.004382, 0.646298}, {0.356359, 0.003798, 0.647810}, {0.362553, 0.003243, 0.649245}, {0.368733, 0.002724, 0.650601},
 747 |         {0.374897, 0.002245, 0.651876}, {0.381047, 0.001814, 0.653068}, {0.387183, 0.001434, 0.654177}, {0.393304, 0.001114, 0.655199},
 748 |         {0.399411, 0.000859, 0.656133}, {0.405503, 0.000678, 0.656977}, {0.411580, 0.000577, 0.657730}, {0.417642, 0.000564, 0.658390},
 749 |         {0.423689, 0.000646, 0.658956}, {0.429719, 0.000831, 0.659425}, {0.435734, 0.001127, 0.659797}, {0.441732, 0.001540, 0.660069},
 750 |         {0.447714, 0.002080, 0.660240}, {0.453677, 0.002755, 0.660310}, {0.459623, 0.003574, 0.660277}, {0.465550, 0.004545, 0.660139},
 751 |         {0.471457, 0.005678, 0.659897}, {0.477344, 0.006980, 0.659549}, {0.483210, 0.008460, 0.659095}, {0.489055, 0.010127, 0.658534},
 752 |         {0.494877, 0.011990, 0.657865}, {0.500678, 0.014055, 0.657088}, {0.506454, 0.016333, 0.656202}, {0.512206, 0.018833, 0.655209},
 753 |         {0.517933, 0.021563, 0.654109}, {0.523633, 0.024532, 0.652901}, {0.529306, 0.027747, 0.651586}, {0.534952, 0.031217, 0.650165},
 754 |         {0.540570, 0.034950, 0.648640}, {0.546157, 0.038954, 0.647010}, {0.551715, 0.043136, 0.645277}, {0.557243, 0.047331, 0.643443},
 755 |         {0.562738, 0.051545, 0.641509}, {0.568201, 0.055778, 0.639477}, {0.573632, 0.060028, 0.637349}, {0.579029, 0.064296, 0.635126},
 756 |         {0.584391, 0.068579, 0.632812}, {0.589719, 0.072878, 0.630408}, {0.595011, 0.077190, 0.627917}, {0.600266, 0.081516, 0.625342},
 757 |         {0.605485, 0.085854, 0.622686}, {0.610667, 0.090204, 0.619951}, {0.615812, 0.094564, 0.617140}, {0.620919, 0.098934, 0.614257},
 758 |         {0.625987, 0.103312, 0.611305}, {0.631017, 0.107699, 0.608287}, {0.636008, 0.112092, 0.605205}, {0.640959, 0.116492, 0.602065},
 759 |         {0.645872, 0.120898, 0.598867}, {0.650746, 0.125309, 0.595617}, {0.655580, 0.129725, 0.592317}, {0.660374, 0.134144, 0.588971},
 760 |         {0.665129, 0.138566, 0.585582}, {0.669845, 0.142992, 0.582154}, {0.674522, 0.147419, 0.578688}, {0.679160, 0.151848, 0.575189},
 761 |         {0.683758, 0.156278, 0.571660}, {0.688318, 0.160709, 0.568103}, {0.692840, 0.165141, 0.564522}, {0.697324, 0.169573, 0.560919},
 762 |         {0.701769, 0.174005, 0.557296}, {0.706178, 0.178437, 0.553657}, {0.710549, 0.182868, 0.550004}, {0.714883, 0.187299, 0.546338},
 763 |         {0.719181, 0.191729, 0.542663}, {0.723444, 0.196158, 0.538981}, {0.727670, 0.200586, 0.535293}, {0.731862, 0.205013, 0.531601},
 764 |         {0.736019, 0.209439, 0.527908}, {0.740143, 0.213864, 0.524216}, {0.744232, 0.218288, 0.520524}, {0.748289, 0.222711, 0.516834},
 765 |         {0.752312, 0.227133, 0.513149}, {0.756304, 0.231555, 0.509468}, {0.760264, 0.235976, 0.505794}, {0.764193, 0.240396, 0.502126},
 766 |         {0.768090, 0.244817, 0.498465}, {0.771958, 0.249237, 0.494813}, {0.775796, 0.253658, 0.491171}, {0.779604, 0.258078, 0.487539},
 767 |         {0.783383, 0.262500, 0.483918}, {0.787133, 0.266922, 0.480307}, {0.790855, 0.271345, 0.476706}, {0.794549, 0.275770, 0.473117},
 768 |         {0.798216, 0.280197, 0.469538}, {0.801855, 0.284626, 0.465971}, {0.805467, 0.289057, 0.462415}, {0.809052, 0.293491, 0.458870},
 769 |         {0.812612, 0.297928, 0.455338}, {0.816144, 0.302368, 0.451816}, {0.819651, 0.306812, 0.448306}, {0.823132, 0.311261, 0.444806},
 770 |         {0.826588, 0.315714, 0.441316}, {0.830018, 0.320172, 0.437836}, {0.833422, 0.324635, 0.434366}, {0.836801, 0.329105, 0.430905},
 771 |         {0.840155, 0.333580, 0.427455}, {0.843484, 0.338062, 0.424013}, {0.846788, 0.342551, 0.420579}, {0.850066, 0.347048, 0.417153},
 772 |         {0.853319, 0.351553, 0.413734}, {0.856547, 0.356066, 0.410322}, {0.859750, 0.360588, 0.406917}, {0.862927, 0.365119, 0.403519},
 773 |         {0.866078, 0.369660, 0.400126}, {0.869203, 0.374212, 0.396738}, {0.872303, 0.378774, 0.393355}, {0.875376, 0.383347, 0.389976},
 774 |         {0.878423, 0.387932, 0.386600}, {0.881443, 0.392529, 0.383229}, {0.884436, 0.397139, 0.379860}, {0.887402, 0.401762, 0.376494},
 775 |         {0.890340, 0.406398, 0.373130}, {0.893250, 0.411048, 0.369768}, {0.896131, 0.415712, 0.366407}, {0.898984, 0.420392, 0.363047},
 776 |         {0.901807, 0.425087, 0.359688}, {0.904601, 0.429797, 0.356329}, {0.907365, 0.434524, 0.352970}, {0.910098, 0.439268, 0.349610},
 777 |         {0.912800, 0.444029, 0.346251}, {0.915471, 0.448807, 0.342890}, {0.918109, 0.453603, 0.339529}, {0.920714, 0.458417, 0.336166},
 778 |         {0.923287, 0.463251, 0.332801}, {0.925825, 0.468103, 0.329435}, {0.928329, 0.472975, 0.326067}, {0.930798, 0.477867, 0.322697},
 779 |         {0.933232, 0.482780, 0.319325}, {0.935630, 0.487712, 0.315952}, {0.937990, 0.492667, 0.312575}, {0.940313, 0.497642, 0.309197},
 780 |         {0.942598, 0.502639, 0.305816}, {0.944844, 0.507658, 0.302433}, {0.947051, 0.512699, 0.299049}, {0.949217, 0.517763, 0.295662},
 781 |         {0.951344, 0.522850, 0.292275}, {0.953428, 0.527960, 0.288883}, {0.955470, 0.533093, 0.285490}, {0.957469, 0.538250, 0.282096},
 782 |         {0.959424, 0.543431, 0.278701}, {0.961336, 0.548636, 0.275305}, {0.963203, 0.553865, 0.271909}, {0.965024, 0.559118, 0.268513},
 783 |         {0.966798, 0.564396, 0.265118}, {0.968526, 0.569700, 0.261721}, {0.970205, 0.575028, 0.258325}, {0.971835, 0.580382, 0.254931},
 784 |         {0.973416, 0.585761, 0.251540}, {0.974947, 0.591165, 0.248151}, {0.976428, 0.596595, 0.244767}, {0.977856, 0.602051, 0.241387},
 785 |         {0.979233, 0.607532, 0.238013}, {0.980556, 0.613039, 0.234646}, {0.981826, 0.618572, 0.231287}, {0.983041, 0.624131, 0.227937},
 786 |         {0.984199, 0.629718, 0.224595}, {0.985301, 0.635330, 0.221265}, {0.986345, 0.640969, 0.217948}, {0.987332, 0.646633, 0.214648},
 787 |         {0.988260, 0.652325, 0.211364}, {0.989128, 0.658043, 0.208100}, {0.989935, 0.663787, 0.204859}, {0.990681, 0.669558, 0.201642},
 788 |         {0.991365, 0.675355, 0.198453}, {0.991985, 0.681179, 0.195295}, {0.992541, 0.687030, 0.192170}, {0.993032, 0.692907, 0.189084},
 789 |         {0.993456, 0.698810, 0.186041}, {0.993814, 0.704741, 0.183043}, {0.994103, 0.710698, 0.180097}, {0.994324, 0.716681, 0.177208},
 790 |         {0.994474, 0.722691, 0.174381}, {0.994553, 0.728728, 0.171622}, {0.994561, 0.734791, 0.168938}, {0.994495, 0.740880, 0.166335},
 791 |         {0.994355, 0.746995, 0.163821}, {0.994141, 0.753137, 0.161404}, {0.993851, 0.759304, 0.159092}, {0.993482, 0.765499, 0.156891},
 792 |         {0.993033, 0.771720, 0.154808}, {0.992505, 0.777967, 0.152855}, {0.991897, 0.784239, 0.151042}, {0.991209, 0.790537, 0.149377},
 793 |         {0.990439, 0.796859, 0.147870}, {0.989587, 0.803205, 0.146529}, {0.988648, 0.809579, 0.145357}, {0.987621, 0.815978, 0.144363},
 794 |         {0.986509, 0.822401, 0.143557}, {0.985314, 0.828846, 0.142945}, {0.984031, 0.835315, 0.142528}, {0.982653, 0.841812, 0.142303},
 795 |         {0.981190, 0.848329, 0.142279}, {0.979644, 0.854866, 0.142453}, {0.977995, 0.861432, 0.142808}, {0.976265, 0.868016, 0.143351},
 796 |         {0.974443, 0.874622, 0.144061}, {0.972530, 0.881250, 0.144923}, {0.970533, 0.887896, 0.145919}, {0.968443, 0.894564, 0.147014},
 797 |         {0.966271, 0.901249, 0.148180}, {0.964021, 0.907950, 0.149370}, {0.961681, 0.914672, 0.150520}, {0.959276, 0.921407, 0.151566},
 798 |         {0.956808, 0.928152, 0.152409}, {0.954287, 0.934908, 0.152921}, {0.951726, 0.941671, 0.152925}, {0.949151, 0.948435, 0.152178},
 799 |         {0.946602, 0.955190, 0.150328}, {0.944152, 0.961916, 0.146861}, {0.941896, 0.968590, 0.140956}, {0.940015, 0.975158, 0.131326}};
 800 | 
 801 |     return internal::CalcLerp(x, data);
 802 | }
 803 | 
 804 | inline Color GetViridisColor(double x) {
 805 |     constexpr Color data[] = {
 806 |         {0.267004, 0.004874, 0.329415}, {0.268510, 0.009605, 0.335427}, {0.269944, 0.014625, 0.341379}, {0.271305, 0.019942, 0.347269},
 807 |         {0.272594, 0.025563, 0.353093}, {0.273809, 0.031497, 0.358853}, {0.274952, 0.037752, 0.364543}, {0.276022, 0.044167, 0.370164},
 808 |         {0.277018, 0.050344, 0.375715}, {0.277941, 0.056324, 0.381191}, {0.278791, 0.062145, 0.386592}, {0.279566, 0.067836, 0.391917},
 809 |         {0.280267, 0.073417, 0.397163}, {0.280894, 0.078907, 0.402329}, {0.281446, 0.084320, 0.407414}, {0.281924, 0.089666, 0.412415},
 810 |         {0.282327, 0.094955, 0.417331}, {0.282656, 0.100196, 0.422160}, {0.282910, 0.105393, 0.426902}, {0.283091, 0.110553, 0.431554},
 811 |         {0.283197, 0.115680, 0.436115}, {0.283229, 0.120777, 0.440584}, {0.283187, 0.125848, 0.444960}, {0.283072, 0.130895, 0.449241},
 812 |         {0.282884, 0.135920, 0.453427}, {0.282623, 0.140926, 0.457517}, {0.282290, 0.145912, 0.461510}, {0.281887, 0.150881, 0.465405},
 813 |         {0.281412, 0.155834, 0.469201}, {0.280868, 0.160771, 0.472899}, {0.280255, 0.165693, 0.476498}, {0.279574, 0.170599, 0.479997},
 814 |         {0.278826, 0.175490, 0.483397}, {0.278012, 0.180367, 0.486697}, {0.277134, 0.185228, 0.489898}, {0.276194, 0.190074, 0.493001},
 815 |         {0.275191, 0.194905, 0.496005}, {0.274128, 0.199721, 0.498911}, {0.273006, 0.204520, 0.501721}, {0.271828, 0.209303, 0.504434},
 816 |         {0.270595, 0.214069, 0.507052}, {0.269308, 0.218818, 0.509577}, {0.267968, 0.223549, 0.512008}, {0.266580, 0.228262, 0.514349},
 817 |         {0.265145, 0.232956, 0.516599}, {0.263663, 0.237631, 0.518762}, {0.262138, 0.242286, 0.520837}, {0.260571, 0.246922, 0.522828},
 818 |         {0.258965, 0.251537, 0.524736}, {0.257322, 0.256130, 0.526563}, {0.255645, 0.260703, 0.528312}, {0.253935, 0.265254, 0.529983},
 819 |         {0.252194, 0.269783, 0.531579}, {0.250425, 0.274290, 0.533103}, {0.248629, 0.278775, 0.534556}, {0.246811, 0.283237, 0.535941},
 820 |         {0.244972, 0.287675, 0.537260}, {0.243113, 0.292092, 0.538516}, {0.241237, 0.296485, 0.539709}, {0.239346, 0.300855, 0.540844},
 821 |         {0.237441, 0.305202, 0.541921}, {0.235526, 0.309527, 0.542944}, {0.233603, 0.313828, 0.543914}, {0.231674, 0.318106, 0.544834},
 822 |         {0.229739, 0.322361, 0.545706}, {0.227802, 0.326594, 0.546532}, {0.225863, 0.330805, 0.547314}, {0.223925, 0.334994, 0.548053},
 823 |         {0.221989, 0.339161, 0.548752}, {0.220057, 0.343307, 0.549413}, {0.218130, 0.347432, 0.550038}, {0.216210, 0.351535, 0.550627},
 824 |         {0.214298, 0.355619, 0.551184}, {0.212395, 0.359683, 0.551710}, {0.210503, 0.363727, 0.552206}, {0.208623, 0.367752, 0.552675},
 825 |         {0.206756, 0.371758, 0.553117}, {0.204903, 0.375746, 0.553533}, {0.203063, 0.379716, 0.553925}, {0.201239, 0.383670, 0.554294},
 826 |         {0.199430, 0.387607, 0.554642}, {0.197636, 0.391528, 0.554969}, {0.195860, 0.395433, 0.555276}, {0.194100, 0.399323, 0.555565},
 827 |         {0.192357, 0.403199, 0.555836}, {0.190631, 0.407061, 0.556089}, {0.188923, 0.410910, 0.556326}, {0.187231, 0.414746, 0.556547},
 828 |         {0.185556, 0.418570, 0.556753}, {0.183898, 0.422383, 0.556944}, {0.182256, 0.426184, 0.557120}, {0.180629, 0.429975, 0.557282},
 829 |         {0.179019, 0.433756, 0.557430}, {0.177423, 0.437527, 0.557565}, {0.175841, 0.441290, 0.557685}, {0.174274, 0.445044, 0.557792},
 830 |         {0.172719, 0.448791, 0.557885}, {0.171176, 0.452530, 0.557965}, {0.169646, 0.456262, 0.558030}, {0.168126, 0.459988, 0.558082},
 831 |         {0.166617, 0.463708, 0.558119}, {0.165117, 0.467423, 0.558141}, {0.163625, 0.471133, 0.558148}, {0.162142, 0.474838, 0.558140},
 832 |         {0.160665, 0.478540, 0.558115}, {0.159194, 0.482237, 0.558073}, {0.157729, 0.485932, 0.558013}, {0.156270, 0.489624, 0.557936},
 833 |         {0.154815, 0.493313, 0.557840}, {0.153364, 0.497000, 0.557724}, {0.151918, 0.500685, 0.557587}, {0.150476, 0.504369, 0.557430},
 834 |         {0.149039, 0.508051, 0.557250}, {0.147607, 0.511733, 0.557049}, {0.146180, 0.515413, 0.556823}, {0.144759, 0.519093, 0.556572},
 835 |         {0.143343, 0.522773, 0.556295}, {0.141935, 0.526453, 0.555991}, {0.140536, 0.530132, 0.555659}, {0.139147, 0.533812, 0.555298},
 836 |         {0.137770, 0.537492, 0.554906}, {0.136408, 0.541173, 0.554483}, {0.135066, 0.544853, 0.554029}, {0.133743, 0.548535, 0.553541},
 837 |         {0.132444, 0.552216, 0.553018}, {0.131172, 0.555899, 0.552459}, {0.129933, 0.559582, 0.551864}, {0.128729, 0.563265, 0.551229},
 838 |         {0.127568, 0.566949, 0.550556}, {0.126453, 0.570633, 0.549841}, {0.125394, 0.574318, 0.549086}, {0.124395, 0.578002, 0.548287},
 839 |         {0.123463, 0.581687, 0.547445}, {0.122606, 0.585371, 0.546557}, {0.121831, 0.589055, 0.545623}, {0.121148, 0.592739, 0.544641},
 840 |         {0.120565, 0.596422, 0.543611}, {0.120092, 0.600104, 0.542530}, {0.119738, 0.603785, 0.541400}, {0.119512, 0.607464, 0.540218},
 841 |         {0.119423, 0.611141, 0.538982}, {0.119483, 0.614817, 0.537692}, {0.119699, 0.618490, 0.536347}, {0.120081, 0.622161, 0.534946},
 842 |         {0.120638, 0.625828, 0.533488}, {0.121380, 0.629492, 0.531973}, {0.122312, 0.633153, 0.530398}, {0.123444, 0.636809, 0.528763},
 843 |         {0.124780, 0.640461, 0.527068}, {0.126326, 0.644107, 0.525311}, {0.128087, 0.647749, 0.523491}, {0.130067, 0.651384, 0.521608},
 844 |         {0.132268, 0.655014, 0.519661}, {0.134692, 0.658636, 0.517649}, {0.137339, 0.662252, 0.515571}, {0.140210, 0.665859, 0.513427},
 845 |         {0.143303, 0.669459, 0.511215}, {0.146616, 0.673050, 0.508936}, {0.150148, 0.676631, 0.506589}, {0.153894, 0.680203, 0.504172},
 846 |         {0.157851, 0.683765, 0.501686}, {0.162016, 0.687316, 0.499129}, {0.166383, 0.690856, 0.496502}, {0.170948, 0.694384, 0.493803},
 847 |         {0.175707, 0.697900, 0.491033}, {0.180653, 0.701402, 0.488189}, {0.185783, 0.704891, 0.485273}, {0.191090, 0.708366, 0.482284},
 848 |         {0.196571, 0.711827, 0.479221}, {0.202219, 0.715272, 0.476084}, {0.208030, 0.718701, 0.472873}, {0.214000, 0.722114, 0.469588},
 849 |         {0.220124, 0.725509, 0.466226}, {0.226397, 0.728888, 0.462789}, {0.232815, 0.732247, 0.459277}, {0.239374, 0.735588, 0.455688},
 850 |         {0.246070, 0.738910, 0.452024}, {0.252899, 0.742211, 0.448284}, {0.259857, 0.745492, 0.444467}, {0.266941, 0.748751, 0.440573},
 851 |         {0.274149, 0.751988, 0.436601}, {0.281477, 0.755203, 0.432552}, {0.288921, 0.758394, 0.428426}, {0.296479, 0.761561, 0.424223},
 852 |         {0.304148, 0.764704, 0.419943}, {0.311925, 0.767822, 0.415586}, {0.319809, 0.770914, 0.411152}, {0.327796, 0.773980, 0.406640},
 853 |         {0.335885, 0.777018, 0.402049}, {0.344074, 0.780029, 0.397381}, {0.352360, 0.783011, 0.392636}, {0.360741, 0.785964, 0.387814},
 854 |         {0.369214, 0.788888, 0.382914}, {0.377779, 0.791781, 0.377939}, {0.386433, 0.794644, 0.372886}, {0.395174, 0.797475, 0.367757},
 855 |         {0.404001, 0.800275, 0.362552}, {0.412913, 0.803041, 0.357269}, {0.421908, 0.805774, 0.351910}, {0.430983, 0.808473, 0.346476},
 856 |         {0.440137, 0.811138, 0.340967}, {0.449368, 0.813768, 0.335384}, {0.458674, 0.816363, 0.329727}, {0.468053, 0.818921, 0.323998},
 857 |         {0.477504, 0.821444, 0.318195}, {0.487026, 0.823929, 0.312321}, {0.496615, 0.826376, 0.306377}, {0.506271, 0.828786, 0.300362},
 858 |         {0.515992, 0.831158, 0.294279}, {0.525776, 0.833491, 0.288127}, {0.535621, 0.835785, 0.281908}, {0.545524, 0.838039, 0.275626},
 859 |         {0.555484, 0.840254, 0.269281}, {0.565498, 0.842430, 0.262877}, {0.575563, 0.844566, 0.256415}, {0.585678, 0.846661, 0.249897},
 860 |         {0.595839, 0.848717, 0.243329}, {0.606045, 0.850733, 0.236712}, {0.616293, 0.852709, 0.230052}, {0.626579, 0.854645, 0.223353},
 861 |         {0.636902, 0.856542, 0.216620}, {0.647257, 0.858400, 0.209861}, {0.657642, 0.860219, 0.203082}, {0.668054, 0.861999, 0.196293},
 862 |         {0.678489, 0.863742, 0.189503}, {0.688944, 0.865448, 0.182725}, {0.699415, 0.867117, 0.175971}, {0.709898, 0.868751, 0.169257},
 863 |         {0.720391, 0.870350, 0.162603}, {0.730889, 0.871916, 0.156029}, {0.741388, 0.873449, 0.149561}, {0.751884, 0.874951, 0.143228},
 864 |         {0.762373, 0.876424, 0.137064}, {0.772852, 0.877868, 0.131109}, {0.783315, 0.879285, 0.125405}, {0.793760, 0.880678, 0.120005},
 865 |         {0.804182, 0.882046, 0.114965}, {0.814576, 0.883393, 0.110347}, {0.824940, 0.884720, 0.106217}, {0.835270, 0.886029, 0.102646},
 866 |         {0.845561, 0.887322, 0.099702}, {0.855810, 0.888601, 0.097452}, {0.866013, 0.889868, 0.095953}, {0.876168, 0.891125, 0.095250},
 867 |         {0.886271, 0.892374, 0.095374}, {0.896320, 0.893616, 0.096335}, {0.906311, 0.894855, 0.098125}, {0.916242, 0.896091, 0.100717},
 868 |         {0.926106, 0.897330, 0.104071}, {0.935904, 0.898570, 0.108131}, {0.945636, 0.899815, 0.112838}, {0.955300, 0.901065, 0.118128},
 869 |         {0.964894, 0.902323, 0.123941}, {0.974417, 0.903590, 0.130215}, {0.983868, 0.904867, 0.136897}, {0.993248, 0.906157, 0.143936}};
 870 | 
 871 |     return internal::CalcLerp(x, data);
 872 | }
 873 | 
 874 | inline Color GetCividisColor(double x) {
 875 |     constexpr Color data[] = {
 876 |         {0.0000, 0.1262, 0.3015}, {0.0000, 0.1292, 0.3077}, {0.0000, 0.1321, 0.3142}, {0.0000, 0.1350, 0.3205}, {0.0000, 0.1379, 0.3269},
 877 |         {0.0000, 0.1408, 0.3334}, {0.0000, 0.1437, 0.3400}, {0.0000, 0.1465, 0.3467}, {0.0000, 0.1492, 0.3537}, {0.0000, 0.1519, 0.3606},
 878 |         {0.0000, 0.1546, 0.3676}, {0.0000, 0.1574, 0.3746}, {0.0000, 0.1601, 0.3817}, {0.0000, 0.1629, 0.3888}, {0.0000, 0.1657, 0.3960},
 879 |         {0.0000, 0.1685, 0.4031}, {0.0000, 0.1714, 0.4102}, {0.0000, 0.1743, 0.4172}, {0.0000, 0.1773, 0.4241}, {0.0000, 0.1798, 0.4307},
 880 |         {0.0000, 0.1817, 0.4347}, {0.0000, 0.1834, 0.4363}, {0.0000, 0.1852, 0.4368}, {0.0000, 0.1872, 0.4368}, {0.0000, 0.1901, 0.4365},
 881 |         {0.0000, 0.1930, 0.4361}, {0.0000, 0.1958, 0.4356}, {0.0000, 0.1987, 0.4349}, {0.0000, 0.2015, 0.4343}, {0.0000, 0.2044, 0.4336},
 882 |         {0.0000, 0.2073, 0.4329}, {0.0055, 0.2101, 0.4322}, {0.0236, 0.2130, 0.4314}, {0.0416, 0.2158, 0.4308}, {0.0576, 0.2187, 0.4301},
 883 |         {0.0710, 0.2215, 0.4293}, {0.0827, 0.2244, 0.4287}, {0.0932, 0.2272, 0.4280}, {0.1030, 0.2300, 0.4274}, {0.1120, 0.2329, 0.4268},
 884 |         {0.1204, 0.2357, 0.4262}, {0.1283, 0.2385, 0.4256}, {0.1359, 0.2414, 0.4251}, {0.1431, 0.2442, 0.4245}, {0.1500, 0.2470, 0.4241},
 885 |         {0.1566, 0.2498, 0.4236}, {0.1630, 0.2526, 0.4232}, {0.1692, 0.2555, 0.4228}, {0.1752, 0.2583, 0.4224}, {0.1811, 0.2611, 0.4220},
 886 |         {0.1868, 0.2639, 0.4217}, {0.1923, 0.2667, 0.4214}, {0.1977, 0.2695, 0.4212}, {0.2030, 0.2723, 0.4209}, {0.2082, 0.2751, 0.4207},
 887 |         {0.2133, 0.2780, 0.4205}, {0.2183, 0.2808, 0.4204}, {0.2232, 0.2836, 0.4203}, {0.2281, 0.2864, 0.4202}, {0.2328, 0.2892, 0.4201},
 888 |         {0.2375, 0.2920, 0.4200}, {0.2421, 0.2948, 0.4200}, {0.2466, 0.2976, 0.4200}, {0.2511, 0.3004, 0.4201}, {0.2556, 0.3032, 0.4201},
 889 |         {0.2599, 0.3060, 0.4202}, {0.2643, 0.3088, 0.4203}, {0.2686, 0.3116, 0.4205}, {0.2728, 0.3144, 0.4206}, {0.2770, 0.3172, 0.4208},
 890 |         {0.2811, 0.3200, 0.4210}, {0.2853, 0.3228, 0.4212}, {0.2894, 0.3256, 0.4215}, {0.2934, 0.3284, 0.4218}, {0.2974, 0.3312, 0.4221},
 891 |         {0.3014, 0.3340, 0.4224}, {0.3054, 0.3368, 0.4227}, {0.3093, 0.3396, 0.4231}, {0.3132, 0.3424, 0.4236}, {0.3170, 0.3453, 0.4240},
 892 |         {0.3209, 0.3481, 0.4244}, {0.3247, 0.3509, 0.4249}, {0.3285, 0.3537, 0.4254}, {0.3323, 0.3565, 0.4259}, {0.3361, 0.3593, 0.4264},
 893 |         {0.3398, 0.3622, 0.4270}, {0.3435, 0.3650, 0.4276}, {0.3472, 0.3678, 0.4282}, {0.3509, 0.3706, 0.4288}, {0.3546, 0.3734, 0.4294},
 894 |         {0.3582, 0.3763, 0.4302}, {0.3619, 0.3791, 0.4308}, {0.3655, 0.3819, 0.4316}, {0.3691, 0.3848, 0.4322}, {0.3727, 0.3876, 0.4331},
 895 |         {0.3763, 0.3904, 0.4338}, {0.3798, 0.3933, 0.4346}, {0.3834, 0.3961, 0.4355}, {0.3869, 0.3990, 0.4364}, {0.3905, 0.4018, 0.4372},
 896 |         {0.3940, 0.4047, 0.4381}, {0.3975, 0.4075, 0.4390}, {0.4010, 0.4104, 0.4400}, {0.4045, 0.4132, 0.4409}, {0.4080, 0.4161, 0.4419},
 897 |         {0.4114, 0.4189, 0.4430}, {0.4149, 0.4218, 0.4440}, {0.4183, 0.4247, 0.4450}, {0.4218, 0.4275, 0.4462}, {0.4252, 0.4304, 0.4473},
 898 |         {0.4286, 0.4333, 0.4485}, {0.4320, 0.4362, 0.4496}, {0.4354, 0.4390, 0.4508}, {0.4388, 0.4419, 0.4521}, {0.4422, 0.4448, 0.4534},
 899 |         {0.4456, 0.4477, 0.4547}, {0.4489, 0.4506, 0.4561}, {0.4523, 0.4535, 0.4575}, {0.4556, 0.4564, 0.4589}, {0.4589, 0.4593, 0.4604},
 900 |         {0.4622, 0.4622, 0.4620}, {0.4656, 0.4651, 0.4635}, {0.4689, 0.4680, 0.4650}, {0.4722, 0.4709, 0.4665}, {0.4756, 0.4738, 0.4679},
 901 |         {0.4790, 0.4767, 0.4691}, {0.4825, 0.4797, 0.4701}, {0.4861, 0.4826, 0.4707}, {0.4897, 0.4856, 0.4714}, {0.4934, 0.4886, 0.4719},
 902 |         {0.4971, 0.4915, 0.4723}, {0.5008, 0.4945, 0.4727}, {0.5045, 0.4975, 0.4730}, {0.5083, 0.5005, 0.4732}, {0.5121, 0.5035, 0.4734},
 903 |         {0.5158, 0.5065, 0.4736}, {0.5196, 0.5095, 0.4737}, {0.5234, 0.5125, 0.4738}, {0.5272, 0.5155, 0.4739}, {0.5310, 0.5186, 0.4739},
 904 |         {0.5349, 0.5216, 0.4738}, {0.5387, 0.5246, 0.4739}, {0.5425, 0.5277, 0.4738}, {0.5464, 0.5307, 0.4736}, {0.5502, 0.5338, 0.4735},
 905 |         {0.5541, 0.5368, 0.4733}, {0.5579, 0.5399, 0.4732}, {0.5618, 0.5430, 0.4729}, {0.5657, 0.5461, 0.4727}, {0.5696, 0.5491, 0.4723},
 906 |         {0.5735, 0.5522, 0.4720}, {0.5774, 0.5553, 0.4717}, {0.5813, 0.5584, 0.4714}, {0.5852, 0.5615, 0.4709}, {0.5892, 0.5646, 0.4705},
 907 |         {0.5931, 0.5678, 0.4701}, {0.5970, 0.5709, 0.4696}, {0.6010, 0.5740, 0.4691}, {0.6050, 0.5772, 0.4685}, {0.6089, 0.5803, 0.4680},
 908 |         {0.6129, 0.5835, 0.4673}, {0.6168, 0.5866, 0.4668}, {0.6208, 0.5898, 0.4662}, {0.6248, 0.5929, 0.4655}, {0.6288, 0.5961, 0.4649},
 909 |         {0.6328, 0.5993, 0.4641}, {0.6368, 0.6025, 0.4632}, {0.6408, 0.6057, 0.4625}, {0.6449, 0.6089, 0.4617}, {0.6489, 0.6121, 0.4609},
 910 |         {0.6529, 0.6153, 0.4600}, {0.6570, 0.6185, 0.4591}, {0.6610, 0.6217, 0.4583}, {0.6651, 0.6250, 0.4573}, {0.6691, 0.6282, 0.4562},
 911 |         {0.6732, 0.6315, 0.4553}, {0.6773, 0.6347, 0.4543}, {0.6813, 0.6380, 0.4532}, {0.6854, 0.6412, 0.4521}, {0.6895, 0.6445, 0.4511},
 912 |         {0.6936, 0.6478, 0.4499}, {0.6977, 0.6511, 0.4487}, {0.7018, 0.6544, 0.4475}, {0.7060, 0.6577, 0.4463}, {0.7101, 0.6610, 0.4450},
 913 |         {0.7142, 0.6643, 0.4437}, {0.7184, 0.6676, 0.4424}, {0.7225, 0.6710, 0.4409}, {0.7267, 0.6743, 0.4396}, {0.7308, 0.6776, 0.4382},
 914 |         {0.7350, 0.6810, 0.4368}, {0.7392, 0.6844, 0.4352}, {0.7434, 0.6877, 0.4338}, {0.7476, 0.6911, 0.4322}, {0.7518, 0.6945, 0.4307},
 915 |         {0.7560, 0.6979, 0.4290}, {0.7602, 0.7013, 0.4273}, {0.7644, 0.7047, 0.4258}, {0.7686, 0.7081, 0.4241}, {0.7729, 0.7115, 0.4223},
 916 |         {0.7771, 0.7150, 0.4205}, {0.7814, 0.7184, 0.4188}, {0.7856, 0.7218, 0.4168}, {0.7899, 0.7253, 0.4150}, {0.7942, 0.7288, 0.4129},
 917 |         {0.7985, 0.7322, 0.4111}, {0.8027, 0.7357, 0.4090}, {0.8070, 0.7392, 0.4070}, {0.8114, 0.7427, 0.4049}, {0.8157, 0.7462, 0.4028},
 918 |         {0.8200, 0.7497, 0.4007}, {0.8243, 0.7532, 0.3984}, {0.8287, 0.7568, 0.3961}, {0.8330, 0.7603, 0.3938}, {0.8374, 0.7639, 0.3915},
 919 |         {0.8417, 0.7674, 0.3892}, {0.8461, 0.7710, 0.3869}, {0.8505, 0.7745, 0.3843}, {0.8548, 0.7781, 0.3818}, {0.8592, 0.7817, 0.3793},
 920 |         {0.8636, 0.7853, 0.3766}, {0.8681, 0.7889, 0.3739}, {0.8725, 0.7926, 0.3712}, {0.8769, 0.7962, 0.3684}, {0.8813, 0.7998, 0.3657},
 921 |         {0.8858, 0.8035, 0.3627}, {0.8902, 0.8071, 0.3599}, {0.8947, 0.8108, 0.3569}, {0.8992, 0.8145, 0.3538}, {0.9037, 0.8182, 0.3507},
 922 |         {0.9082, 0.8219, 0.3474}, {0.9127, 0.8256, 0.3442}, {0.9172, 0.8293, 0.3409}, {0.9217, 0.8330, 0.3374}, {0.9262, 0.8367, 0.3340},
 923 |         {0.9308, 0.8405, 0.3306}, {0.9353, 0.8442, 0.3268}, {0.9399, 0.8480, 0.3232}, {0.9444, 0.8518, 0.3195}, {0.9490, 0.8556, 0.3155},
 924 |         {0.9536, 0.8593, 0.3116}, {0.9582, 0.8632, 0.3076}, {0.9628, 0.8670, 0.3034}, {0.9674, 0.8708, 0.2990}, {0.9721, 0.8746, 0.2947},
 925 |         {0.9767, 0.8785, 0.2901}, {0.9814, 0.8823, 0.2856}, {0.9860, 0.8862, 0.2807}, {0.9907, 0.8901, 0.2759}, {0.9954, 0.8940, 0.2708},
 926 |         {1.0000, 0.8979, 0.2655}, {1.0000, 0.9018, 0.2600}, {1.0000, 0.9057, 0.2593}, {1.0000, 0.9094, 0.2634}, {1.0000, 0.9131, 0.2680},
 927 |         {1.0000, 0.9169, 0.2731}};
 928 | 
 929 |     return internal::CalcLerp(x, data);
 930 | }
 931 | 
 932 | inline Color GetGithubColor(double x) {
 933 |     constexpr Color data[] = {{0.933333, 0.933333, 0.933333},
 934 |                               {0.776470, 0.894117, 0.545098},
 935 |                               {0.482352, 0.788235, 0.435294},
 936 |                               {0.137254, 0.603921, 0.231372},
 937 |                               {0.098039, 0.380392, 0.152941}};
 938 | 
 939 |     return internal::CalcLerp(x, data);
 940 | }
 941 | 
 942 | inline Color GetCubehelixColor(double x) {
 943 |     constexpr Color data[] = {
 944 |         {0.000000, 0.000000, 0.000000}, {0.006716, 0.002119, 0.005970}, {0.013252, 0.004287, 0.012162}, {0.019599, 0.006514, 0.018563},
 945 |         {0.025748, 0.008803, 0.025162}, {0.031691, 0.011164, 0.031946}, {0.037421, 0.013600, 0.038902}, {0.042932, 0.016118, 0.046016},
 946 |         {0.048217, 0.018724, 0.053275}, {0.053271, 0.021423, 0.060666}, {0.058090, 0.024220, 0.068173}, {0.062670, 0.027119, 0.075781},
 947 |         {0.067008, 0.030126, 0.083478}, {0.071101, 0.033243, 0.091246}, {0.074947, 0.036475, 0.099072}, {0.078546, 0.039824, 0.106939},
 948 |         {0.081898, 0.043295, 0.114834}, {0.085002, 0.046889, 0.122740}, {0.087860, 0.050609, 0.130643}, {0.090474, 0.054457, 0.138527},
 949 |         {0.092845, 0.058434, 0.146378}, {0.094978, 0.062542, 0.154180}, {0.096875, 0.066781, 0.161918}, {0.098542, 0.071152, 0.169579},
 950 |         {0.099984, 0.075655, 0.177147}, {0.101205, 0.080290, 0.184609}, {0.102212, 0.085055, 0.191951}, {0.103013, 0.089951, 0.199159},
 951 |         {0.103615, 0.094975, 0.206221}, {0.104025, 0.100126, 0.213124}, {0.104252, 0.105403, 0.219855}, {0.104305, 0.110801, 0.226402},
 952 |         {0.104194, 0.116320, 0.232755}, {0.103929, 0.121956, 0.238903}, {0.103519, 0.127705, 0.244834}, {0.102976, 0.133564, 0.250541},
 953 |         {0.102310, 0.139529, 0.256012}, {0.101534, 0.145596, 0.261240}, {0.100659, 0.151759, 0.266217}, {0.099697, 0.158016, 0.270935},
 954 |         {0.098661, 0.164359, 0.275388}, {0.097563, 0.170785, 0.279569}, {0.096415, 0.177287, 0.283474}, {0.095232, 0.183860, 0.287097},
 955 |         {0.094026, 0.190498, 0.290434}, {0.092810, 0.197194, 0.293483}, {0.091597, 0.203943, 0.296240}, {0.090402, 0.210739, 0.298703},
 956 |         {0.089237, 0.217573, 0.300873}, {0.088115, 0.224441, 0.302747}, {0.087051, 0.231334, 0.304327}, {0.086056, 0.238247, 0.305612},
 957 |         {0.085146, 0.245171, 0.306606}, {0.084331, 0.252101, 0.307310}, {0.083626, 0.259028, 0.307728}, {0.083043, 0.265946, 0.307863},
 958 |         {0.082594, 0.272848, 0.307720}, {0.082291, 0.279726, 0.307304}, {0.082148, 0.286573, 0.306621}, {0.082174, 0.293383, 0.305677},
 959 |         {0.082381, 0.300147, 0.304480}, {0.082780, 0.306860, 0.303037}, {0.083383, 0.313514, 0.301356}, {0.084198, 0.320102, 0.299448},
 960 |         {0.085235, 0.326618, 0.297320}, {0.086504, 0.333055, 0.294984}, {0.088014, 0.339406, 0.292449}, {0.089772, 0.345666, 0.289728},
 961 |         {0.091787, 0.351829, 0.286831}, {0.094066, 0.357887, 0.283771}, {0.096615, 0.363836, 0.280560}, {0.099441, 0.369671, 0.277211},
 962 |         {0.102549, 0.375385, 0.273736}, {0.105944, 0.380974, 0.270151}, {0.109630, 0.386433, 0.266468}, {0.113611, 0.391757, 0.262703},
 963 |         {0.117891, 0.396943, 0.258868}, {0.122472, 0.401985, 0.254979}, {0.127356, 0.406881, 0.251051}, {0.132543, 0.411627, 0.247099},
 964 |         {0.138035, 0.416220, 0.243137}, {0.143832, 0.420656, 0.239182}, {0.149933, 0.424934, 0.235247}, {0.156336, 0.429052, 0.231350},
 965 |         {0.163040, 0.433007, 0.227504}, {0.170042, 0.436798, 0.223726}, {0.177339, 0.440423, 0.220029}, {0.184927, 0.443882, 0.216431},
 966 |         {0.192802, 0.447175, 0.212944}, {0.200958, 0.450301, 0.209585}, {0.209391, 0.453260, 0.206367}, {0.218092, 0.456053, 0.203306},
 967 |         {0.227057, 0.458680, 0.200415}, {0.236277, 0.461144, 0.197707}, {0.245744, 0.463444, 0.195197}, {0.255451, 0.465584, 0.192898},
 968 |         {0.265388, 0.467565, 0.190822}, {0.275545, 0.469391, 0.188982}, {0.285913, 0.471062, 0.187389}, {0.296481, 0.472584, 0.186055},
 969 |         {0.307239, 0.473959, 0.184992}, {0.318175, 0.475191, 0.184208}, {0.329277, 0.476285, 0.183716}, {0.340534, 0.477243, 0.183523},
 970 |         {0.351934, 0.478072, 0.183638}, {0.363463, 0.478776, 0.184071}, {0.375109, 0.479360, 0.184829}, {0.386858, 0.479829, 0.185918},
 971 |         {0.398697, 0.480190, 0.187345}, {0.410613, 0.480448, 0.189115}, {0.422591, 0.480609, 0.191235}, {0.434618, 0.480679, 0.193708},
 972 |         {0.446680, 0.480665, 0.196538}, {0.458762, 0.480574, 0.199728}, {0.470850, 0.480412, 0.203280}, {0.482930, 0.480186, 0.207197},
 973 |         {0.494987, 0.479903, 0.211478}, {0.507008, 0.479572, 0.216124}, {0.518978, 0.479198, 0.221136}, {0.530883, 0.478789, 0.226510},
 974 |         {0.542708, 0.478353, 0.232247}, {0.554441, 0.477898, 0.238342}, {0.566067, 0.477430, 0.244794}, {0.577573, 0.476958, 0.251597},
 975 |         {0.588945, 0.476490, 0.258747}, {0.600171, 0.476032, 0.266239}, {0.611237, 0.475592, 0.274067}, {0.622132, 0.475178, 0.282223},
 976 |         {0.632842, 0.474798, 0.290702}, {0.643357, 0.474459, 0.299495}, {0.653665, 0.474168, 0.308593}, {0.663755, 0.473933, 0.317987},
 977 |         {0.673616, 0.473761, 0.327668}, {0.683239, 0.473658, 0.337626}, {0.692613, 0.473632, 0.347849}, {0.701729, 0.473690, 0.358327},
 978 |         {0.710579, 0.473838, 0.369047}, {0.719155, 0.474083, 0.379998}, {0.727448, 0.474430, 0.391167}, {0.735453, 0.474886, 0.402541},
 979 |         {0.743162, 0.475457, 0.414106}, {0.750569, 0.476148, 0.425849}, {0.757669, 0.476964, 0.437755}, {0.764458, 0.477911, 0.449811},
 980 |         {0.770932, 0.478994, 0.462001}, {0.777086, 0.480216, 0.474310}, {0.782918, 0.481583, 0.486725}, {0.788426, 0.483098, 0.499228},
 981 |         {0.793609, 0.484765, 0.511805}, {0.798465, 0.486587, 0.524441}, {0.802993, 0.488567, 0.537119}, {0.807196, 0.490708, 0.549824},
 982 |         {0.811072, 0.493013, 0.562540}, {0.814625, 0.495483, 0.575253}, {0.817855, 0.498121, 0.587945}, {0.820767, 0.500927, 0.600602},
 983 |         {0.823364, 0.503903, 0.613208}, {0.825649, 0.507050, 0.625748}, {0.827628, 0.510368, 0.638207}, {0.829305, 0.513857, 0.650570},
 984 |         {0.830688, 0.517516, 0.662822}, {0.831781, 0.521346, 0.674949}, {0.832593, 0.525345, 0.686938}, {0.833130, 0.529511, 0.698773},
 985 |         {0.833402, 0.533844, 0.710443}, {0.833416, 0.538342, 0.721933}, {0.833181, 0.543001, 0.733232}, {0.832708, 0.547820, 0.744327},
 986 |         {0.832006, 0.552795, 0.755206}, {0.831086, 0.557924, 0.765859}, {0.829958, 0.563202, 0.776274}, {0.828633, 0.568627, 0.786443},
 987 |         {0.827124, 0.574193, 0.796354}, {0.825442, 0.579897, 0.805999}, {0.823599, 0.585733, 0.815370}, {0.821608, 0.591698, 0.824459},
 988 |         {0.819482, 0.597785, 0.833258}, {0.817233, 0.603990, 0.841761}, {0.814875, 0.610307, 0.849963}, {0.812421, 0.616730, 0.857858},
 989 |         {0.809884, 0.623252, 0.865441}, {0.807278, 0.629869, 0.872709}, {0.804617, 0.636573, 0.879658}, {0.801914, 0.643359, 0.886286},
 990 |         {0.799183, 0.650218, 0.892592}, {0.796438, 0.657146, 0.898574}, {0.793692, 0.664134, 0.904231}, {0.790959, 0.671176, 0.909565},
 991 |         {0.788253, 0.678264, 0.914576}, {0.785586, 0.685392, 0.919267}, {0.782973, 0.692553, 0.923639}, {0.780425, 0.699738, 0.927695},
 992 |         {0.777957, 0.706942, 0.931441}, {0.775579, 0.714157, 0.934879}, {0.773305, 0.721375, 0.938016}, {0.771147, 0.728589, 0.940857},
 993 |         {0.769116, 0.735793, 0.943409}, {0.767224, 0.742979, 0.945678}, {0.765481, 0.750140, 0.947673}, {0.763898, 0.757269, 0.949402},
 994 |         {0.762485, 0.764360, 0.950874}, {0.761251, 0.771405, 0.952098}, {0.760207, 0.778399, 0.953084}, {0.759360, 0.785335, 0.953843},
 995 |         {0.758718, 0.792207, 0.954386}, {0.758290, 0.799008, 0.954724}, {0.758082, 0.805734, 0.954869}, {0.758101, 0.812378, 0.954833},
 996 |         {0.758353, 0.818934, 0.954629}, {0.758842, 0.825399, 0.954270}, {0.759575, 0.831767, 0.953769}, {0.760554, 0.838033, 0.953140},
 997 |         {0.761784, 0.844192, 0.952397}, {0.763267, 0.850242, 0.951554}, {0.765006, 0.856178, 0.950625}, {0.767001, 0.861997, 0.949624},
 998 |         {0.769255, 0.867695, 0.948567}, {0.771766, 0.873270, 0.947467}, {0.774535, 0.878718, 0.946340}, {0.777561, 0.884039, 0.945201},
 999 |         {0.780841, 0.889230, 0.944063}, {0.784374, 0.894289, 0.942942}, {0.788156, 0.899216, 0.941853}, {0.792184, 0.904010, 0.940809},
1000 |         {0.796453, 0.908669, 0.939825}, {0.800958, 0.913194, 0.938916}, {0.805694, 0.917586, 0.938095}, {0.810654, 0.921845, 0.937376},
1001 |         {0.815832, 0.925971, 0.936772}, {0.821221, 0.929967, 0.936297}, {0.826811, 0.933833, 0.935962}, {0.832595, 0.937572, 0.935781},
1002 |         {0.838565, 0.941187, 0.935766}, {0.844709, 0.944679, 0.935927}, {0.851018, 0.948053, 0.936275}, {0.857482, 0.951311, 0.936822},
1003 |         {0.864090, 0.954457, 0.937578}, {0.870830, 0.957495, 0.938550}, {0.877690, 0.960430, 0.939749}, {0.884659, 0.963266, 0.941183},
1004 |         {0.891723, 0.966009, 0.942858}, {0.898871, 0.968662, 0.944783}, {0.906088, 0.971233, 0.946962}, {0.913362, 0.973726, 0.949402},
1005 |         {0.920679, 0.976147, 0.952108}, {0.928026, 0.978504, 0.955083}, {0.935387, 0.980802, 0.958331}, {0.942750, 0.983048, 0.961854},
1006 |         {0.950101, 0.985249, 0.965654}, {0.957424, 0.987412, 0.969733}, {0.964706, 0.989543, 0.974090}, {0.971932, 0.991652, 0.978724},
1007 |         {0.979088, 0.993744, 0.983635}, {0.986161, 0.995828, 0.988820}, {0.993136, 0.997910, 0.994276}, {1.000000, 1.000000, 1.000000}};
1008 | 
1009 |     return internal::CalcLerp(x, data);
1010 | }
1011 | inline Color GetHSVColor(double x) {
1012 |     constexpr Color data[] = {
1013 |         {1.0000, 0.0000, 0.0000}, {1.0000, 0.0234, 0.0000}, {1.0000, 0.0469, 0.0000}, {1.0000, 0.0703, 0.0000}, {1.0000, 0.0938, 0.0000},
1014 |         {1.0000, 0.1172, 0.0000}, {1.0000, 0.1406, 0.0000}, {1.0000, 0.1641, 0.0000}, {1.0000, 0.1875, 0.0000}, {1.0000, 0.2109, 0.0000},
1015 |         {1.0000, 0.2344, 0.0000}, {1.0000, 0.2578, 0.0000}, {1.0000, 0.2812, 0.0000}, {1.0000, 0.3047, 0.0000}, {1.0000, 0.3281, 0.0000},
1016 |         {1.0000, 0.3516, 0.0000}, {1.0000, 0.3750, 0.0000}, {1.0000, 0.3984, 0.0000}, {1.0000, 0.4219, 0.0000}, {1.0000, 0.4453, 0.0000},
1017 |         {1.0000, 0.4688, 0.0000}, {1.0000, 0.4922, 0.0000}, {1.0000, 0.5156, 0.0000}, {1.0000, 0.5391, 0.0000}, {1.0000, 0.5625, 0.0000},
1018 |         {1.0000, 0.5859, 0.0000}, {1.0000, 0.6094, 0.0000}, {1.0000, 0.6328, 0.0000}, {1.0000, 0.6562, 0.0000}, {1.0000, 0.6797, 0.0000},
1019 |         {1.0000, 0.7031, 0.0000}, {1.0000, 0.7266, 0.0000}, {1.0000, 0.7500, 0.0000}, {1.0000, 0.7734, 0.0000}, {1.0000, 0.7969, 0.0000},
1020 |         {1.0000, 0.8203, 0.0000}, {1.0000, 0.8438, 0.0000}, {1.0000, 0.8672, 0.0000}, {1.0000, 0.8906, 0.0000}, {1.0000, 0.9141, 0.0000},
1021 |         {1.0000, 0.9375, 0.0000}, {1.0000, 0.9609, 0.0000}, {1.0000, 0.9844, 0.0000}, {0.9922, 1.0000, 0.0000}, {0.9688, 1.0000, 0.0000},
1022 |         {0.9453, 1.0000, 0.0000}, {0.9219, 1.0000, 0.0000}, {0.8984, 1.0000, 0.0000}, {0.8750, 1.0000, 0.0000}, {0.8516, 1.0000, 0.0000},
1023 |         {0.8281, 1.0000, 0.0000}, {0.8047, 1.0000, 0.0000}, {0.7812, 1.0000, 0.0000}, {0.7578, 1.0000, 0.0000}, {0.7344, 1.0000, 0.0000},
1024 |         {0.7109, 1.0000, 0.0000}, {0.6875, 1.0000, 0.0000}, {0.6641, 1.0000, 0.0000}, {0.6406, 1.0000, 0.0000}, {0.6172, 1.0000, 0.0000},
1025 |         {0.5938, 1.0000, 0.0000}, {0.5703, 1.0000, 0.0000}, {0.5469, 1.0000, 0.0000}, {0.5234, 1.0000, 0.0000}, {0.5000, 1.0000, 0.0000},
1026 |         {0.4766, 1.0000, 0.0000}, {0.4531, 1.0000, 0.0000}, {0.4297, 1.0000, 0.0000}, {0.4062, 1.0000, 0.0000}, {0.3828, 1.0000, 0.0000},
1027 |         {0.3594, 1.0000, 0.0000}, {0.3359, 1.0000, 0.0000}, {0.3125, 1.0000, 0.0000}, {0.2891, 1.0000, 0.0000}, {0.2656, 1.0000, 0.0000},
1028 |         {0.2422, 1.0000, 0.0000}, {0.2188, 1.0000, 0.0000}, {0.1953, 1.0000, 0.0000}, {0.1719, 1.0000, 0.0000}, {0.1484, 1.0000, 0.0000},
1029 |         {0.1250, 1.0000, 0.0000}, {0.1016, 1.0000, 0.0000}, {0.0781, 1.0000, 0.0000}, {0.0547, 1.0000, 0.0000}, {0.0312, 1.0000, 0.0000},
1030 |         {0.0078, 1.0000, 0.0000}, {0.0000, 1.0000, 0.0156}, {0.0000, 1.0000, 0.0391}, {0.0000, 1.0000, 0.0625}, {0.0000, 1.0000, 0.0859},
1031 |         {0.0000, 1.0000, 0.1094}, {0.0000, 1.0000, 0.1328}, {0.0000, 1.0000, 0.1562}, {0.0000, 1.0000, 0.1797}, {0.0000, 1.0000, 0.2031},
1032 |         {0.0000, 1.0000, 0.2266}, {0.0000, 1.0000, 0.2500}, {0.0000, 1.0000, 0.2734}, {0.0000, 1.0000, 0.2969}, {0.0000, 1.0000, 0.3203},
1033 |         {0.0000, 1.0000, 0.3438}, {0.0000, 1.0000, 0.3672}, {0.0000, 1.0000, 0.3906}, {0.0000, 1.0000, 0.4141}, {0.0000, 1.0000, 0.4375},
1034 |         {0.0000, 1.0000, 0.4609}, {0.0000, 1.0000, 0.4844}, {0.0000, 1.0000, 0.5078}, {0.0000, 1.0000, 0.5312}, {0.0000, 1.0000, 0.5547},
1035 |         {0.0000, 1.0000, 0.5781}, {0.0000, 1.0000, 0.6016}, {0.0000, 1.0000, 0.6250}, {0.0000, 1.0000, 0.6484}, {0.0000, 1.0000, 0.6719},
1036 |         {0.0000, 1.0000, 0.6953}, {0.0000, 1.0000, 0.7188}, {0.0000, 1.0000, 0.7422}, {0.0000, 1.0000, 0.7656}, {0.0000, 1.0000, 0.7891},
1037 |         {0.0000, 1.0000, 0.8125}, {0.0000, 1.0000, 0.8359}, {0.0000, 1.0000, 0.8594}, {0.0000, 1.0000, 0.8828}, {0.0000, 1.0000, 0.9062},
1038 |         {0.0000, 1.0000, 0.9297}, {0.0000, 1.0000, 0.9531}, {0.0000, 1.0000, 0.9766}, {0.0000, 1.0000, 1.0000}, {0.0000, 0.9766, 1.0000},
1039 |         {0.0000, 0.9531, 1.0000}, {0.0000, 0.9297, 1.0000}, {0.0000, 0.9062, 1.0000}, {0.0000, 0.8828, 1.0000}, {0.0000, 0.8594, 1.0000},
1040 |         {0.0000, 0.8359, 1.0000}, {0.0000, 0.8125, 1.0000}, {0.0000, 0.7891, 1.0000}, {0.0000, 0.7656, 1.0000}, {0.0000, 0.7422, 1.0000},
1041 |         {0.0000, 0.7188, 1.0000}, {0.0000, 0.6953, 1.0000}, {0.0000, 0.6719, 1.0000}, {0.0000, 0.6484, 1.0000}, {0.0000, 0.6250, 1.0000},
1042 |         {0.0000, 0.6016, 1.0000}, {0.0000, 0.5781, 1.0000}, {0.0000, 0.5547, 1.0000}, {0.0000, 0.5312, 1.0000}, {0.0000, 0.5078, 1.0000},
1043 |         {0.0000, 0.4844, 1.0000}, {0.0000, 0.4609, 1.0000}, {0.0000, 0.4375, 1.0000}, {0.0000, 0.4141, 1.0000}, {0.0000, 0.3906, 1.0000},
1044 |         {0.0000, 0.3672, 1.0000}, {0.0000, 0.3438, 1.0000}, {0.0000, 0.3203, 1.0000}, {0.0000, 0.2969, 1.0000}, {0.0000, 0.2734, 1.0000},
1045 |         {0.0000, 0.2500, 1.0000}, {0.0000, 0.2266, 1.0000}, {0.0000, 0.2031, 1.0000}, {0.0000, 0.1797, 1.0000}, {0.0000, 0.1562, 1.0000},
1046 |         {0.0000, 0.1328, 1.0000}, {0.0000, 0.1094, 1.0000}, {0.0000, 0.0859, 1.0000}, {0.0000, 0.0625, 1.0000}, {0.0000, 0.0391, 1.0000},
1047 |         {0.0000, 0.0156, 1.0000}, {0.0078, 0.0000, 1.0000}, {0.0312, 0.0000, 1.0000}, {0.0547, 0.0000, 1.0000}, {0.0781, 0.0000, 1.0000},
1048 |         {0.1016, 0.0000, 1.0000}, {0.1250, 0.0000, 1.0000}, {0.1484, 0.0000, 1.0000}, {0.1719, 0.0000, 1.0000}, {0.1953, 0.0000, 1.0000},
1049 |         {0.2188, 0.0000, 1.0000}, {0.2422, 0.0000, 1.0000}, {0.2656, 0.0000, 1.0000}, {0.2891, 0.0000, 1.0000}, {0.3125, 0.0000, 1.0000},
1050 |         {0.3359, 0.0000, 1.0000}, {0.3594, 0.0000, 1.0000}, {0.3828, 0.0000, 1.0000}, {0.4062, 0.0000, 1.0000}, {0.4297, 0.0000, 1.0000},
1051 |         {0.4531, 0.0000, 1.0000}, {0.4766, 0.0000, 1.0000}, {0.5000, 0.0000, 1.0000}, {0.5234, 0.0000, 1.0000}, {0.5469, 0.0000, 1.0000},
1052 |         {0.5703, 0.0000, 1.0000}, {0.5938, 0.0000, 1.0000}, {0.6172, 0.0000, 1.0000}, {0.6406, 0.0000, 1.0000}, {0.6641, 0.0000, 1.0000},
1053 |         {0.6875, 0.0000, 1.0000}, {0.7109, 0.0000, 1.0000}, {0.7344, 0.0000, 1.0000}, {0.7578, 0.0000, 1.0000}, {0.7812, 0.0000, 1.0000},
1054 |         {0.8047, 0.0000, 1.0000}, {0.8281, 0.0000, 1.0000}, {0.8516, 0.0000, 1.0000}, {0.8750, 0.0000, 1.0000}, {0.8984, 0.0000, 1.0000},
1055 |         {0.9219, 0.0000, 1.0000}, {0.9453, 0.0000, 1.0000}, {0.9688, 0.0000, 1.0000}, {0.9922, 0.0000, 1.0000}, {1.0000, 0.0000, 0.9844},
1056 |         {1.0000, 0.0000, 0.9609}, {1.0000, 0.0000, 0.9375}, {1.0000, 0.0000, 0.9141}, {1.0000, 0.0000, 0.8906}, {1.0000, 0.0000, 0.8672},
1057 |         {1.0000, 0.0000, 0.8438}, {1.0000, 0.0000, 0.8203}, {1.0000, 0.0000, 0.7969}, {1.0000, 0.0000, 0.7734}, {1.0000, 0.0000, 0.7500},
1058 |         {1.0000, 0.0000, 0.7266}, {1.0000, 0.0000, 0.7031}, {1.0000, 0.0000, 0.6797}, {1.0000, 0.0000, 0.6562}, {1.0000, 0.0000, 0.6328},
1059 |         {1.0000, 0.0000, 0.6094}, {1.0000, 0.0000, 0.5859}, {1.0000, 0.0000, 0.5625}, {1.0000, 0.0000, 0.5391}, {1.0000, 0.0000, 0.5156},
1060 |         {1.0000, 0.0000, 0.4922}, {1.0000, 0.0000, 0.4688}, {1.0000, 0.0000, 0.4453}, {1.0000, 0.0000, 0.4219}, {1.0000, 0.0000, 0.3984},
1061 |         {1.0000, 0.0000, 0.3750}, {1.0000, 0.0000, 0.3516}, {1.0000, 0.0000, 0.3281}, {1.0000, 0.0000, 0.3047}, {1.0000, 0.0000, 0.2812},
1062 |         {1.0000, 0.0000, 0.2578}, {1.0000, 0.0000, 0.2344}, {1.0000, 0.0000, 0.2109}, {1.0000, 0.0000, 0.1875}, {1.0000, 0.0000, 0.1641},
1063 |         {1.0000, 0.0000, 0.1406}, {1.0000, 0.0000, 0.1172}, {1.0000, 0.0000, 0.0938}, {1.0000, 0.0000, 0.0703}, {1.0000, 0.0000, 0.0469},
1064 |         {1.0000, 0.0000, 0.0234},
1065 |     };
1066 |     return internal::CalcLerp(x, data);
1067 | }
1068 | 
1069 | #if defined(TINYCOLORMAP_WITH_QT5) && defined(TINYCOLORMAP_WITH_EIGEN)
1070 | inline QImage CreateMatrixVisualization(const Eigen::MatrixXd& matrix) {
1071 |     const int w = matrix.cols();
1072 |     const int h = matrix.rows();
1073 |     const double max_coeff = matrix.maxCoeff();
1074 |     const double min_coeff = matrix.minCoeff();
1075 |     const Eigen::MatrixXd normalized = (1.0 / (max_coeff - min_coeff)) * (matrix - Eigen::MatrixXd::Constant(h, w, min_coeff));
1076 | 
1077 |     QImage image(w, h, QImage::Format_ARGB32);
1078 |     for (int x = 0; x < w; ++x) {
1079 |         for (int y = 0; y < h; ++y) {
1080 |             const QColor color = tinycolormap::GetColor(normalized(y, x)).ConvertToQColor();
1081 |             image.setPixel(x, y, color.rgb());
1082 |         }
1083 |     }
1084 | 
1085 |     return image;
1086 | }
1087 | 
1088 | inline void ExportMatrixVisualization(const Eigen::MatrixXd& matrix, const std::string& path) {
1089 |     CreateMatrixVisualization(matrix).save(QString::fromStdString(path));
1090 | }
1091 | #endif
1092 | }  // namespace tinycolormap
1093 | 
1094 | #endif

```

`src\reference_waypoint_loader\launch\reference_waypoint_loader.launch`:

```launch
   1 | <launch>
   2 |     <node pkg="reference_waypoint_loader" type="reference_waypoint_loader_node" name="reference_waypoint_loader" output="screen">
   3 |         <!-- <param name="reference_waypoint_csv" value="$(find reference_waypoint_loader)/data/reference_waypoint.csv" /> -->
   4 |         <param name="reference_waypoint_topic" value="/reference_waypoint" />
   5 |         <param name="reference_path_topic" value="/reference_path" />
   6 |         <param name="reference_rviz_marker_topic" value="/rviz_reference_marker" />
   7 |         <param name="reference_waypoint_frame_id" value="map" />
   8 |         <param name="reference_waypoint_x_column_label" value="opt_x" />
   9 |         <param name="reference_waypoint_y_column_label" value="opt_y" />
  10 |         <param name="reference_waypoint_v_column_label" value="ref_v" />
  11 |         <param name="reference_velocity_scale" value="1.0" />
  12 |         <param name="reference_velocity_min" value="0.8" />
  13 |         <param name="reference_velocity_max" value="10" />
  14 |     </node>
  15 | </launch>

```

`src\reference_waypoint_loader\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>reference_waypoint_loader</name>
   4 |   <version>0.0.0</version>
   5 |   <description>The reference_waypoint_loader package</description>
   6 | 
   7 |   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8 |   <!-- Example:  -->
   9 |   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 |   <maintainer email="mizuho@todo.todo">mizuho</maintainer>
  11 | 
  12 | 
  13 |   <!-- One license tag required, multiple allowed, one license per tag -->
  14 |   <!-- Commonly used license strings: -->
  15 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  16 |   <license>TODO</license>
  17 | 
  18 |   <buildtool_depend>catkin</buildtool_depend>
  19 |   <build_depend>waypoint_msgs</build_depend>
  20 |   <build_depend>nav_msgs</build_depend>
  21 |   <build_depend>visualization_msgs</build_depend>
  22 | 
  23 |   <build_depend>roscpp</build_depend>
  24 |   
  25 |   <build_export_depend>nav_msgs</build_export_depend>
  26 |   <build_export_depend>visualization_msgs</build_export_depend>
  27 |   <build_export_depend>roscpp</build_export_depend>
  28 |   
  29 |   <exec_depend>nav_msgs</exec_depend>
  30 |   <exec_depend>visualization_msgs</exec_depend>
  31 |   <exec_depend>roscpp</exec_depend>
  32 | 
  33 |   <!-- The export tag contains other, unspecified, tags -->
  34 |   <export>
  35 |     <!-- Other tools can request additional information be placed here -->
  36 | 
  37 |   </export>
  38 | </package>

```

`src\reference_waypoint_loader\src\reference_waypoint_loader.cpp`:

```cpp
   1 | #include "reference_waypoint_loader/reference_waypoint_loader.hpp"
   2 | 
   3 | namespace planning {
   4 | 
   5 | ReferenceWaypointLoader::ReferenceWaypointLoader() : nh_(), private_nh_("~") {
   6 |     private_nh_.param<std::string>("reference_waypoint_csv", csv_waypoint_, "");
   7 |     private_nh_.param<std::string>("map_frame", map_frame_, "map");
   8 |     private_nh_.param<std::string>("reference_waypoint_topic", topic_name_waypoint_, "reference_waypoint");
   9 |     private_nh_.param<std::string>("reference_path_topic", topic_name_path_, "reference_path");
  10 |     private_nh_.param<std::string>("reference_rviz_marker_topic", topic_name_rviz_waypoint_marker_, "rviz_waypoint_marker");
  11 |     private_nh_.param<std::string>("reference_waypoint_x_column_label", ref_x_label_, "opt_x");
  12 |     private_nh_.param<std::string>("reference_waypoint_y_column_label", ref_y_label_, "opt_y");
  13 |     private_nh_.param<std::string>("reference_waypoint_v_column_label", ref_v_label_, "ref_v");
  14 |     private_nh_.param<float>("reference_velocity_scale", ref_v_scale_, 1.0);
  15 |     private_nh_.param<float>("scaled_velocity_min", scaled_v_min_, 0.0);
  16 |     private_nh_.param<float>("scaled_velocity_max", scaled_v_max_, 100.0);
  17 | 
  18 |     if (!std::filesystem::exists(csv_waypoint_)) {
  19 |         ROS_ERROR("reference path file does not exist: %s", csv_waypoint_.c_str());
  20 |         exit(1);
  21 |     }
  22 | 
  23 |     // load csv file
  24 |     const rapidcsv::Document doc(csv_waypoint_);
  25 | 
  26 |     // get csv data
  27 |     const std::vector<double> x = doc.GetColumn<double>(ref_x_label_);
  28 |     const std::vector<double> y = doc.GetColumn<double>(ref_y_label_);
  29 |     const std::vector<double> v = doc.GetColumn<double>(ref_v_label_);
  30 |     const double ref_v_max = *max_element(v.begin(), v.end());  // max reference velocity
  31 |     const double ref_v_min = *min_element(v.begin(), v.end());  // min reference velocity
  32 | 
  33 |     // publish as waypoint_msgs::Waypoint (= List of "Pose + Twist")
  34 |     waypoint_msgs::Waypoint reference_waypoint;
  35 | 
  36 |     // publish path as well for visualization with nav_msgs::Path (= List of "Pose")
  37 |     nav_msgs::Path reference_path;
  38 | 
  39 |     // set waypoint header
  40 |     reference_waypoint.header.frame_id = map_frame_;
  41 |     reference_waypoint.header.stamp = ros::Time::now();
  42 | 
  43 |     // set path header
  44 |     reference_path.header.frame_id = map_frame_;
  45 |     reference_path.header.stamp = ros::Time::now();
  46 | 
  47 |     // declare marker array
  48 |     visualization_msgs::MarkerArray polygon_markers;
  49 |     polygon_markers.markers.clear();
  50 | 
  51 |     // prepare marker template
  52 |     int marker_id = 0;
  53 |     visualization_msgs::Marker marker_template;
  54 |     marker_template.header.frame_id = map_frame_;
  55 |     marker_template.header.stamp = ros::Time();
  56 |     marker_template.ns = "waypoints";
  57 |     marker_template.id = marker_id;
  58 |     marker_template.type = visualization_msgs::Marker::LINE_STRIP;
  59 |     marker_template.action = visualization_msgs::Marker::ADD;
  60 |     marker_template.pose.position.x = 0.0;
  61 |     marker_template.pose.position.y = 0.0;
  62 |     marker_template.pose.position.z = 0.01;
  63 |     marker_template.pose.orientation.x = 0.0;
  64 |     marker_template.pose.orientation.y = 0.0;
  65 |     marker_template.pose.orientation.z = 0.0;
  66 |     marker_template.pose.orientation.w = 1.0;
  67 |     marker_template.scale.x = 0.1;
  68 |     marker_template.scale.y = 0.1;
  69 |     marker_template.scale.z = 0.1;
  70 |     marker_template.color.a = 0.5;
  71 |     marker_template.color.r = 1.0;
  72 |     marker_template.color.g = 0.0;
  73 |     marker_template.color.b = 0.0;
  74 | 
  75 |     // initialize buffer to keep previous marker position
  76 |     geometry_msgs::Point prev_marker_point;
  77 |     prev_marker_point.x = x[0];
  78 |     prev_marker_point.y = y[0];
  79 |     prev_marker_point.z = 0.0;
  80 | 
  81 |     for (int i = 0; i < x.size(); i++) {
  82 |         // add reference pose info
  83 |         geometry_msgs::PoseStamped pose;
  84 |         pose.pose.position.x = x[i];
  85 |         pose.pose.position.y = y[i];
  86 |         pose.pose.position.z = 0.0;
  87 |         reference_waypoint.poses.push_back(pose);
  88 |         reference_path.poses.push_back(pose);
  89 | 
  90 |         // add reference velocity info
  91 |         geometry_msgs::TwistStamped twist;
  92 |         float scaled_v = ref_v_scale_ * v[i];
  93 |         scaled_v = std::clamp(scaled_v, scaled_v_min_, scaled_v_max_);
  94 |         twist.twist.linear.x = v[i];
  95 |         twist.twist.linear.y = 0.0;
  96 |         twist.twist.linear.z = 0.0;
  97 |         reference_waypoint.twists.push_back(twist);
  98 | 
  99 |         // add line
 100 |         int skip_num = 3;
 101 |         if (i > 0 && i % skip_num == 1) {  // skip the first point && i % skip_num == 1)
 102 | 
 103 |             // decide marker color
 104 |             std_msgs::ColorRGBA current_marker_color;
 105 | 
 106 |             // simple normalization of the current velocity
 107 |             double normalized_v = (v[i] - ref_v_min) / std::max((ref_v_max - ref_v_min), 0.1);  // 0 <= normalized_v <= 1
 108 |             tinycolormap::Color colormap_val = tinycolormap::GetColor(normalized_v, tinycolormap::ColormapType::Heat);
 109 | 
 110 |             current_marker_color.r = colormap_val.r();
 111 |             current_marker_color.g = colormap_val.g();
 112 |             current_marker_color.b = colormap_val.b();
 113 |             current_marker_color.a = 1.0;
 114 | 
 115 |             // set 2 positions to draw line
 116 |             visualization_msgs::Marker current_marker = marker_template;
 117 |             current_marker.id = marker_id;  // set an unique id
 118 |             current_marker.points.push_back(prev_marker_point);
 119 |             current_marker.colors.push_back(current_marker_color);
 120 |             geometry_msgs::Point current_marker_point;
 121 |             current_marker_point.x = x[i];
 122 |             current_marker_point.y = y[i];
 123 |             current_marker_point.z = 0.0;
 124 |             prev_marker_point = current_marker_point;
 125 |             current_marker.points.push_back(current_marker_point);
 126 |             current_marker.colors.push_back(current_marker_color);
 127 | 
 128 |             // add current marker to markers
 129 |             polygon_markers.markers.push_back(current_marker);
 130 |             marker_id++;
 131 |         }
 132 |     }
 133 | 
 134 |     // publish waypoint msg
 135 |     reference_waypoint_pub_ = nh_.advertise<waypoint_msgs::Waypoint>(topic_name_waypoint_, 1, true);
 136 |     reference_waypoint_pub_.publish(reference_waypoint);
 137 | 
 138 |     // publish path msg
 139 |     reference_path_pub_ = nh_.advertise<nav_msgs::Path>(topic_name_path_, 1, true);
 140 |     reference_path_pub_.publish(reference_path);
 141 | 
 142 |     // publish visualization msg
 143 |     reference_rvizmarker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_rviz_waypoint_marker_, 1, true);
 144 |     reference_rvizmarker_pub_.publish(polygon_markers);
 145 | }
 146 | 
 147 | }  // namespace planning

```

`src\reference_waypoint_loader\src\reference_waypoint_loader_node.cpp`:

```cpp
   1 | #include "reference_waypoint_loader/reference_waypoint_loader.hpp"
   2 | 
   3 | int main(int argc, char** argv) {
   4 |     ros::init(argc, argv, "reference_waypoint_loader");
   5 |     planning::ReferenceWaypointLoader reference_waypoint_loader;
   6 |     ros::spin();
   7 |     return 0;
   8 | }

```

`src\simulator\CMakeLists.txt`:

```txt
   1 | cmake_minimum_required(VERSION 2.8.3)
   2 | project(f1tenth_gym_ros)
   3 | 
   4 | ## Compile as C++11, supported in ROS Kinetic and newer
   5 | # add_compile_options(-std=c++11)
   6 | 
   7 | ## Find catkin macros and libraries
   8 | ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
   9 | ## is used, also find other catkin packages
  10 | find_package(catkin REQUIRED COMPONENTS
  11 |   message_filters
  12 |   nav_msgs
  13 |   roscpp
  14 |   rospy
  15 |   sensor_msgs
  16 |   std_msgs
  17 |   visualization_msgs
  18 |   message_generation
  19 | )
  20 | 
  21 | ## System dependencies are found with CMake's conventions
  22 | # find_package(Boost REQUIRED COMPONENTS system)
  23 | 
  24 | 
  25 | ## Uncomment this if the package has a setup.py. This macro ensures
  26 | ## modules and global scripts declared therein get installed
  27 | ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
  28 | # catkin_python_setup()
  29 | 
  30 | ################################################
  31 | ## Declare ROS messages, services and actions ##
  32 | ################################################
  33 | 
  34 | ## To declare and build messages, services or actions from within this
  35 | ## package, follow these steps:
  36 | ## * Let MSG_DEP_SET be the set of packages whose message types you use in
  37 | ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
  38 | ## * In the file package.xml:
  39 | ##   * add a build_depend tag for "message_generation"
  40 | ##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
  41 | ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
  42 | ##     but can be declared for certainty nonetheless:
  43 | ##     * add a exec_depend tag for "message_runtime"
  44 | ## * In this file (CMakeLists.txt):
  45 | ##   * add "message_generation" and every package in MSG_DEP_SET to
  46 | ##     find_package(catkin REQUIRED COMPONENTS ...)
  47 | ##   * add "message_runtime" and every package in MSG_DEP_SET to
  48 | ##     catkin_package(CATKIN_DEPENDS ...)
  49 | ##   * uncomment the add_*_files sections below as needed
  50 | ##     and list every .msg/.srv/.action file to be processed
  51 | ##   * uncomment the generate_messages entry below
  52 | ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
  53 | 
  54 | ## Generate messages in the 'msg' folder
  55 | add_message_files(
  56 |   FILES
  57 |   RaceInfo.msg
  58 | )
  59 | 
  60 | ## Generate services in the 'srv' folder
  61 | # add_service_files(
  62 | #   FILES
  63 | #   Service1.srv
  64 | #   Service2.srv
  65 | # )
  66 | 
  67 | ## Generate actions in the 'action' folder
  68 | # add_action_files(
  69 | #   FILES
  70 | #   Action1.action
  71 | #   Action2.action
  72 | # )
  73 | 
  74 | ## Generate added messages and services with any dependencies listed here
  75 | generate_messages(
  76 |   DEPENDENCIES
  77 |   std_msgs
  78 | )
  79 | 
  80 | ################################################
  81 | ## Declare ROS dynamic reconfigure parameters ##
  82 | ################################################
  83 | 
  84 | ## To declare and build dynamic reconfigure parameters within this
  85 | ## package, follow these steps:
  86 | ## * In the file package.xml:
  87 | ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
  88 | ## * In this file (CMakeLists.txt):
  89 | ##   * add "dynamic_reconfigure" to
  90 | ##     find_package(catkin REQUIRED COMPONENTS ...)
  91 | ##   * uncomment the "generate_dynamic_reconfigure_options" section below
  92 | ##     and list every .cfg file to be processed
  93 | 
  94 | ## Generate dynamic reconfigure parameters in the 'cfg' folder
  95 | # generate_dynamic_reconfigure_options(
  96 | #   cfg/DynReconf1.cfg
  97 | #   cfg/DynReconf2.cfg
  98 | # )
  99 | 
 100 | ###################################
 101 | ## catkin specific configuration ##
 102 | ###################################
 103 | ## The catkin_package macro generates cmake config files for your package
 104 | ## Declare things to be passed to dependent projects
 105 | ## INCLUDE_DIRS: uncomment this if your package contains header files
 106 | ## LIBRARIES: libraries you create in this project that dependent projects also need
 107 | ## CATKIN_DEPENDS: catkin_packages dependent projects also need
 108 | ## DEPENDS: system dependencies of this project that dependent projects also need
 109 | catkin_package(
 110 | #  INCLUDE_DIRS include
 111 | #  LIBRARIES f1tenth_gym_ros
 112 | #  CATKIN_DEPENDS message_filters nav_msgs roscpp rospy sensor_msgs std_msgs visualization_msgs
 113 | #  DEPENDS system_lib
 114 |   CATKIN_DEPENDS message_runtime
 115 | )
 116 | 
 117 | ###########
 118 | ## Build ##
 119 | ###########
 120 | 
 121 | ## Specify additional locations of header files
 122 | ## Your package locations should be listed before other locations
 123 | include_directories(
 124 | # include
 125 |   ${catkin_INCLUDE_DIRS}
 126 | )
 127 | 
 128 | ## Declare a C++ library
 129 | # add_library(${PROJECT_NAME}
 130 | #   src/${PROJECT_NAME}/f1tenth_gym_ros.cpp
 131 | # )
 132 | 
 133 | ## Add cmake target dependencies of the library
 134 | ## as an example, code may need to be generated before libraries
 135 | ## either from message generation or dynamic reconfigure
 136 | # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 137 | 
 138 | ## Declare a C++ executable
 139 | ## With catkin_make all packages are built within a single CMake context
 140 | ## The recommended prefix ensures that target names across packages don't collide
 141 | # add_executable(${PROJECT_NAME}_node src/f1tenth_gym_ros_node.cpp)
 142 | 
 143 | ## Rename C++ executable without prefix
 144 | ## The above recommended prefix causes long target names, the following renames the
 145 | ## target back to the shorter version for ease of user use
 146 | ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
 147 | # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
 148 | 
 149 | ## Add cmake target dependencies of the executable
 150 | ## same as for the library above
 151 | # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 152 | 
 153 | ## Specify libraries to link a library or executable target against
 154 | # target_link_libraries(${PROJECT_NAME}_node
 155 | #   ${catkin_LIBRARIES}
 156 | # )
 157 | 
 158 | #############
 159 | ## Install ##
 160 | #############
 161 | 
 162 | # all install targets should use catkin DESTINATION variables
 163 | # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
 164 | 
 165 | ## Mark executable scripts (Python etc.) for installation
 166 | ## in contrast to setup.py, you can choose the destination
 167 | # install(PROGRAMS
 168 | #   scripts/my_python_script
 169 | #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 170 | # )
 171 | 
 172 | ## Mark executables for installation
 173 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 174 | # install(TARGETS ${PROJECT_NAME}_node
 175 | #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 176 | # )
 177 | 
 178 | ## Mark libraries for installation
 179 | ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 180 | # install(TARGETS ${PROJECT_NAME}
 181 | #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 182 | #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 183 | #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 184 | # )
 185 | 
 186 | ## Mark cpp header files for installation
 187 | # install(DIRECTORY include/${PROJECT_NAME}/
 188 | #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 189 | #   FILES_MATCHING PATTERN "*.h"
 190 | #   PATTERN ".svn" EXCLUDE
 191 | # )
 192 | 
 193 | ## Mark other files for installation (e.g. launch and bag files, etc.)
 194 | # install(FILES
 195 | #   # myfile1
 196 | #   # myfile2
 197 | #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 198 | # )
 199 | 
 200 | #############
 201 | ## Testing ##
 202 | #############
 203 | 
 204 | ## Add gtest based cpp test target and link libraries
 205 | # catkin_add_gtest(${PROJECT_NAME}-test test/test_f1tenth_gym_ros.cpp)
 206 | # if(TARGET ${PROJECT_NAME}-test)
 207 | #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 208 | # endif()
 209 | 
 210 | ## Add folders to be run by python nosetests
 211 | # catkin_add_nosetests(test)

```

`src\simulator\Dockerfile`:

```
   1 | FROM ros:noetic-ros-base-focal
   2 | 
   3 | ENV IM_IN_DOCKER Yes
   4 | 
   5 | RUN apt-get update --fix-missing && \
   6 |     apt-get install -y \
   7 |                     python3-dev \
   8 |                     python3-pip \
   9 |                     python-is-python3 \
  10 |                     git \
  11 |                     build-essential \
  12 |                     libgl1-mesa-dev \
  13 |                     mesa-utils \
  14 |                     libglu1-mesa-dev \
  15 |                     fontconfig \
  16 |                     libfreetype6-dev
  17 | 
  18 | RUN apt-get install -y ros-noetic-ackermann-msgs \
  19 |                        ros-noetic-map-server \
  20 |                        ros-noetic-genpy \
  21 |                        ros-noetic-xacro \
  22 |                        ros-noetic-robot-state-publisher 
  23 | 
  24 | # RUN pip3 install --upgrade pip
  25 | 
  26 | RUN pip3 install PyOpenGL \
  27 |                  PyOpenGL_accelerate
  28 | 
  29 | 
  30 | # RUN git clone https://github.com/f1tenth/f1tenth_gym
  31 | RUN mkdir /f1tenth_gym
  32 | COPY ./f1tenth_gym /f1tenth_gym
  33 | 
  34 | RUN cd f1tenth_gym && \
  35 |     pip3 install -e .
  36 | 
  37 | RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; mkdir -p catkin_ws/src; cd catkin_ws; catkin_make"
  38 | 
  39 | RUN mkdir /catkin_ws/src/f1tenth_gym_ros
  40 | 
  41 | COPY . /catkin_ws/src/f1tenth_gym_ros
  42 | 
  43 | RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"
  44 | 
  45 | # CMD ["/catkin_ws/src/f1tenth_gym_ros/start.sh"]
  46 | ENTRYPOINT [ "/catkin_ws/src/f1tenth_gym_ros/start.sh" ]
  47 | 
  48 | # CMD ["roslaunch", "package file.launch"]

```

`src\simulator\LICENSE`:

```
   1 | MIT License
   2 | 
   3 | Copyright (c) 2020 Hongrui Zheng, Matthew O'Kelly
   4 | 
   5 | Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | of this software and associated documentation files (the "Software"), to deal
   7 | in the Software without restriction, including without limitation the rights
   8 | to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | copies of the Software, and to permit persons to whom the Software is
  10 | furnished to do so, subject to the following conditions:
  11 | 
  12 | The above copyright notice and this permission notice shall be included in all
  13 | copies or substantial portions of the Software.
  14 | 
  15 | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | SOFTWARE.

```

`src\simulator\README.md`:

```md
   1 | # F1TENTH gym environment ROS communication bridge
   2 | This is a containerized ROS communication bridge for F1TENTH gym environment.
   3 | 
   4 | This project is still under heavy developement.
   5 | 
   6 | # Overview
   7 | 
   8 | <img src="f1tenth_gym_ros.png" width="600">
   9 | 
  10 | # Different Benchmarks
  11 | In our virtual race, there will be three benchmark tasks. 
  12 | 
  13 | 1. **Benchmark 1** is a single agent time trial without obstacle on the track. The objective is to achieve lower lap times. 
  14 | 2. **Benchmark 2** is a single agent task with unknown obstacles in the map before hand. The objective is to finish laps without crashing. 
  15 | 3. **Benchmark 3** is a task where two agents compete simultaneously on the same track. The objective is to finish a certain number of laps before the other agent.
  16 | 
  17 | We provide several branches for different benchmarks. On the **master** branch, the simulator is created for Benchmarks 1 & 2, where only a single agent (the ego agent) will spawn in the map. On the **multi_node** branch, the simulator is modified for Benchmark 3, where two agents will spawn in the map. We'll go over how these agents are controlled in a following section.
  18 | 
  19 | # Installation
  20 | <!---Before cloning this repo, you'll need to install Docker. Note that this environment is only tested on Ubuntu. You'll also need ROS on your host system. --->
  21 | 
  22 | **System Requirements:**
  23 | - Ubuntu (tested on 20.04)
  24 | - ROS (tested on Noetic)
  25 | - Docker (Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker.)
  26 | 
  27 | 1. Clone this repo into the ```src/``` directory in your workspace, 
  28 | 2. Build the docker image by:
  29 | ```bash
  30 | $ cd f1tenth_gym_ros
  31 | $ sudo ./build_docker.sh
  32 | ```
  33 | This will take around 5 minutes to build depending on your system
  34 | 
  35 | 3. To run the containerized environment, start a docker container by:
  36 | ```bash
  37 | $ sudo ./docker.sh
  38 | ```
  39 | 4. Next, in a new terminal in the host system, check everything is working by:
  40 | ```bash
  41 | $ rostopic list
  42 | ```
  43 | You should a see a few topics like the usual ```/rosout``` etc. And topics provided by the environment like ```/scan``` etc.
  44 | 
  45 | > **When you're creating your own launch file to launch your node, please include ```gym_bridge_host.launch``` in the ```launch``` directory in your own launch file by putting this line in your launch file:**
  46 | > ```xml
  47 | > <include file="$(find f1tenth_gym_ros)/launch/gym_bridge_host.launch"/>
  48 | > ```
  49 | 
  50 | 5. An example agent launch file is in ```launch/agent_template.launch```. After you build your workspace after ```catkin_make```, you can run the agent template by running:
  51 | ```bash
  52 | $ roslaunch f1tenth_gym_ros agent_template.launch
  53 | ```
  54 | You should see an rviz window show up, showing the map, the two cars (ego is blue and opponent is orange), and the LaserScan of the ego car. The opponent is running pure pursuit around the track, and the ego agent is not moving.
  55 | 
  56 | # Available Topics for subscription
  57 | 
  58 | ```/scan```: The ego agent's laser scan
  59 | 
  60 | ```/odom```: The ego agent's odometry
  61 | 
  62 | ```/opp_odom```: The opponent agent's odometry
  63 | 
  64 | ```/opp_scan```: The opponent agent's laser scan (only available on the multi_node branch)
  65 | 
  66 | ```/map```: The map of the environment
  67 | 
  68 | ```/race_info```: Information of the environment including both agents' elapsed runtimes, both agents' lap count, and both agents' collsion info. **Currently, the race ends after both agents finish two laps, so the elapsed times will stop increasing after both lap counts are > 2**
  69 | 
  70 | # Developing and creating your own agent in ROS
  71 | A basic dummy agent node is provided in ```scripts/dummy_agent_node.py```. Launch your own node in your launch file, and don't forget to include ```gym_bridge_host.launch``` in your own launch file.
  72 | 
  73 | On the **master** branch for single agent simulation, publish your drive message on the ```/drive``` topic using the AckermannDriveStamped message type. The simulation is stepped by a callback function subscribed to the drive topic.
  74 | 
  75 | On the **multi_node** branch for two-agent simulation, publish the ego agent's drive commands to ```/drive```, and the opponent agent's drive commands to ```/opp_drive```. At this point, we're not providing any agents built in for testing. A good way to start test your algorithms in this setting is to use another algorithm that you've created, or even the same algorithm.
  76 | 
  77 | # Changing maps
  78 | After you've ran the ```build_docker.sh``` script, you can copy the corresponding .yaml and image file into two directories: ```f1tenth_gym_ros/maps``` and ```f1tenth_gym_ros/f1tenth_gym/maps```. Then change the ```map_path``` and ```map_img_ext``` parameters in ```f1tenth_gym_ros/params.yaml``` to the corresponding paths. Lastly, change the ```map``` argument in ```f1tenth_gym_ros/launch/gym_bridge.launch``` to the new map.
  79 | 
  80 | After making all the changes, make sure you run ```build_docker.sh``` to rebuild the container.
  81 | 
  82 | You can find a collection of maps including the ones from past competitions here: https://github.com/f1tenth/f1tenth_simulator/tree/master/maps
  83 | 
  84 | # TODO
  85 | - [x] Two-way comm tests
  86 | - [x] RobotModel state update
  87 | - [x] Some way to notify collision between agents
  88 | - [x] Some way to notify two cars finishing fixed number of laps
  89 | - [x] Since we have timer update instead of action stepping, what is the notion of 'done'?
  90 | - [x] Publish more topics on collsions, laptime, and done
  91 | - [x] Integrate example test agents
  92 | - [ ] ~~Integrate competent racing agents (with random order when testing)~~
  93 | - [x] Fix mismatch between ray casted scan and robot model
  94 | - [ ] ~~Add instruction in README for rebuilding image when remote repo updates~~
  95 | - [ ] Handle env physics when collisions happen (agent-agent, agent-env)
  96 | - [ ] ~~Add some parameterization on racing scenarios~~

```

`src\simulator\build_docker.sh`:

```sh
   1 | #!/bin/bash
   2 | if [ ! -d f1tenth_gym ] ; then
   3 |     git clone git@github.com:IV2023ADContest/f1tenth_gym.git
   4 | else
   5 |     echo f1tenth_gym exists, not cloning.
   6 | fi
   7 | docker build -t f1tenth_gym -f Dockerfile .;
   8 | docker image prune --force;

```

`src\simulator\docker.sh`:

```sh
   1 | #!/bin/bash
   2 | 
   3 | NUM_STATIC_OBS=$1
   4 | 
   5 | if [ -z "$NUM_STATIC_OBS" ]
   6 | then
   7 |     NUM_STATIC_OBS=5
   8 | fi
   9 | 
  10 | docker run -it --name=f1tenth_gym_container --rm --net=host f1tenth_gym $NUM_STATIC_OBS
  11 | 

```

`src\simulator\ego_racecar.xacro`:

```xacro
   1 | <?xml version="1.0"?>
   2 | 
   3 | <!-- A simple model of the racecar for rviz -->
   4 | 
   5 | <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="racecar">
   6 | 
   7 |   <xacro:property name="wheelbase" value="0.3302" />
   8 |   <xacro:property name="width" value="0.2032" />
   9 |   <xacro:property name="height" value="0.1" />
  10 |   <xacro:property name="ground_offset" value="0.04" />
  11 |   <xacro:property name="wheel_radius" value="0.0508" />
  12 |   <xacro:property name="wheel_length" value="0.0381" />
  13 |   <xacro:property name="laser_distance_from_base_link" value="0.275" />
  14 |   <xacro:property name="laser_height" value="0.05" />
  15 |   <xacro:property name="laser_radius" value="0.026" />
  16 | 
  17 |   <material name="black">
  18 |     <color rgba="0.2 0.2 0.2 1."/>
  19 |   </material>
  20 | 
  21 |   <material name="blue">
  22 |     <color rgba="0.3 0.57 1. 1."/>
  23 |   </material>
  24 | 
  25 |   <link name="base_link">
  26 |     <visual>
  27 |       <origin xyz="${wheelbase/2} 0 ${ground_offset+height/2}"/>
  28 |       <geometry>
  29 |         <box size="${wheelbase} ${width} ${height}"/>
  30 |       </geometry>
  31 |       <material name="blue"/>
  32 |     </visual>
  33 |   </link>
  34 | 
  35 |   <joint name="base_to_laser_model" type="fixed">
  36 |     <parent link="base_link"/>
  37 |     <child link="laser_model"/>
  38 |     <origin xyz="${laser_distance_from_base_link} 0 ${ground_offset+height+(laser_height/2)}"/>
  39 |   </joint>
  40 | 
  41 |   <link name="laser_model">
  42 |     <visual>
  43 |       <geometry>
  44 |         <cylinder radius="${laser_radius}" length="${laser_height}"/>
  45 |       </geometry>
  46 |       <material name="black"/>
  47 |     </visual>
  48 |   </link>
  49 | 
  50 |   <joint name="base_to_back_left_wheel" type="fixed">
  51 |     <parent link="base_link"/>
  52 |     <child link="back_left_wheel"/>
  53 |     <origin xyz="0 ${(wheel_length+width)/2} ${wheel_radius}"/>
  54 |   </joint>
  55 | 
  56 |   <link name="back_left_wheel">
  57 |     <visual>
  58 |       <geometry>
  59 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  60 |       </geometry>
  61 |       <material name="black"/>
  62 |       <origin rpy="${pi/2} 0 0"/>
  63 |     </visual>
  64 |   </link>
  65 | 
  66 |   <joint name="base_to_back_right_wheel" type="fixed">
  67 |     <parent link="base_link"/>
  68 |     <child link="back_right_wheel"/>
  69 |     <origin xyz="0 ${-(wheel_length+width)/2} ${wheel_radius}"/>
  70 |   </joint>
  71 | 
  72 |   <link name="back_right_wheel">
  73 |     <visual>
  74 |       <geometry>
  75 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  76 |       </geometry>
  77 |       <material name="black"/>
  78 |       <origin rpy="${pi/2} 0 0"/>
  79 |     </visual>
  80 |   </link>
  81 | 
  82 |   <joint name="base_to_front_left_hinge" type="fixed">
  83 |     <parent link="base_link"/>
  84 |     <child link="front_left_hinge"/>
  85 |     <origin xyz="${wheelbase} ${(wheel_length+width)/2} ${wheel_radius}"/>
  86 |   </joint>
  87 | 
  88 |   <link name="front_left_hinge"/>
  89 | 
  90 |   <joint name="front_left_hinge_to_wheel" type="continuous">
  91 |     <parent link="front_left_hinge"/>
  92 |     <child link="front_left_wheel"/>
  93 |   </joint>
  94 | 
  95 |   <link name="front_left_wheel">
  96 |     <visual>
  97 |       <geometry>
  98 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  99 |       </geometry>
 100 |       <material name="black"/>
 101 |       <origin rpy="${pi/2} 0 0"/>
 102 |     </visual>
 103 |   </link>
 104 | 
 105 |   <joint name="base_to_front_right_hinge" type="fixed">
 106 |     <parent link="base_link"/>
 107 |     <child link="front_right_hinge"/>
 108 |     <origin xyz="${wheelbase} ${-(wheel_length+width)/2} ${wheel_radius}"/>
 109 |   </joint>
 110 | 
 111 |   <link name="front_right_hinge"/>
 112 | 
 113 |   <joint name="front_right_hinge_to_wheel" type="continuous">
 114 |     <parent link="front_right_hinge"/>
 115 |     <child link="front_right_wheel"/>
 116 |   </joint>
 117 | 
 118 |   <link name="front_right_wheel">
 119 |     <visual>
 120 |       <geometry>
 121 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
 122 |       </geometry>
 123 |       <material name="black"/>
 124 |       <origin rpy="${pi/2} 0 0"/>
 125 |     </visual>
 126 |   </link>
 127 | 
 128 | </robot>

```

`src\simulator\f1tenth_gym\Dockerfile`:

```
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | FROM ubuntu:20.04
  24 | 
  25 | ARG DEBIAN_FRONTEND="noninteractive"
  26 | ENV LIBGL_ALWAYS_INDIRECT=1
  27 | ENV NVIDIA_VISIBLE_DEVICES \
  28 |     ${NVIDIA_VISIBLE_DEVICES:-all}
  29 | 
  30 | ENV NVIDIA_DRIVER_CAPABILITIES \
  31 |     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
  32 | 
  33 | RUN apt-get update --fix-missing && \
  34 |     apt-get install -y \
  35 |                     python3-dev \
  36 |                     python3-pip \
  37 |                     git \
  38 |                     build-essential \
  39 |                     libgl1-mesa-dev \
  40 |                     mesa-utils \
  41 |                     libglu1-mesa-dev \
  42 |                     fontconfig \
  43 |                     libfreetype6-dev
  44 | 
  45 | RUN pip3 install --upgrade pip
  46 | RUN pip3 install PyOpenGL \
  47 |                  PyOpenGL_accelerate
  48 | 
  49 | RUN mkdir /f1tenth_gym
  50 | COPY . /f1tenth_gym
  51 | 
  52 | RUN cd /f1tenth_gym && \
  53 |     pip3 install -e .
  54 | 
  55 | WORKDIR /f1tenth_gym
  56 | 
  57 | ENTRYPOINT ["/bin/bash"]

```

`src\simulator\f1tenth_gym\LICENSE`:

```
   1 | MIT License
   2 | 
   3 | Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | of this software and associated documentation files (the "Software"), to deal
   7 | in the Software without restriction, including without limitation the rights
   8 | to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | copies of the Software, and to permit persons to whom the Software is
  10 | furnished to do so, subject to the following conditions:
  11 | 
  12 | The above copyright notice and this permission notice shall be included in all
  13 | copies or substantial portions of the Software.
  14 | 
  15 | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | SOFTWARE.

```

`src\simulator\f1tenth_gym\README.md`:

```md
   1 | ![Python 3.8 3.9](https://github.com/f1tenth/f1tenth_gym/actions/workflows/ci.yml/badge.svg)
   2 | ![Docker](https://github.com/f1tenth/f1tenth_gym/actions/workflows/docker.yml/badge.svg)
   3 | # The F1TENTH Gym environment
   4 | 
   5 | This is the repository of the F1TENTH Gym environment.
   6 | 
   7 | This project is still under heavy developement.
   8 | 
   9 | You can find the [documentation](https://f1tenth-gym.readthedocs.io/en/latest/) of the environment here.
  10 | 
  11 | ## Quickstart
  12 | We recommend installing the simulation inside a virtualenv. You can install the environment by running:
  13 | 
  14 | ```bash
  15 | virtualenv gym_env
  16 | source gym_env/bin/activate
  17 | git clone https://github.com/f1tenth/f1tenth_gym.git
  18 | cd f1tenth_gym
  19 | pip install -e .
  20 | ```
  21 | 
  22 | Then you can run a quick waypoint follow example by:
  23 | ```bash
  24 | cd examples
  25 | python3 waypoint_follow.py
  26 | ```
  27 | 
  28 | A Dockerfile is also provided with support for the GUI with nvidia-docker (nvidia GPU required):
  29 | ```bash
  30 | docker build -t f1tenth_gym_container -f Dockerfile .
  31 | docker run --gpus all -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix f1tenth_gym_container
  32 | ````
  33 | Then the same example can be ran.
  34 | 
  35 | ## Known issues
  36 | - Library support issues on Windows. You must use Python 3.8 as of 10-2021
  37 | - On MacOS Big Sur and above, when rendering is turned on, you might encounter the error:
  38 | ```
  39 | ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.
  40 | ```
  41 | You can fix the error by installing a newer version of pyglet:
  42 | ```bash
  43 | $ pip3 install pyglet==1.5.11
  44 | ```
  45 | And you might see an error similar to
  46 | ```
  47 | gym 0.17.3 requires pyglet<=1.5.0,>=1.4.0, but you'll have pyglet 1.5.11 which is incompatible.
  48 | ```
  49 | which could be ignored. The environment should still work without error.
  50 | 
  51 | ## Citing
  52 | If you find this Gym environment useful, please consider citing:
  53 | 
  54 | ```
  55 | @inproceedings{okelly2020f1tenth,
  56 |   title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  57 |   author={O’Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  58 |   booktitle={NeurIPS 2019 Competition and Demonstration Track},
  59 |   pages={77--89},
  60 |   year={2020},
  61 |   organization={PMLR}
  62 | }
  63 | ```

```

`src\simulator\f1tenth_gym\examples\config_example_map.yaml`:

```yaml
   1 | # metadata
   2 | run_name: 'map_wide'
   3 | # characteristic number for map
   4 | perf_num: 6
   5 | 
   6 | # map paths
   7 | map_path: './example_map'
   8 | map_ext: '.png'
   9 | 
  10 | # starting pose for map
  11 | sx: 0.7
  12 | sy: 0.0
  13 | stheta: 1.37079632679
  14 | 
  15 | # raceline path and indices
  16 | wpt_path: './example_waypoints.csv'
  17 | wpt_delim: ';'
  18 | wpt_rowskip: 3
  19 | wpt_xind: 1
  20 | wpt_yind: 2
  21 | wpt_thind: 3
  22 | wpt_vind: 5
  23 | 
  24 | # varied params bound
  25 | # physical params
  26 | mass_min: 3.0
  27 | mass_max: 4.0
  28 | lf_min: 0.147
  29 | lf_max: 0.170
  30 | # controller params
  31 | tlad_min: 0.2
  32 | tlad_max: 5.
  33 | vgain_min: 0.5
  34 | vgain_max: 1.5
  35 | 
  36 | # computation budget (can think of it as gen_num times pop_size)
  37 | popsize: 100
  38 | budget: 1000
  39 | 
  40 | # optimization method
  41 | optim_method: 'CMA'
  42 | 
  43 | # seed
  44 | seed: 12345

```

`src\simulator\f1tenth_gym\examples\example_map.yaml`:

```yaml
   1 | image: example_map.png
   2 | resolution: 0.062500
   3 | origin: [-78.21853769831466,-44.37590462453829, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.45
   6 | free_thresh: 0.196

```

`src\simulator\f1tenth_gym\examples\waypoint_follow.py`:

```py
   1 | import time
   2 | from f110_gym.envs.base_classes import Integrator
   3 | import yaml
   4 | import gym
   5 | import numpy as np
   6 | from argparse import Namespace
   7 | 
   8 | from numba import njit
   9 | 
  10 | from pyglet.gl import GL_POINTS
  11 | 
  12 | """
  13 | Planner Helpers
  14 | """
  15 | 
  16 | 
  17 | @njit(fastmath=False, cache=True)
  18 | def nearest_point_on_trajectory(point, trajectory):
  19 |     """
  20 |     Return the nearest point along the given piecewise linear trajectory.
  21 | 
  22 |     Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
  23 |     not be an issue so long as trajectories are not insanely long.
  24 | 
  25 |         Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)
  26 | 
  27 |     point: size 2 numpy array
  28 |     trajectory: Nx2 matrix of (x,y) trajectory waypoints
  29 |         - these must be unique. If they are not unique, a divide by 0 error will destroy the world
  30 |     """
  31 |     diffs = trajectory[1:, :] - trajectory[:-1, :]
  32 |     l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
  33 |     # this is equivalent to the elementwise dot product
  34 |     # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
  35 |     dots = np.empty((trajectory.shape[0] - 1,))
  36 |     for i in range(dots.shape[0]):
  37 |         dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
  38 |     t = dots / l2s
  39 |     t[t < 0.0] = 0.0
  40 |     t[t > 1.0] = 1.0
  41 |     # t = np.clip(dots / l2s, 0.0, 1.0)
  42 |     projections = trajectory[:-1, :] + (t * diffs.T).T
  43 |     # dists = np.linalg.norm(point - projections, axis=1)
  44 |     dists = np.empty((projections.shape[0],))
  45 |     for i in range(dists.shape[0]):
  46 |         temp = point - projections[i]
  47 |         dists[i] = np.sqrt(np.sum(temp * temp))
  48 |     min_dist_segment = np.argmin(dists)
  49 |     return (
  50 |         projections[min_dist_segment],
  51 |         dists[min_dist_segment],
  52 |         t[min_dist_segment],
  53 |         min_dist_segment,
  54 |     )
  55 | 
  56 | 
  57 | @njit(fastmath=False, cache=True)
  58 | def first_point_on_trajectory_intersecting_circle(
  59 |     point, radius, trajectory, t=0.0, wrap=False
  60 | ):
  61 |     """
  62 |     starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.
  63 | 
  64 |     Assumes that the first segment passes within a single radius of the point
  65 | 
  66 |     http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
  67 |     """
  68 |     start_i = int(t)
  69 |     start_t = t % 1.0
  70 |     first_t = None
  71 |     first_i = None
  72 |     first_p = None
  73 |     trajectory = np.ascontiguousarray(trajectory)
  74 |     for i in range(start_i, trajectory.shape[0] - 1):
  75 |         start = trajectory[i, :]
  76 |         end = trajectory[i + 1, :] + 1e-6
  77 |         V = np.ascontiguousarray(end - start)
  78 | 
  79 |         a = np.dot(V, V)
  80 |         b = 2.0 * np.dot(V, start - point)
  81 |         c = (
  82 |             np.dot(start, start)
  83 |             + np.dot(point, point)
  84 |             - 2.0 * np.dot(start, point)
  85 |             - radius * radius
  86 |         )
  87 |         discriminant = b * b - 4 * a * c
  88 | 
  89 |         if discriminant < 0:
  90 |             continue
  91 |         #   print "NO INTERSECTION"
  92 |         # else:
  93 |         # if discriminant >= 0.0:
  94 |         discriminant = np.sqrt(discriminant)
  95 |         t1 = (-b - discriminant) / (2.0 * a)
  96 |         t2 = (-b + discriminant) / (2.0 * a)
  97 |         if i == start_i:
  98 |             if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
  99 |                 first_t = t1
 100 |                 first_i = i
 101 |                 first_p = start + t1 * V
 102 |                 break
 103 |             if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
 104 |                 first_t = t2
 105 |                 first_i = i
 106 |                 first_p = start + t2 * V
 107 |                 break
 108 |         elif t1 >= 0.0 and t1 <= 1.0:
 109 |             first_t = t1
 110 |             first_i = i
 111 |             first_p = start + t1 * V
 112 |             break
 113 |         elif t2 >= 0.0 and t2 <= 1.0:
 114 |             first_t = t2
 115 |             first_i = i
 116 |             first_p = start + t2 * V
 117 |             break
 118 |     # wrap around to the beginning of the trajectory if no intersection is found1
 119 |     if wrap and first_p is None:
 120 |         for i in range(-1, start_i):
 121 |             start = trajectory[i % trajectory.shape[0], :]
 122 |             end = trajectory[(i + 1) % trajectory.shape[0], :] + 1e-6
 123 |             V = end - start
 124 | 
 125 |             a = np.dot(V, V)
 126 |             b = 2.0 * np.dot(V, start - point)
 127 |             c = (
 128 |                 np.dot(start, start)
 129 |                 + np.dot(point, point)
 130 |                 - 2.0 * np.dot(start, point)
 131 |                 - radius * radius
 132 |             )
 133 |             discriminant = b * b - 4 * a * c
 134 | 
 135 |             if discriminant < 0:
 136 |                 continue
 137 |             discriminant = np.sqrt(discriminant)
 138 |             t1 = (-b - discriminant) / (2.0 * a)
 139 |             t2 = (-b + discriminant) / (2.0 * a)
 140 |             if t1 >= 0.0 and t1 <= 1.0:
 141 |                 first_t = t1
 142 |                 first_i = i
 143 |                 first_p = start + t1 * V
 144 |                 break
 145 |             elif t2 >= 0.0 and t2 <= 1.0:
 146 |                 first_t = t2
 147 |                 first_i = i
 148 |                 first_p = start + t2 * V
 149 |                 break
 150 | 
 151 |     return first_p, first_i, first_t
 152 | 
 153 | 
 154 | @njit(fastmath=False, cache=True)
 155 | def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
 156 |     """
 157 |     Returns actuation
 158 |     """
 159 |     waypoint_y = np.dot(
 160 |         np.array([np.sin(-pose_theta), np.cos(-pose_theta)]),
 161 |         lookahead_point[0:2] - position,
 162 |     )
 163 |     speed = lookahead_point[2]
 164 |     if np.abs(waypoint_y) < 1e-6:
 165 |         return speed, 0.0
 166 |     radius = 1 / (2.0 * waypoint_y / lookahead_distance**2)
 167 |     steering_angle = np.arctan(wheelbase / radius)
 168 |     return speed, steering_angle
 169 | 
 170 | 
 171 | class PurePursuitPlanner:
 172 |     """
 173 |     Example Planner
 174 |     """
 175 | 
 176 |     def __init__(self, conf, wb):
 177 |         self.wheelbase = wb
 178 |         self.conf = conf
 179 |         self.load_waypoints(conf)
 180 |         self.max_reacquire = 20.0
 181 | 
 182 |         self.drawn_waypoints = []
 183 | 
 184 |     def load_waypoints(self, conf):
 185 |         """
 186 |         loads waypoints
 187 |         """
 188 |         self.waypoints = np.loadtxt(
 189 |             conf.wpt_path, delimiter=conf.wpt_delim, skiprows=conf.wpt_rowskip
 190 |         )
 191 | 
 192 |     def render_waypoints(self, e):
 193 |         """
 194 |         update waypoints being drawn by EnvRenderer
 195 |         """
 196 | 
 197 |         # points = self.waypoints
 198 | 
 199 |         points = np.vstack(
 200 |             (
 201 |                 self.waypoints[:, self.conf.wpt_xind],
 202 |                 self.waypoints[:, self.conf.wpt_yind],
 203 |             )
 204 |         ).T
 205 | 
 206 |         scaled_points = 50.0 * points
 207 | 
 208 |         for i in range(points.shape[0]):
 209 |             if len(self.drawn_waypoints) < points.shape[0]:
 210 |                 b = e.batch.add(
 211 |                     1,
 212 |                     GL_POINTS,
 213 |                     None,
 214 |                     ("v3f/stream", [scaled_points[i, 0], scaled_points[i, 1], 0.0]),
 215 |                     ("c3B/stream", [183, 193, 222]),
 216 |                 )
 217 |                 self.drawn_waypoints.append(b)
 218 |             else:
 219 |                 self.drawn_waypoints[i].vertices = [
 220 |                     scaled_points[i, 0],
 221 |                     scaled_points[i, 1],
 222 |                     0.0,
 223 |                 ]
 224 | 
 225 |     def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
 226 |         """
 227 |         gets the current waypoint to follow
 228 |         """
 229 |         wpts = np.vstack(
 230 |             (
 231 |                 self.waypoints[:, self.conf.wpt_xind],
 232 |                 self.waypoints[:, self.conf.wpt_yind],
 233 |             )
 234 |         ).T
 235 |         nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)
 236 |         if nearest_dist < lookahead_distance:
 237 |             lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(
 238 |                 position, lookahead_distance, wpts, i + t, wrap=True
 239 |             )
 240 |             if i2 == None:
 241 |                 return None
 242 |             current_waypoint = np.empty((3,))
 243 |             # x, y
 244 |             current_waypoint[0:2] = wpts[i2, :]
 245 |             # speed
 246 |             current_waypoint[2] = waypoints[i, self.conf.wpt_vind]
 247 |             return current_waypoint
 248 |         elif nearest_dist < self.max_reacquire:
 249 |             return np.append(wpts[i, :], waypoints[i, self.conf.wpt_vind])
 250 |         else:
 251 |             return None
 252 | 
 253 |     def plan(self, pose_x, pose_y, pose_theta, lookahead_distance, vgain):
 254 |         """
 255 |         gives actuation given observation
 256 |         """
 257 |         position = np.array([pose_x, pose_y])
 258 |         lookahead_point = self._get_current_waypoint(
 259 |             self.waypoints, lookahead_distance, position, pose_theta
 260 |         )
 261 | 
 262 |         if lookahead_point is None:
 263 |             return 4.0, 0.0
 264 | 
 265 |         speed, steering_angle = get_actuation(
 266 |             pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase
 267 |         )
 268 |         speed = vgain * speed
 269 | 
 270 |         return speed, steering_angle
 271 | 
 272 | 
 273 | class FlippyPlanner:
 274 |     """
 275 |     Planner designed to exploit integration methods and dynamics.
 276 |     For testing only. To observe this error, use single track dynamics for all velocities >0.1
 277 |     """
 278 | 
 279 |     def __init__(self, speed=1, flip_every=1, steer=2):
 280 |         self.speed = speed
 281 |         self.flip_every = flip_every
 282 |         self.counter = 0
 283 |         self.steer = steer
 284 | 
 285 |     def render_waypoints(self, *args, **kwargs):
 286 |         pass
 287 | 
 288 |     def plan(self, *args, **kwargs):
 289 |         if self.counter % self.flip_every == 0:
 290 |             self.counter = 0
 291 |             self.steer *= -1
 292 |         return self.speed, self.steer
 293 | 
 294 | 
 295 | def main():
 296 |     """
 297 |     main entry point
 298 |     """
 299 | 
 300 |     work = {
 301 |         "mass": 3.463388126201571,
 302 |         "lf": 0.15597534362552312,
 303 |         "tlad": 0.82461887897713965,
 304 |         "vgain": 1.375,
 305 |     }  # 0.90338203837889}
 306 | 
 307 |     with open("config_example_map.yaml") as file:
 308 |         conf_dict = yaml.load(file, Loader=yaml.FullLoader)
 309 |     conf = Namespace(**conf_dict)
 310 | 
 311 |     planner = PurePursuitPlanner(
 312 |         conf, (0.17145 + 0.15875)
 313 |     )  # FlippyPlanner(speed=0.2, flip_every=1, steer=10)
 314 | 
 315 |     def render_callback(env_renderer):
 316 |         # custom extra drawing function
 317 | 
 318 |         e = env_renderer
 319 | 
 320 |         # update camera to follow car
 321 |         x = e.cars[0].vertices[::2]
 322 |         y = e.cars[0].vertices[1::2]
 323 |         top, bottom, left, right = max(y), min(y), min(x), max(x)
 324 |         e.score_label.x = left
 325 |         e.score_label.y = top - 700
 326 |         e.left = left - 800
 327 |         e.right = right + 800
 328 |         e.top = top + 800
 329 |         e.bottom = bottom - 800
 330 | 
 331 |         planner.render_waypoints(env_renderer)
 332 | 
 333 |     num_obs = 8
 334 |     env = gym.make(
 335 |         "f110_gym:f110-v0",
 336 |         map=conf.map_path,
 337 |         map_ext=conf.map_ext,
 338 |         num_agents=num_obs + 1,
 339 |         timestep=0.01,
 340 |         integrator=Integrator.RK4,
 341 |     )
 342 |     env.add_render_callback(render_callback)
 343 | 
 344 |     print("Starting simulation")
 345 | 
 346 |     ego_init = np.array([[conf.sx, conf.sy, conf.stheta]])
 347 |     other_init = np.array([[1, 1, 0]] * num_obs)
 348 |     for i in range(num_obs):
 349 |         other_init[i, 0] += 1.0 * i
 350 |         other_init[i, 1] += 1.0 * i
 351 | 
 352 |     init_state = np.concatenate((ego_init, other_init), axis=0)
 353 | 
 354 |     obs, step_reward, done, info = env.reset(init_state)
 355 |     env.render()
 356 | 
 357 |     laptime = 0.0
 358 |     start = time.time()
 359 | 
 360 |     while not done:
 361 |         speed, steer = planner.plan(
 362 |             obs["poses_x"][0],
 363 |             obs["poses_y"][0],
 364 |             obs["poses_theta"][0],
 365 |             work["tlad"],
 366 |             work["vgain"],
 367 |         )
 368 |         ego_control = np.array([[steer, speed]])
 369 |         others_control = np.array([[0, 0]] * num_obs)
 370 |         control_array = np.concatenate((ego_control, others_control), axis=0)
 371 |         obs, step_reward, done, info = env.step(control_array)
 372 |         laptime += step_reward
 373 |         env.render(mode="human")
 374 | 
 375 |     print("Sim elapsed time:", laptime, "Real elapsed time:", time.time() - start)
 376 | 
 377 | 
 378 | if __name__ == "__main__":
 379 |     main()

```

`src\simulator\f1tenth_gym\gym\f110_gym\__init__.py`:

```py
   1 | from gym.envs.registration import register
   2 | 
   3 | register(id="f110-v0", entry_point="f110_gym.envs:F110Env")

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\__init__.py`:

```py
   1 | from f110_gym.envs.f110_env import F110Env
   2 | from f110_gym.envs.dynamic_models import *
   3 | from f110_gym.envs.laser_models import *
   4 | from f110_gym.envs.base_classes import *
   5 | from f110_gym.envs.collision_models import *

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\base_classes.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Prototype of base classes
  26 | Replacement of the old RaceCar, Simulator classes in C++
  27 | Author: Hongrui Zheng
  28 | """
  29 | from enum import Enum
  30 | import warnings
  31 | 
  32 | import numpy as np
  33 | from numba import njit
  34 | 
  35 | from f110_gym.envs.dynamic_models import vehicle_dynamics_st, pid
  36 | from f110_gym.envs.laser_models import ScanSimulator2D, check_ttc_jit, ray_cast
  37 | from f110_gym.envs.collision_models import get_vertices, collision_multiple
  38 | 
  39 | 
  40 | class Integrator(Enum):
  41 |     RK4 = 1
  42 |     Euler = 2
  43 | 
  44 | 
  45 | class RaceCar(object):
  46 |     """
  47 |     Base level race car class, handles the physics and laser scan of a single vehicle
  48 | 
  49 |     Data Members:
  50 |         params (dict): vehicle parameters dictionary
  51 |         is_ego (bool): ego identifier
  52 |         time_step (float): physics timestep
  53 |         num_beams (int): number of beams in laser
  54 |         fov (float): field of view of laser
  55 |         state (np.ndarray (7, )): state vector [x, y, theta, vel, steer_angle, ang_vel, slip_angle]
  56 |         odom (np.ndarray(13, )): odometry vector [x, y, z, qx, qy, qz, qw, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
  57 |         accel (float): current acceleration input
  58 |         steer_angle_vel (float): current steering velocity input
  59 |         in_collision (bool): collision indicator
  60 | 
  61 |     """
  62 | 
  63 |     # static objects that don't need to be stored in class instances
  64 |     scan_simulator = None
  65 |     cosines = None
  66 |     scan_angles = None
  67 |     side_distances = None
  68 | 
  69 |     def __init__(
  70 |         self,
  71 |         params,
  72 |         seed,
  73 |         is_ego=False,
  74 |         time_step=0.01,
  75 |         num_beams=1080,
  76 |         fov=4.7,
  77 |         integrator=Integrator.Euler,
  78 |     ):
  79 |         """
  80 |         Init function
  81 | 
  82 |         Args:
  83 |             params (dict): vehicle parameter dictionary, includes {'mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I', 's_min', 's_max', 'sv_min', 'sv_max', 'v_switch', 'a_max': 9.51, 'v_min', 'v_max', 'length', 'width'}
  84 |             is_ego (bool, default=False): ego identifier
  85 |             time_step (float, default=0.01): physics sim time step
  86 |             num_beams (int, default=1080): number of beams in the laser scan
  87 |             fov (float, default=4.7): field of view of the laser
  88 | 
  89 |         Returns:
  90 |             None
  91 |         """
  92 | 
  93 |         # initialization
  94 |         self.params = params
  95 |         self.seed = seed
  96 |         self.is_ego = is_ego
  97 |         self.time_step = time_step
  98 |         self.num_beams = num_beams
  99 |         self.fov = fov
 100 |         self.integrator = integrator
 101 |         if self.integrator is Integrator.RK4:
 102 |             warnings.warn(
 103 |                 f"Chosen integrator is RK4. This is different from previous versions of the gym."
 104 |             )
 105 | 
 106 |         # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
 107 |         self.state = np.zeros((7,))
 108 | 
 109 |         # pose of opponents in the world
 110 |         self.opp_poses = None
 111 | 
 112 |         # control inputs
 113 |         self.accel = 0.0
 114 |         self.steer_angle_vel = 0.0
 115 | 
 116 |         # steering delay buffer
 117 |         self.steer_buffer = np.empty((0,))
 118 |         self.steer_buffer_size = 2
 119 | 
 120 |         # collision identifier
 121 |         self.in_collision = False
 122 | 
 123 |         # collision threshold for iTTC to environment
 124 |         self.ttc_thresh = 0.005
 125 | 
 126 |         # initialize scan sim
 127 |         if RaceCar.scan_simulator is None:
 128 |             self.scan_rng = np.random.default_rng(seed=self.seed)
 129 |             RaceCar.scan_simulator = ScanSimulator2D(num_beams, fov)
 130 | 
 131 |             scan_ang_incr = RaceCar.scan_simulator.get_increment()
 132 | 
 133 |             # angles of each scan beam, distance from lidar to edge of car at each beam, and precomputed cosines of each angle
 134 |             RaceCar.cosines = np.zeros((num_beams,))
 135 |             RaceCar.scan_angles = np.zeros((num_beams,))
 136 |             RaceCar.side_distances = np.zeros((num_beams,))
 137 | 
 138 |             dist_sides = params["width"] / 2.0
 139 |             dist_fr = (params["lf"] + params["lr"]) / 2.0
 140 | 
 141 |             for i in range(num_beams):
 142 |                 angle = -fov / 2.0 + i * scan_ang_incr
 143 |                 RaceCar.scan_angles[i] = angle
 144 |                 RaceCar.cosines[i] = np.cos(angle)
 145 | 
 146 |                 if angle > 0:
 147 |                     if angle < np.pi / 2:
 148 |                         # between 0 and pi/2
 149 |                         to_side = dist_sides / np.sin(angle)
 150 |                         to_fr = dist_fr / np.cos(angle)
 151 |                         RaceCar.side_distances[i] = min(to_side, to_fr)
 152 |                     else:
 153 |                         # between pi/2 and pi
 154 |                         to_side = dist_sides / np.cos(angle - np.pi / 2.0)
 155 |                         to_fr = dist_fr / np.sin(angle - np.pi / 2.0)
 156 |                         RaceCar.side_distances[i] = min(to_side, to_fr)
 157 |                 else:
 158 |                     if angle > -np.pi / 2:
 159 |                         # between 0 and -pi/2
 160 |                         to_side = dist_sides / np.sin(-angle)
 161 |                         to_fr = dist_fr / np.cos(-angle)
 162 |                         RaceCar.side_distances[i] = min(to_side, to_fr)
 163 |                     else:
 164 |                         # between -pi/2 and -pi
 165 |                         to_side = dist_sides / np.cos(-angle - np.pi / 2)
 166 |                         to_fr = dist_fr / np.sin(-angle - np.pi / 2)
 167 |                         RaceCar.side_distances[i] = min(to_side, to_fr)
 168 | 
 169 |     def update_params(self, params):
 170 |         """
 171 |         Updates the physical parameters of the vehicle
 172 |         Note that does not need to be called at initialization of class anymore
 173 | 
 174 |         Args:
 175 |             params (dict): new parameters for the vehicle
 176 | 
 177 |         Returns:
 178 |             None
 179 |         """
 180 |         self.params = params
 181 | 
 182 |     def set_map(self, map_path, map_ext):
 183 |         """
 184 |         Sets the map for scan simulator
 185 |         
 186 |         Args:
 187 |             map_path (str): absolute path to the map yaml file
 188 |             map_ext (str): extension of the map image file
 189 |         """
 190 |         RaceCar.scan_simulator.set_map(map_path, map_ext)
 191 | 
 192 |     def reset(self, pose):
 193 |         """
 194 |         Resets the vehicle to a pose
 195 |         
 196 |         Args:
 197 |             pose (np.ndarray (3, )): pose to reset the vehicle to
 198 | 
 199 |         Returns:
 200 |             None
 201 |         """
 202 |         # clear control inputs
 203 |         self.accel = 0.0
 204 |         self.steer_angle_vel = 0.0
 205 |         # clear collision indicator
 206 |         self.in_collision = False
 207 |         # clear state
 208 |         self.state = np.zeros((7,))
 209 |         self.state[0:2] = pose[0:2]
 210 |         self.state[4] = pose[2]
 211 |         self.steer_buffer = np.empty((0,))
 212 |         # reset scan random generator
 213 |         self.scan_rng = np.random.default_rng(seed=self.seed)
 214 | 
 215 |     def ray_cast_agents(self, scan):
 216 |         """
 217 |         Ray cast onto other agents in the env, modify original scan
 218 | 
 219 |         Args:
 220 |             scan (np.ndarray, (n, )): original scan range array
 221 | 
 222 |         Returns:
 223 |             new_scan (np.ndarray, (n, )): modified scan
 224 |         """
 225 | 
 226 |         # starting from original scan
 227 |         new_scan = scan
 228 | 
 229 |         # loop over all opponent vehicle poses
 230 |         for opp_pose in self.opp_poses:
 231 |             # get vertices of current oppoenent
 232 |             opp_vertices = get_vertices(
 233 |                 opp_pose, self.params["length"], self.params["width"]
 234 |             )
 235 | 
 236 |             new_scan = ray_cast(
 237 |                 np.append(self.state[0:2], self.state[4]),
 238 |                 new_scan,
 239 |                 self.scan_angles,
 240 |                 opp_vertices,
 241 |             )
 242 | 
 243 |         return new_scan
 244 | 
 245 |     def check_ttc(self, current_scan):
 246 |         """
 247 |         Check iTTC against the environment, sets vehicle states accordingly if collision occurs.
 248 |         Note that this does NOT check collision with other agents.
 249 | 
 250 |         state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
 251 | 
 252 |         Args:
 253 |             current_scan
 254 | 
 255 |         Returns:
 256 |             None
 257 |         """
 258 | 
 259 |         in_collision = check_ttc_jit(
 260 |             current_scan,
 261 |             self.state[3],
 262 |             self.scan_angles,
 263 |             self.cosines,
 264 |             self.side_distances,
 265 |             self.ttc_thresh,
 266 |         )
 267 | 
 268 |         # if in collision stop vehicle
 269 |         if in_collision:
 270 |             self.state[3:] = 0.0
 271 |             self.accel = 0.0
 272 |             self.steer_angle_vel = 0.0
 273 | 
 274 |         # update state
 275 |         self.in_collision = in_collision
 276 | 
 277 |         return in_collision
 278 | 
 279 |     def update_pose(self, raw_steer, vel):
 280 |         """
 281 |         Steps the vehicle's physical simulation
 282 | 
 283 |         Args:
 284 |             steer (float): desired steering angle
 285 |             vel (float): desired longitudinal velocity
 286 | 
 287 |         Returns:
 288 |             current_scan
 289 |         """
 290 | 
 291 |         # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
 292 | 
 293 |         # steering delay
 294 |         steer = 0.0
 295 |         if self.steer_buffer.shape[0] < self.steer_buffer_size:
 296 |             steer = 0.0
 297 |             self.steer_buffer = np.append(raw_steer, self.steer_buffer)
 298 |         else:
 299 |             steer = self.steer_buffer[-1]
 300 |             self.steer_buffer = self.steer_buffer[:-1]
 301 |             self.steer_buffer = np.append(raw_steer, self.steer_buffer)
 302 | 
 303 |         # steering angle velocity input to steering velocity acceleration input
 304 |         accl, sv = pid(
 305 |             vel,
 306 |             steer,
 307 |             self.state[3],
 308 |             self.state[2],
 309 |             self.params["sv_max"],
 310 |             self.params["a_max"],
 311 |             self.params["v_max"],
 312 |             self.params["v_min"],
 313 |         )
 314 | 
 315 |         if self.integrator is Integrator.RK4:
 316 |             # RK4 integration
 317 |             k1 = vehicle_dynamics_st(
 318 |                 self.state,
 319 |                 np.array([sv, accl]),
 320 |                 self.params["mu"],
 321 |                 self.params["C_Sf"],
 322 |                 self.params["C_Sr"],
 323 |                 self.params["lf"],
 324 |                 self.params["lr"],
 325 |                 self.params["h"],
 326 |                 self.params["m"],
 327 |                 self.params["I"],
 328 |                 self.params["s_min"],
 329 |                 self.params["s_max"],
 330 |                 self.params["sv_min"],
 331 |                 self.params["sv_max"],
 332 |                 self.params["v_switch"],
 333 |                 self.params["a_max"],
 334 |                 self.params["v_min"],
 335 |                 self.params["v_max"],
 336 |             )
 337 | 
 338 |             k2_state = self.state + self.time_step * (k1 / 2)
 339 | 
 340 |             k2 = vehicle_dynamics_st(
 341 |                 k2_state,
 342 |                 np.array([sv, accl]),
 343 |                 self.params["mu"],
 344 |                 self.params["C_Sf"],
 345 |                 self.params["C_Sr"],
 346 |                 self.params["lf"],
 347 |                 self.params["lr"],
 348 |                 self.params["h"],
 349 |                 self.params["m"],
 350 |                 self.params["I"],
 351 |                 self.params["s_min"],
 352 |                 self.params["s_max"],
 353 |                 self.params["sv_min"],
 354 |                 self.params["sv_max"],
 355 |                 self.params["v_switch"],
 356 |                 self.params["a_max"],
 357 |                 self.params["v_min"],
 358 |                 self.params["v_max"],
 359 |             )
 360 | 
 361 |             k3_state = self.state + self.time_step * (k2 / 2)
 362 | 
 363 |             k3 = vehicle_dynamics_st(
 364 |                 k3_state,
 365 |                 np.array([sv, accl]),
 366 |                 self.params["mu"],
 367 |                 self.params["C_Sf"],
 368 |                 self.params["C_Sr"],
 369 |                 self.params["lf"],
 370 |                 self.params["lr"],
 371 |                 self.params["h"],
 372 |                 self.params["m"],
 373 |                 self.params["I"],
 374 |                 self.params["s_min"],
 375 |                 self.params["s_max"],
 376 |                 self.params["sv_min"],
 377 |                 self.params["sv_max"],
 378 |                 self.params["v_switch"],
 379 |                 self.params["a_max"],
 380 |                 self.params["v_min"],
 381 |                 self.params["v_max"],
 382 |             )
 383 | 
 384 |             k4_state = self.state + self.time_step * k3
 385 | 
 386 |             k4 = vehicle_dynamics_st(
 387 |                 k4_state,
 388 |                 np.array([sv, accl]),
 389 |                 self.params["mu"],
 390 |                 self.params["C_Sf"],
 391 |                 self.params["C_Sr"],
 392 |                 self.params["lf"],
 393 |                 self.params["lr"],
 394 |                 self.params["h"],
 395 |                 self.params["m"],
 396 |                 self.params["I"],
 397 |                 self.params["s_min"],
 398 |                 self.params["s_max"],
 399 |                 self.params["sv_min"],
 400 |                 self.params["sv_max"],
 401 |                 self.params["v_switch"],
 402 |                 self.params["a_max"],
 403 |                 self.params["v_min"],
 404 |                 self.params["v_max"],
 405 |             )
 406 | 
 407 |             # dynamics integration
 408 |             self.state = self.state + self.time_step * (1 / 6) * (
 409 |                 k1 + 2 * k2 + 2 * k3 + k4
 410 |             )
 411 | 
 412 |         elif self.integrator is Integrator.Euler:
 413 |             f = vehicle_dynamics_st(
 414 |                 self.state,
 415 |                 np.array([sv, accl]),
 416 |                 self.params["mu"],
 417 |                 self.params["C_Sf"],
 418 |                 self.params["C_Sr"],
 419 |                 self.params["lf"],
 420 |                 self.params["lr"],
 421 |                 self.params["h"],
 422 |                 self.params["m"],
 423 |                 self.params["I"],
 424 |                 self.params["s_min"],
 425 |                 self.params["s_max"],
 426 |                 self.params["sv_min"],
 427 |                 self.params["sv_max"],
 428 |                 self.params["v_switch"],
 429 |                 self.params["a_max"],
 430 |                 self.params["v_min"],
 431 |                 self.params["v_max"],
 432 |             )
 433 |             self.state = self.state + self.time_step * f
 434 | 
 435 |         else:
 436 |             raise SyntaxError(
 437 |                 f"Invalid Integrator Specified. Provided {self.integrator.name}. Please choose RK4 or Euler"
 438 |             )
 439 | 
 440 |         # bound yaw angle
 441 |         if self.state[4] > 2 * np.pi:
 442 |             self.state[4] = self.state[4] - 2 * np.pi
 443 |         elif self.state[4] < 0:
 444 |             self.state[4] = self.state[4] + 2 * np.pi
 445 | 
 446 |         # update scan
 447 |         current_scan = RaceCar.scan_simulator.scan(
 448 |             np.append(self.state[0:2], self.state[4]), self.scan_rng
 449 |         )
 450 | 
 451 |         return current_scan
 452 | 
 453 |     def update_opp_poses(self, opp_poses):
 454 |         """
 455 |         Updates the vehicle's information on other vehicles
 456 | 
 457 |         Args:
 458 |             opp_poses (np.ndarray(num_other_agents, 3)): updated poses of other agents
 459 | 
 460 |         Returns:
 461 |             None
 462 |         """
 463 |         self.opp_poses = opp_poses
 464 | 
 465 |     def update_scan(self, agent_scans, agent_index):
 466 |         """
 467 |         Steps the vehicle's laser scan simulation
 468 |         Separated from update_pose because needs to update scan based on NEW poses of agents in the environment
 469 | 
 470 |         Args:
 471 |             agent scans list (modified in-place),
 472 |             agent index (int)
 473 | 
 474 |         Returns:
 475 |             None
 476 |         """
 477 | 
 478 |         current_scan = agent_scans[agent_index]
 479 | 
 480 |         # check ttc
 481 |         self.check_ttc(current_scan)
 482 | 
 483 |         # ray cast other agents to modify scan
 484 |         new_scan = self.ray_cast_agents(current_scan)
 485 | 
 486 |         agent_scans[agent_index] = new_scan
 487 | 
 488 | 
 489 | class Simulator(object):
 490 |     """
 491 |     Simulator class, handles the interaction and update of all vehicles in the environment
 492 | 
 493 |     Data Members:
 494 |         num_agents (int): number of agents in the environment
 495 |         time_step (float): physics time step
 496 |         agent_poses (np.ndarray(num_agents, 3)): all poses of all agents
 497 |         agents (list[RaceCar]): container for RaceCar objects
 498 |         collisions (np.ndarray(num_agents, )): array of collision indicator for each agent
 499 |         collision_idx (np.ndarray(num_agents, )): which agent is each agent in collision with
 500 | 
 501 |     """
 502 | 
 503 |     def __init__(
 504 |         self,
 505 |         params,
 506 |         num_agents,
 507 |         seed,
 508 |         time_step=0.01,
 509 |         ego_idx=0,
 510 |         integrator=Integrator.RK4,
 511 |     ):
 512 |         """
 513 |         Init function
 514 | 
 515 |         Args:
 516 |             params (dict): vehicle parameter dictionary, includes {'mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I', 's_min', 's_max', 'sv_min', 'sv_max', 'v_switch', 'a_max', 'v_min', 'v_max', 'length', 'width'}
 517 |             num_agents (int): number of agents in the environment
 518 |             seed (int): seed of the rng in scan simulation
 519 |             time_step (float, default=0.01): physics time step
 520 |             ego_idx (int, default=0): ego vehicle's index in list of agents
 521 | 
 522 |         Returns:
 523 |             None
 524 |         """
 525 |         self.num_agents = num_agents
 526 |         self.seed = seed
 527 |         self.time_step = time_step
 528 |         self.ego_idx = ego_idx
 529 |         self.params = params
 530 |         self.agent_poses = np.empty((self.num_agents, 3))
 531 |         self.agents = []
 532 |         self.collisions = np.zeros((self.num_agents,))
 533 |         self.collision_idx = -1 * np.ones((self.num_agents,))
 534 | 
 535 |         # initializing agents
 536 |         for i in range(self.num_agents):
 537 |             if i == ego_idx:
 538 |                 ego_car = RaceCar(
 539 |                     params,
 540 |                     self.seed,
 541 |                     is_ego=True,
 542 |                     time_step=self.time_step,
 543 |                     integrator=integrator,
 544 |                 )
 545 |                 self.agents.append(ego_car)
 546 |             else:
 547 |                 agent = RaceCar(
 548 |                     params,
 549 |                     self.seed,
 550 |                     is_ego=False,
 551 |                     time_step=self.time_step,
 552 |                     integrator=integrator,
 553 |                 )
 554 |                 self.agents.append(agent)
 555 | 
 556 |     def set_map(self, map_path, map_ext):
 557 |         """
 558 |         Sets the map of the environment and sets the map for scan simulator of each agent
 559 | 
 560 |         Args:
 561 |             map_path (str): path to the map yaml file
 562 |             map_ext (str): extension for the map image file
 563 | 
 564 |         Returns:
 565 |             None
 566 |         """
 567 |         for agent in self.agents:
 568 |             agent.set_map(map_path, map_ext)
 569 | 
 570 |     def update_params(self, params, agent_idx=-1):
 571 |         """
 572 |         Updates the params of agents, if an index of an agent is given, update only that agent's params
 573 | 
 574 |         Args:
 575 |             params (dict): dictionary of params, see details in docstring of __init__
 576 |             agent_idx (int, default=-1): index for agent that needs param update, if negative, update all agents
 577 | 
 578 |         Returns:
 579 |             None
 580 |         """
 581 |         if agent_idx < 0:
 582 |             # update params for all
 583 |             for agent in self.agents:
 584 |                 agent.update_params(params)
 585 |         elif agent_idx >= 0 and agent_idx < self.num_agents:
 586 |             # only update one agent's params
 587 |             self.agents[agent_idx].update_params(params)
 588 |         else:
 589 |             # index out of bounds, throw error
 590 |             raise IndexError("Index given is out of bounds for list of agents.")
 591 | 
 592 |     def check_collision(self):
 593 |         """
 594 |         Checks for collision between agents using GJK and agents' body vertices
 595 | 
 596 |         Args:
 597 |             None
 598 | 
 599 |         Returns:
 600 |             None
 601 |         """
 602 |         # get vertices of all agents
 603 |         all_vertices = np.empty((self.num_agents, 4, 2))
 604 |         for i in range(self.num_agents):
 605 |             all_vertices[i, :, :] = get_vertices(
 606 |                 np.append(self.agents[i].state[0:2], self.agents[i].state[4]),
 607 |                 self.params["length"],
 608 |                 self.params["width"],
 609 |             )
 610 |         self.collisions, self.collision_idx = collision_multiple(all_vertices)
 611 | 
 612 |     def step(self, control_inputs):
 613 |         """
 614 |         Steps the simulation environment
 615 | 
 616 |         Args:
 617 |             control_inputs (np.ndarray (num_agents, 2)): control inputs of all agents, first column is desired steering angle, second column is desired velocity
 618 |         
 619 |         Returns:
 620 |             observations (dict): dictionary for observations: poses of agents, current laser scan of each agent, collision indicators, etc.
 621 |         """
 622 | 
 623 |         agent_scans = []
 624 | 
 625 |         # looping over agents
 626 |         for i, agent in enumerate(self.agents):
 627 |             # update each agent's pose
 628 |             current_scan = agent.update_pose(control_inputs[i, 0], control_inputs[i, 1])
 629 |             agent_scans.append(current_scan)
 630 | 
 631 |             # update sim's information of agent poses
 632 |             self.agent_poses[i, :] = np.append(agent.state[0:2], agent.state[4])
 633 | 
 634 |         # check collisions between all agents
 635 |         self.check_collision()
 636 | 
 637 |         for i, agent in enumerate(self.agents):
 638 |             # update agent's information on other agents
 639 |             opp_poses = np.concatenate(
 640 |                 (self.agent_poses[0:i, :], self.agent_poses[i + 1 :, :]), axis=0
 641 |             )
 642 |             agent.update_opp_poses(opp_poses)
 643 | 
 644 |             # update each agent's current scan based on other agents
 645 |             agent.update_scan(agent_scans, i)
 646 | 
 647 |             # update agent collision with environment
 648 |             if agent.in_collision:
 649 |                 self.collisions[i] = 1.0
 650 | 
 651 |         # fill in observations
 652 |         # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
 653 |         # collision_angles is removed from observations
 654 |         observations = {
 655 |             "ego_idx": self.ego_idx,
 656 |             "scans": [],
 657 |             "poses_x": [],
 658 |             "poses_y": [],
 659 |             "poses_theta": [],
 660 |             "linear_vels_x": [],
 661 |             "linear_vels_y": [],
 662 |             "ang_vels_z": [],
 663 |             "collisions": self.collisions,
 664 |         }
 665 |         for i, agent in enumerate(self.agents):
 666 |             observations["scans"].append(agent_scans[i])
 667 |             observations["poses_x"].append(agent.state[0])
 668 |             observations["poses_y"].append(agent.state[1])
 669 |             observations["poses_theta"].append(agent.state[4])
 670 |             observations["linear_vels_x"].append(agent.state[3])
 671 |             observations["linear_vels_y"].append(0.0)
 672 |             observations["ang_vels_z"].append(agent.state[5])
 673 | 
 674 |         return observations
 675 | 
 676 |     def reset(self, poses):
 677 |         """
 678 |         Resets the simulation environment by given poses
 679 | 
 680 |         Arges:
 681 |             poses (np.ndarray (num_agents, 3)): poses to reset agents to
 682 | 
 683 |         Returns:
 684 |             None
 685 |         """
 686 | 
 687 |         if poses.shape[0] != self.num_agents:
 688 |             raise ValueError(
 689 |                 "Number of poses for reset does not match number of agents."
 690 |             )
 691 | 
 692 |         # loop over poses to reset
 693 |         for i in range(self.num_agents):
 694 |             self.agents[i].reset(poses[i, :])

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\collision_models.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Prototype of Utility functions and GJK algorithm for Collision checks between vehicles
  26 | Originally from https://github.com/kroitor/gjk.c
  27 | Author: Hongrui Zheng
  28 | """
  29 | 
  30 | import numpy as np
  31 | from numba import njit
  32 | 
  33 | 
  34 | @njit(cache=True)
  35 | def perpendicular(pt):
  36 |     """
  37 |     Return a 2-vector's perpendicular vector
  38 | 
  39 |     Args:
  40 |         pt (np.ndarray, (2,)): input vector
  41 | 
  42 |     Returns:
  43 |         pt (np.ndarray, (2,)): perpendicular vector
  44 |     """
  45 |     temp = pt[0]
  46 |     pt[0] = pt[1]
  47 |     pt[1] = -1 * temp
  48 |     return pt
  49 | 
  50 | 
  51 | @njit(cache=True)
  52 | def tripleProduct(a, b, c):
  53 |     """
  54 |     Return triple product of three vectors
  55 | 
  56 |     Args:
  57 |         a, b, c (np.ndarray, (2,)): input vectors
  58 | 
  59 |     Returns:
  60 |         (np.ndarray, (2,)): triple product
  61 |     """
  62 |     ac = a.dot(c)
  63 |     bc = b.dot(c)
  64 |     return b * ac - a * bc
  65 | 
  66 | 
  67 | @njit(cache=True)
  68 | def avgPoint(vertices):
  69 |     """
  70 |     Return the average point of multiple vertices
  71 | 
  72 |     Args:
  73 |         vertices (np.ndarray, (n, 2)): the vertices we want to find avg on
  74 | 
  75 |     Returns:
  76 |         avg (np.ndarray, (2,)): average point of the vertices
  77 |     """
  78 |     return np.sum(vertices, axis=0) / vertices.shape[0]
  79 | 
  80 | 
  81 | @njit(cache=True)
  82 | def indexOfFurthestPoint(vertices, d):
  83 |     """
  84 |     Return the index of the vertex furthest away along a direction in the list of vertices
  85 | 
  86 |     Args:
  87 |         vertices (np.ndarray, (n, 2)): the vertices we want to find avg on
  88 | 
  89 |     Returns:
  90 |         idx (int): index of the furthest point
  91 |     """
  92 |     return np.argmax(vertices.dot(d))
  93 | 
  94 | 
  95 | @njit(cache=True)
  96 | def support(vertices1, vertices2, d):
  97 |     """
  98 |     Minkowski sum support function for GJK
  99 | 
 100 |     Args:
 101 |         vertices1 (np.ndarray, (n, 2)): vertices of the first body
 102 |         vertices2 (np.ndarray, (n, 2)): vertices of the second body
 103 |         d (np.ndarray, (2, )): direction to find the support along
 104 | 
 105 |     Returns:
 106 |         support (np.ndarray, (n, 2)): Minkowski sum
 107 |     """
 108 |     i = indexOfFurthestPoint(vertices1, d)
 109 |     j = indexOfFurthestPoint(vertices2, -d)
 110 |     return vertices1[i] - vertices2[j]
 111 | 
 112 | 
 113 | @njit(cache=True)
 114 | def collision(vertices1, vertices2):
 115 |     """
 116 |     GJK test to see whether two bodies overlap
 117 | 
 118 |     Args:
 119 |         vertices1 (np.ndarray, (n, 2)): vertices of the first body
 120 |         vertices2 (np.ndarray, (n, 2)): vertices of the second body
 121 | 
 122 |     Returns:
 123 |         overlap (boolean): True if two bodies collide
 124 |     """
 125 |     index = 0
 126 |     simplex = np.empty((3, 2))
 127 | 
 128 |     position1 = avgPoint(vertices1)
 129 |     position2 = avgPoint(vertices2)
 130 | 
 131 |     d = position1 - position2
 132 | 
 133 |     if d[0] == 0 and d[1] == 0:
 134 |         d[0] = 1.0
 135 | 
 136 |     a = support(vertices1, vertices2, d)
 137 |     simplex[index, :] = a
 138 | 
 139 |     if d.dot(a) <= 0:
 140 |         return False
 141 | 
 142 |     d = -a
 143 | 
 144 |     iter_count = 0
 145 |     while iter_count < 1e3:
 146 |         a = support(vertices1, vertices2, d)
 147 |         index += 1
 148 |         simplex[index, :] = a
 149 |         if d.dot(a) <= 0:
 150 |             return False
 151 | 
 152 |         ao = -a
 153 | 
 154 |         if index < 2:
 155 |             b = simplex[0, :]
 156 |             ab = b - a
 157 |             d = tripleProduct(ab, ao, ab)
 158 |             if np.linalg.norm(d) < 1e-10:
 159 |                 d = perpendicular(ab)
 160 |             continue
 161 | 
 162 |         b = simplex[1, :]
 163 |         c = simplex[0, :]
 164 |         ab = b - a
 165 |         ac = c - a
 166 | 
 167 |         acperp = tripleProduct(ab, ac, ac)
 168 | 
 169 |         if acperp.dot(ao) >= 0:
 170 |             d = acperp
 171 |         else:
 172 |             abperp = tripleProduct(ac, ab, ab)
 173 |             if abperp.dot(ao) < 0:
 174 |                 return True
 175 |             simplex[0, :] = simplex[1, :]
 176 |             d = abperp
 177 | 
 178 |         simplex[1, :] = simplex[2, :]
 179 |         index -= 1
 180 | 
 181 |         iter_count += 1
 182 |     return False
 183 | 
 184 | 
 185 | @njit(cache=True)
 186 | def collision_multiple(vertices):
 187 |     """
 188 |     Check pair-wise collisions for all provided vertices
 189 | 
 190 |     Args:
 191 |         vertices (np.ndarray (num_bodies, 4, 2)): all vertices for checking pair-wise collision
 192 | 
 193 |     Returns:
 194 |         collisions (np.ndarray (num_vertices, )): whether each body is in collision
 195 |         collision_idx (np.ndarray (num_vertices, )): which index of other body is each index's body is in collision, -1 if not in collision
 196 |     """
 197 |     collisions = np.zeros((vertices.shape[0],))
 198 |     collision_idx = -1 * np.ones((vertices.shape[0],))
 199 |     # looping over all pairs
 200 |     for i in range(vertices.shape[0] - 1):
 201 |         for j in range(i + 1, vertices.shape[0]):
 202 |             # check collision
 203 |             vi = np.ascontiguousarray(vertices[i, :, :])
 204 |             vj = np.ascontiguousarray(vertices[j, :, :])
 205 |             ij_collision = collision(vi, vj)
 206 |             # fill in results
 207 |             if ij_collision:
 208 |                 collisions[i] = 1.0
 209 |                 collisions[j] = 1.0
 210 |                 collision_idx[i] = j
 211 |                 collision_idx[j] = i
 212 | 
 213 |     return collisions, collision_idx
 214 | 
 215 | 
 216 | """
 217 | Utility functions for getting vertices by pose and shape
 218 | """
 219 | 
 220 | 
 221 | @njit(cache=True)
 222 | def get_trmtx(pose):
 223 |     """
 224 |     Get transformation matrix of vehicle frame -> global frame
 225 | 
 226 |     Args:
 227 |         pose (np.ndarray (3, )): current pose of the vehicle
 228 | 
 229 |     return:
 230 |         H (np.ndarray (4, 4)): transformation matrix
 231 |     """
 232 |     x = pose[0]
 233 |     y = pose[1]
 234 |     th = pose[2]
 235 |     cos = np.cos(th)
 236 |     sin = np.sin(th)
 237 |     H = np.array(
 238 |         [
 239 |             [cos, -sin, 0.0, x],
 240 |             [sin, cos, 0.0, y],
 241 |             [0.0, 0.0, 1.0, 0.0],
 242 |             [0.0, 0.0, 0.0, 1.0],
 243 |         ]
 244 |     )
 245 |     return H
 246 | 
 247 | 
 248 | @njit(cache=True)
 249 | def get_vertices(pose, length, width):
 250 |     """
 251 |     Utility function to return vertices of the car body given pose and size
 252 | 
 253 |     Args:
 254 |         pose (np.ndarray, (3, )): current world coordinate pose of the vehicle
 255 |         length (float): car length
 256 |         width (float): car width
 257 | 
 258 |     Returns:
 259 |         vertices (np.ndarray, (4, 2)): corner vertices of the vehicle body
 260 |     """
 261 |     H = get_trmtx(pose)
 262 |     rl = H.dot(np.asarray([[-length / 2], [width / 2], [0.0], [1.0]])).flatten()
 263 |     rr = H.dot(np.asarray([[-length / 2], [-width / 2], [0.0], [1.0]])).flatten()
 264 |     fl = H.dot(np.asarray([[length / 2], [width / 2], [0.0], [1.0]])).flatten()
 265 |     fr = H.dot(np.asarray([[length / 2], [-width / 2], [0.0], [1.0]])).flatten()
 266 |     rl = rl / rl[3]
 267 |     rr = rr / rr[3]
 268 |     fl = fl / fl[3]
 269 |     fr = fr / fr[3]
 270 |     vertices = np.asarray(
 271 |         [[rl[0], rl[1]], [rr[0], rr[1]], [fr[0], fr[1]], [fl[0], fl[1]]]
 272 |     )
 273 |     return vertices
 274 | 
 275 | 
 276 | """
 277 | Unit tests for GJK collision checks
 278 | Author: Hongrui Zheng
 279 | """
 280 | 
 281 | import time
 282 | import unittest
 283 | 
 284 | 
 285 | class CollisionTests(unittest.TestCase):
 286 |     def setUp(self):
 287 |         # test params
 288 |         np.random.seed(1234)
 289 | 
 290 |         # Collision check body
 291 |         self.vertices1 = np.asarray([[4, 11.0], [5, 5], [9, 9], [10, 10]])
 292 | 
 293 |         # car size
 294 |         self.length = 0.32
 295 |         self.width = 0.22
 296 | 
 297 |     def test_get_vert(self):
 298 |         test_pose = np.array([2.3, 6.7, 0.8])
 299 |         vertices = get_vertices(test_pose, self.length, self.width)
 300 |         rect = np.vstack((vertices, vertices[0, :]))
 301 |         import matplotlib.pyplot as plt
 302 | 
 303 |         plt.scatter(test_pose[0], test_pose[1], c="red")
 304 |         plt.plot(rect[:, 0], rect[:, 1])
 305 |         plt.xlim([1, 4])
 306 |         plt.ylim([5, 8])
 307 |         plt.axes().set_aspect("equal")
 308 |         plt.show()
 309 |         self.assertTrue(vertices.shape == (4, 2))
 310 | 
 311 |     def test_get_vert_fps(self):
 312 |         test_pose = np.array([2.3, 6.7, 0.8])
 313 |         start = time.time()
 314 |         for _ in range(1000):
 315 |             vertices = get_vertices(test_pose, self.length, self.width)
 316 |         elapsed = time.time() - start
 317 |         fps = 1000 / elapsed
 318 |         print("get vertices fps:", fps)
 319 |         self.assertTrue(fps > 500)
 320 | 
 321 |     def test_random_collision(self):
 322 |         # perturb the body by a small amount and make sure it all collides with the original body
 323 |         for _ in range(1000):
 324 |             a = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 325 |             b = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 326 |             self.assertTrue(collision(a, b))
 327 | 
 328 |     def test_multiple_collisions(self):
 329 |         a = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 330 |         b = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 331 |         c = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 332 |         d = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 333 |         e = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 334 |         f = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 335 |         g = self.vertices1 + 10.0
 336 |         allv = np.stack((a, b, c, d, e, f, g))
 337 |         collisions, collision_idx = collision_multiple(allv)
 338 |         self.assertTrue(
 339 |             np.all(collisions == np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]))
 340 |         )
 341 |         self.assertTrue(
 342 |             np.all(collision_idx == np.array([5.0, 5.0, 5.0, 5.0, 5.0, 4.0, -1.0]))
 343 |         )
 344 | 
 345 |     def test_fps(self):
 346 |         # also perturb the body but mainly want to test GJK speed
 347 |         start = time.time()
 348 |         for _ in range(1000):
 349 |             a = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 350 |             b = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 351 |             collision(a, b)
 352 |         elapsed = time.time() - start
 353 |         fps = 1000 / elapsed
 354 |         print("gjk fps:", fps)
 355 |         self.assertTrue(fps > 500)
 356 | 
 357 | 
 358 | if __name__ == "__main__":
 359 |     unittest.main()

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\dynamic_models.py`:

```py
   1 | # Copyright 2020 Technical University of Munich, Professorship of Cyber-Physical Systems, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   2 | 
   3 | # Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
   4 | 
   5 | # 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   6 | 
   7 | # 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   8 | 
   9 | # 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
  10 | 
  11 | # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  12 | 
  13 | 
  14 | """
  15 | Prototype of vehicle dynamics functions and classes for simulating 2D Single
  16 | Track dynamic model
  17 | Following the implementation of commanroad's Single Track Dynamics model
  18 | Original implementation: https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/
  19 | Author: Hongrui Zheng
  20 | """
  21 | 
  22 | import numpy as np
  23 | from numba import njit
  24 | 
  25 | import unittest
  26 | import time
  27 | 
  28 | 
  29 | @njit(cache=True)
  30 | def accl_constraints(vel, accl, v_switch, a_max, v_min, v_max):
  31 |     """
  32 |     Acceleration constraints, adjusts the acceleration based on constraints
  33 | 
  34 |         Args:
  35 |             vel (float): current velocity of the vehicle
  36 |             accl (float): unconstraint desired acceleration
  37 |             v_switch (float): switching velocity (velocity at which the acceleration is no longer able to create wheel spin)
  38 |             a_max (float): maximum allowed acceleration
  39 |             v_min (float): minimum allowed velocity
  40 |             v_max (float): maximum allowed velocity
  41 | 
  42 |         Returns:
  43 |             accl (float): adjusted acceleration
  44 |     """
  45 | 
  46 |     # positive accl limit
  47 |     if vel > v_switch:
  48 |         pos_limit = a_max * v_switch / vel
  49 |     else:
  50 |         pos_limit = a_max
  51 | 
  52 |     # accl limit reached?
  53 |     if (vel <= v_min and accl <= 0) or (vel >= v_max and accl >= 0):
  54 |         accl = 0.0
  55 |     elif accl <= -a_max:
  56 |         accl = -a_max
  57 |     elif accl >= pos_limit:
  58 |         accl = pos_limit
  59 | 
  60 |     return accl
  61 | 
  62 | 
  63 | @njit(cache=True)
  64 | def steering_constraint(
  65 |     steering_angle, steering_velocity, s_min, s_max, sv_min, sv_max
  66 | ):
  67 |     """
  68 |     Steering constraints, adjusts the steering velocity based on constraints
  69 | 
  70 |         Args:
  71 |             steering_angle (float): current steering_angle of the vehicle
  72 |             steering_velocity (float): unconstraint desired steering_velocity
  73 |             s_min (float): minimum steering angle
  74 |             s_max (float): maximum steering angle
  75 |             sv_min (float): minimum steering velocity
  76 |             sv_max (float): maximum steering velocity
  77 | 
  78 |         Returns:
  79 |             steering_velocity (float): adjusted steering velocity
  80 |     """
  81 | 
  82 |     # constraint steering velocity
  83 |     if (steering_angle <= s_min and steering_velocity <= 0) or (
  84 |         steering_angle >= s_max and steering_velocity >= 0
  85 |     ):
  86 |         steering_velocity = 0.0
  87 |     elif steering_velocity <= sv_min:
  88 |         steering_velocity = sv_min
  89 |     elif steering_velocity >= sv_max:
  90 |         steering_velocity = sv_max
  91 | 
  92 |     return steering_velocity
  93 | 
  94 | 
  95 | @njit(cache=True)
  96 | def vehicle_dynamics_ks(
  97 |     x,
  98 |     u_init,
  99 |     mu,
 100 |     C_Sf,
 101 |     C_Sr,
 102 |     lf,
 103 |     lr,
 104 |     h,
 105 |     m,
 106 |     I,
 107 |     s_min,
 108 |     s_max,
 109 |     sv_min,
 110 |     sv_max,
 111 |     v_switch,
 112 |     a_max,
 113 |     v_min,
 114 |     v_max,
 115 | ):
 116 |     """
 117 |     Single Track Kinematic Vehicle Dynamics.
 118 | 
 119 |         Args:
 120 |             x (numpy.ndarray (3, )): vehicle state vector (x1, x2, x3, x4, x5)
 121 |                 x1: x position in global coordinates
 122 |                 x2: y position in global coordinates
 123 |                 x3: steering angle of front wheels
 124 |                 x4: velocity in x direction
 125 |                 x5: yaw angle
 126 |             u (numpy.ndarray (2, )): control input vector (u1, u2)
 127 |                 u1: steering angle velocity of front wheels
 128 |                 u2: longitudinal acceleration
 129 | 
 130 |         Returns:
 131 |             f (numpy.ndarray): right hand side of differential equations
 132 |     """
 133 |     # wheelbase
 134 |     lwb = lf + lr
 135 | 
 136 |     # constraints
 137 |     u = np.array(
 138 |         [
 139 |             steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max),
 140 |             accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max),
 141 |         ]
 142 |     )
 143 | 
 144 |     # system dynamics
 145 |     f = np.array(
 146 |         [
 147 |             x[3] * np.cos(x[4]),
 148 |             x[3] * np.sin(x[4]),
 149 |             u[0],
 150 |             u[1],
 151 |             x[3] / lwb * np.tan(x[2]),
 152 |         ]
 153 |     )
 154 |     return f
 155 | 
 156 | 
 157 | @njit(cache=True)
 158 | def vehicle_dynamics_st(
 159 |     x,
 160 |     u_init,
 161 |     mu,
 162 |     C_Sf,
 163 |     C_Sr,
 164 |     lf,
 165 |     lr,
 166 |     h,
 167 |     m,
 168 |     I,
 169 |     s_min,
 170 |     s_max,
 171 |     sv_min,
 172 |     sv_max,
 173 |     v_switch,
 174 |     a_max,
 175 |     v_min,
 176 |     v_max,
 177 | ):
 178 |     """
 179 |     Single Track Dynamic Vehicle Dynamics.
 180 | 
 181 |         Args:
 182 |             x (numpy.ndarray (3, )): vehicle state vector (x1, x2, x3, x4, x5, x6, x7)
 183 |                 x1: x position in global coordinates
 184 |                 x2: y position in global coordinates
 185 |                 x3: steering angle of front wheels
 186 |                 x4: velocity in x direction
 187 |                 x5: yaw angle
 188 |                 x6: yaw rate
 189 |                 x7: slip angle at vehicle center
 190 |             u (numpy.ndarray (2, )): control input vector (u1, u2)
 191 |                 u1: steering angle velocity of front wheels
 192 |                 u2: longitudinal acceleration
 193 | 
 194 |         Returns:
 195 |             f (numpy.ndarray): right hand side of differential equations
 196 |     """
 197 | 
 198 |     # gravity constant m/s^2
 199 |     g = 9.81
 200 | 
 201 |     # constraints
 202 |     u = np.array(
 203 |         [
 204 |             steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max),
 205 |             accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max),
 206 |         ]
 207 |     )
 208 | 
 209 |     # switch to kinematic model for small velocities
 210 |     if abs(x[3]) < 0.5:
 211 |         # wheelbase
 212 |         lwb = lf + lr
 213 | 
 214 |         # system dynamics
 215 |         x_ks = x[0:5]
 216 |         f_ks = vehicle_dynamics_ks(
 217 |             x_ks,
 218 |             u,
 219 |             mu,
 220 |             C_Sf,
 221 |             C_Sr,
 222 |             lf,
 223 |             lr,
 224 |             h,
 225 |             m,
 226 |             I,
 227 |             s_min,
 228 |             s_max,
 229 |             sv_min,
 230 |             sv_max,
 231 |             v_switch,
 232 |             a_max,
 233 |             v_min,
 234 |             v_max,
 235 |         )
 236 |         f = np.hstack(
 237 |             (
 238 |                 f_ks,
 239 |                 np.array(
 240 |                     [
 241 |                         u[1] / lwb * np.tan(x[2])
 242 |                         + x[3] / (lwb * np.cos(x[2]) ** 2) * u[0],
 243 |                         0,
 244 |                     ]
 245 |                 ),
 246 |             )
 247 |         )
 248 | 
 249 |     else:
 250 |         # system dynamics
 251 |         f = np.array(
 252 |             [
 253 |                 x[3] * np.cos(x[6] + x[4]),
 254 |                 x[3] * np.sin(x[6] + x[4]),
 255 |                 u[0],
 256 |                 u[1],
 257 |                 x[5],
 258 |                 -mu
 259 |                 * m
 260 |                 / (x[3] * I * (lr + lf))
 261 |                 * (
 262 |                     lf ** 2 * C_Sf * (g * lr - u[1] * h)
 263 |                     + lr ** 2 * C_Sr * (g * lf + u[1] * h)
 264 |                 )
 265 |                 * x[5]
 266 |                 + mu
 267 |                 * m
 268 |                 / (I * (lr + lf))
 269 |                 * (lr * C_Sr * (g * lf + u[1] * h) - lf * C_Sf * (g * lr - u[1] * h))
 270 |                 * x[6]
 271 |                 + mu * m / (I * (lr + lf)) * lf * C_Sf * (g * lr - u[1] * h) * x[2],
 272 |                 (
 273 |                     mu
 274 |                     / (x[3] ** 2 * (lr + lf))
 275 |                     * (
 276 |                         C_Sr * (g * lf + u[1] * h) * lr
 277 |                         - C_Sf * (g * lr - u[1] * h) * lf
 278 |                     )
 279 |                     - 1
 280 |                 )
 281 |                 * x[5]
 282 |                 - mu
 283 |                 / (x[3] * (lr + lf))
 284 |                 * (C_Sr * (g * lf + u[1] * h) + C_Sf * (g * lr - u[1] * h))
 285 |                 * x[6]
 286 |                 + mu / (x[3] * (lr + lf)) * (C_Sf * (g * lr - u[1] * h)) * x[2],
 287 |             ]
 288 |         )
 289 | 
 290 |     return f
 291 | 
 292 | 
 293 | @njit(cache=True)
 294 | def pid(speed, steer, current_speed, current_steer, max_sv, max_a, max_v, min_v):
 295 |     """
 296 |     Basic controller for speed/steer -> accl./steer vel.
 297 | 
 298 |         Args:
 299 |             speed (float): desired input speed
 300 |             steer (float): desired input steering angle
 301 | 
 302 |         Returns:
 303 |             accl (float): desired input acceleration
 304 |             sv (float): desired input steering velocity
 305 |     """
 306 |     # steering
 307 |     steer_diff = steer - current_steer
 308 |     if np.fabs(steer_diff) > 1e-4:
 309 |         sv = (steer_diff / np.fabs(steer_diff)) * max_sv
 310 |     else:
 311 |         sv = 0.0
 312 | 
 313 |     # accl
 314 |     vel_diff = speed - current_speed
 315 |     # currently forward
 316 |     if current_speed > 0.0:
 317 |         if vel_diff > 0:
 318 |             # accelerate
 319 |             kp = 10.0 * max_a / max_v
 320 |             accl = kp * vel_diff
 321 |         else:
 322 |             # braking
 323 |             kp = 10.0 * max_a / (-min_v)
 324 |             accl = kp * vel_diff
 325 |     # currently backwards
 326 |     else:
 327 |         if vel_diff > 0:
 328 |             # braking
 329 |             kp = 2.0 * max_a / max_v
 330 |             accl = kp * vel_diff
 331 |         else:
 332 |             # accelerating
 333 |             kp = 2.0 * max_a / (-min_v)
 334 |             accl = kp * vel_diff
 335 | 
 336 |     return accl, sv
 337 | 
 338 | 
 339 | def func_KS(
 340 |     x,
 341 |     t,
 342 |     u,
 343 |     mu,
 344 |     C_Sf,
 345 |     C_Sr,
 346 |     lf,
 347 |     lr,
 348 |     h,
 349 |     m,
 350 |     I,
 351 |     s_min,
 352 |     s_max,
 353 |     sv_min,
 354 |     sv_max,
 355 |     v_switch,
 356 |     a_max,
 357 |     v_min,
 358 |     v_max,
 359 | ):
 360 |     f = vehicle_dynamics_ks(
 361 |         x,
 362 |         u,
 363 |         mu,
 364 |         C_Sf,
 365 |         C_Sr,
 366 |         lf,
 367 |         lr,
 368 |         h,
 369 |         m,
 370 |         I,
 371 |         s_min,
 372 |         s_max,
 373 |         sv_min,
 374 |         sv_max,
 375 |         v_switch,
 376 |         a_max,
 377 |         v_min,
 378 |         v_max,
 379 |     )
 380 |     return f
 381 | 
 382 | 
 383 | def func_ST(
 384 |     x,
 385 |     t,
 386 |     u,
 387 |     mu,
 388 |     C_Sf,
 389 |     C_Sr,
 390 |     lf,
 391 |     lr,
 392 |     h,
 393 |     m,
 394 |     I,
 395 |     s_min,
 396 |     s_max,
 397 |     sv_min,
 398 |     sv_max,
 399 |     v_switch,
 400 |     a_max,
 401 |     v_min,
 402 |     v_max,
 403 | ):
 404 |     f = vehicle_dynamics_st(
 405 |         x,
 406 |         u,
 407 |         mu,
 408 |         C_Sf,
 409 |         C_Sr,
 410 |         lf,
 411 |         lr,
 412 |         h,
 413 |         m,
 414 |         I,
 415 |         s_min,
 416 |         s_max,
 417 |         sv_min,
 418 |         sv_max,
 419 |         v_switch,
 420 |         a_max,
 421 |         v_min,
 422 |         v_max,
 423 |     )
 424 |     return f
 425 | 
 426 | 
 427 | class DynamicsTest(unittest.TestCase):
 428 |     def setUp(self):
 429 |         # test params
 430 |         self.mu = 1.0489
 431 |         self.C_Sf = 21.92 / 1.0489
 432 |         self.C_Sr = 21.92 / 1.0489
 433 |         self.lf = 0.3048 * 3.793293
 434 |         self.lr = 0.3048 * 4.667707
 435 |         self.h = 0.3048 * 2.01355
 436 |         self.m = 4.4482216152605 / 0.3048 * 74.91452
 437 |         self.I = 4.4482216152605 * 0.3048 * 1321.416
 438 | 
 439 |         # steering constraints
 440 |         self.s_min = -1.066  # minimum steering angle [rad]
 441 |         self.s_max = 1.066  # maximum steering angle [rad]
 442 |         self.sv_min = -0.4  # minimum steering velocity [rad/s]
 443 |         self.sv_max = 0.4  # maximum steering velocity [rad/s]
 444 | 
 445 |         # longitudinal constraints
 446 |         self.v_min = -13.6  # minimum velocity [m/s]
 447 |         self.v_max = 50.8  # minimum velocity [m/s]
 448 |         self.v_switch = 7.319  # switching velocity [m/s]
 449 |         self.a_max = 11.5  # maximum absolute acceleration [m/s^2]
 450 | 
 451 |     def test_derivatives(self):
 452 |         # ground truth derivatives
 453 |         f_ks_gt = [
 454 |             16.3475935934250209,
 455 |             0.4819314886013121,
 456 |             0.1500000000000000,
 457 |             5.1464424102339752,
 458 |             0.2401426578627629,
 459 |         ]
 460 |         f_st_gt = [
 461 |             15.7213512030862397,
 462 |             0.0925527979719355,
 463 |             0.1500000000000000,
 464 |             5.3536773276413925,
 465 |             0.0529001056654038,
 466 |             0.6435589397748606,
 467 |             0.0313297971641291,
 468 |         ]
 469 | 
 470 |         # system dynamics
 471 |         g = 9.81
 472 |         x_ks = np.array(
 473 |             [
 474 |                 3.9579422297936526,
 475 |                 0.0391650102771405,
 476 |                 0.0378491427211811,
 477 |                 16.3546957860883566,
 478 |                 0.0294717351052816,
 479 |             ]
 480 |         )
 481 |         x_st = np.array(
 482 |             [
 483 |                 2.0233348142065677,
 484 |                 0.0041907137716636,
 485 |                 0.0197545248559617,
 486 |                 15.7216236334290116,
 487 |                 0.0025857914776859,
 488 |                 0.0529001056654038,
 489 |                 0.0033012170610298,
 490 |             ]
 491 |         )
 492 |         v_delta = 0.15
 493 |         acc = 0.63 * g
 494 |         u = np.array([v_delta, acc])
 495 | 
 496 |         f_ks = vehicle_dynamics_ks(
 497 |             x_ks,
 498 |             u,
 499 |             self.mu,
 500 |             self.C_Sf,
 501 |             self.C_Sr,
 502 |             self.lf,
 503 |             self.lr,
 504 |             self.h,
 505 |             self.m,
 506 |             self.I,
 507 |             self.s_min,
 508 |             self.s_max,
 509 |             self.sv_min,
 510 |             self.sv_max,
 511 |             self.v_switch,
 512 |             self.a_max,
 513 |             self.v_min,
 514 |             self.v_max,
 515 |         )
 516 |         f_st = vehicle_dynamics_st(
 517 |             x_st,
 518 |             u,
 519 |             self.mu,
 520 |             self.C_Sf,
 521 |             self.C_Sr,
 522 |             self.lf,
 523 |             self.lr,
 524 |             self.h,
 525 |             self.m,
 526 |             self.I,
 527 |             self.s_min,
 528 |             self.s_max,
 529 |             self.sv_min,
 530 |             self.sv_max,
 531 |             self.v_switch,
 532 |             self.a_max,
 533 |             self.v_min,
 534 |             self.v_max,
 535 |         )
 536 | 
 537 |         start = time.time()
 538 |         for i in range(10000):
 539 |             f_st = vehicle_dynamics_st(
 540 |                 x_st,
 541 |                 u,
 542 |                 self.mu,
 543 |                 self.C_Sf,
 544 |                 self.C_Sr,
 545 |                 self.lf,
 546 |                 self.lr,
 547 |                 self.h,
 548 |                 self.m,
 549 |                 self.I,
 550 |                 self.s_min,
 551 |                 self.s_max,
 552 |                 self.sv_min,
 553 |                 self.sv_max,
 554 |                 self.v_switch,
 555 |                 self.a_max,
 556 |                 self.v_min,
 557 |                 self.v_max,
 558 |             )
 559 |         duration = time.time() - start
 560 |         avg_fps = 10000 / duration
 561 | 
 562 |         self.assertAlmostEqual(np.max(np.abs(f_ks_gt - f_ks)), 0.0)
 563 |         self.assertAlmostEqual(np.max(np.abs(f_st_gt - f_st)), 0.0)
 564 |         self.assertGreater(avg_fps, 5000)
 565 | 
 566 |     def test_zeroinit_roll(self):
 567 |         from scipy.integrate import odeint
 568 | 
 569 |         # testing for zero initial state, zero input singularities
 570 |         g = 9.81
 571 |         t_start = 0.0
 572 |         t_final = 1.0
 573 |         delta0 = 0.0
 574 |         vel0 = 0.0
 575 |         Psi0 = 0.0
 576 |         dotPsi0 = 0.0
 577 |         beta0 = 0.0
 578 |         sy0 = 0.0
 579 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 580 | 
 581 |         x0_KS = np.array(initial_state[0:5])
 582 |         x0_ST = np.array(initial_state)
 583 | 
 584 |         # time vector
 585 |         t = np.arange(t_start, t_final, 1e-4)
 586 | 
 587 |         # set input: rolling car (velocity should stay constant)
 588 |         u = np.array([0.0, 0.0])
 589 | 
 590 |         # simulate single-track model
 591 |         x_roll_st = odeint(
 592 |             func_ST,
 593 |             x0_ST,
 594 |             t,
 595 |             args=(
 596 |                 u,
 597 |                 self.mu,
 598 |                 self.C_Sf,
 599 |                 self.C_Sr,
 600 |                 self.lf,
 601 |                 self.lr,
 602 |                 self.h,
 603 |                 self.m,
 604 |                 self.I,
 605 |                 self.s_min,
 606 |                 self.s_max,
 607 |                 self.sv_min,
 608 |                 self.sv_max,
 609 |                 self.v_switch,
 610 |                 self.a_max,
 611 |                 self.v_min,
 612 |                 self.v_max,
 613 |             ),
 614 |         )
 615 |         # simulate kinematic single-track model
 616 |         x_roll_ks = odeint(
 617 |             func_KS,
 618 |             x0_KS,
 619 |             t,
 620 |             args=(
 621 |                 u,
 622 |                 self.mu,
 623 |                 self.C_Sf,
 624 |                 self.C_Sr,
 625 |                 self.lf,
 626 |                 self.lr,
 627 |                 self.h,
 628 |                 self.m,
 629 |                 self.I,
 630 |                 self.s_min,
 631 |                 self.s_max,
 632 |                 self.sv_min,
 633 |                 self.sv_max,
 634 |                 self.v_switch,
 635 |                 self.a_max,
 636 |                 self.v_min,
 637 |                 self.v_max,
 638 |             ),
 639 |         )
 640 | 
 641 |         self.assertTrue(all(x_roll_st[-1] == x0_ST))
 642 |         self.assertTrue(all(x_roll_ks[-1] == x0_KS))
 643 | 
 644 |     def test_zeroinit_dec(self):
 645 |         from scipy.integrate import odeint
 646 | 
 647 |         # testing for zero initial state, decelerating input singularities
 648 |         g = 9.81
 649 |         t_start = 0.0
 650 |         t_final = 1.0
 651 |         delta0 = 0.0
 652 |         vel0 = 0.0
 653 |         Psi0 = 0.0
 654 |         dotPsi0 = 0.0
 655 |         beta0 = 0.0
 656 |         sy0 = 0.0
 657 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 658 | 
 659 |         x0_KS = np.array(initial_state[0:5])
 660 |         x0_ST = np.array(initial_state)
 661 | 
 662 |         # time vector
 663 |         t = np.arange(t_start, t_final, 1e-4)
 664 | 
 665 |         # set decel input
 666 |         u = np.array([0.0, -0.7 * g])
 667 | 
 668 |         # simulate single-track model
 669 |         x_dec_st = odeint(
 670 |             func_ST,
 671 |             x0_ST,
 672 |             t,
 673 |             args=(
 674 |                 u,
 675 |                 self.mu,
 676 |                 self.C_Sf,
 677 |                 self.C_Sr,
 678 |                 self.lf,
 679 |                 self.lr,
 680 |                 self.h,
 681 |                 self.m,
 682 |                 self.I,
 683 |                 self.s_min,
 684 |                 self.s_max,
 685 |                 self.sv_min,
 686 |                 self.sv_max,
 687 |                 self.v_switch,
 688 |                 self.a_max,
 689 |                 self.v_min,
 690 |                 self.v_max,
 691 |             ),
 692 |         )
 693 |         # simulate kinematic single-track model
 694 |         x_dec_ks = odeint(
 695 |             func_KS,
 696 |             x0_KS,
 697 |             t,
 698 |             args=(
 699 |                 u,
 700 |                 self.mu,
 701 |                 self.C_Sf,
 702 |                 self.C_Sr,
 703 |                 self.lf,
 704 |                 self.lr,
 705 |                 self.h,
 706 |                 self.m,
 707 |                 self.I,
 708 |                 self.s_min,
 709 |                 self.s_max,
 710 |                 self.sv_min,
 711 |                 self.sv_max,
 712 |                 self.v_switch,
 713 |                 self.a_max,
 714 |                 self.v_min,
 715 |                 self.v_max,
 716 |             ),
 717 |         )
 718 | 
 719 |         # ground truth for single-track model
 720 |         x_dec_st_gt = [
 721 |             -3.4335000000000013,
 722 |             0.0000000000000000,
 723 |             0.0000000000000000,
 724 |             -6.8670000000000018,
 725 |             0.0000000000000000,
 726 |             0.0000000000000000,
 727 |             0.0000000000000000,
 728 |         ]
 729 |         # ground truth for kinematic single-track model
 730 |         x_dec_ks_gt = [
 731 |             -3.4335000000000013,
 732 |             0.0000000000000000,
 733 |             0.0000000000000000,
 734 |             -6.8670000000000018,
 735 |             0.0000000000000000,
 736 |         ]
 737 | 
 738 |         self.assertTrue(all(abs(x_dec_st[-1] - x_dec_st_gt) < 1e-2))
 739 |         self.assertTrue(all(abs(x_dec_ks[-1] - x_dec_ks_gt) < 1e-2))
 740 | 
 741 |     def test_zeroinit_acc(self):
 742 |         from scipy.integrate import odeint
 743 | 
 744 |         # testing for zero initial state, accelerating with left steer input singularities
 745 |         # wheel spin and velocity should increase more wheel spin at rear
 746 |         g = 9.81
 747 |         t_start = 0.0
 748 |         t_final = 1.0
 749 |         delta0 = 0.0
 750 |         vel0 = 0.0
 751 |         Psi0 = 0.0
 752 |         dotPsi0 = 0.0
 753 |         beta0 = 0.0
 754 |         sy0 = 0.0
 755 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 756 | 
 757 |         x0_KS = np.array(initial_state[0:5])
 758 |         x0_ST = np.array(initial_state)
 759 | 
 760 |         # time vector
 761 |         t = np.arange(t_start, t_final, 1e-4)
 762 | 
 763 |         # set decel input
 764 |         u = np.array([0.15, 0.63 * g])
 765 | 
 766 |         # simulate single-track model
 767 |         x_acc_st = odeint(
 768 |             func_ST,
 769 |             x0_ST,
 770 |             t,
 771 |             args=(
 772 |                 u,
 773 |                 self.mu,
 774 |                 self.C_Sf,
 775 |                 self.C_Sr,
 776 |                 self.lf,
 777 |                 self.lr,
 778 |                 self.h,
 779 |                 self.m,
 780 |                 self.I,
 781 |                 self.s_min,
 782 |                 self.s_max,
 783 |                 self.sv_min,
 784 |                 self.sv_max,
 785 |                 self.v_switch,
 786 |                 self.a_max,
 787 |                 self.v_min,
 788 |                 self.v_max,
 789 |             ),
 790 |         )
 791 |         # simulate kinematic single-track model
 792 |         x_acc_ks = odeint(
 793 |             func_KS,
 794 |             x0_KS,
 795 |             t,
 796 |             args=(
 797 |                 u,
 798 |                 self.mu,
 799 |                 self.C_Sf,
 800 |                 self.C_Sr,
 801 |                 self.lf,
 802 |                 self.lr,
 803 |                 self.h,
 804 |                 self.m,
 805 |                 self.I,
 806 |                 self.s_min,
 807 |                 self.s_max,
 808 |                 self.sv_min,
 809 |                 self.sv_max,
 810 |                 self.v_switch,
 811 |                 self.a_max,
 812 |                 self.v_min,
 813 |                 self.v_max,
 814 |             ),
 815 |         )
 816 | 
 817 |         # ground truth for single-track model
 818 |         x_acc_st_gt = [
 819 |             3.0731976046859715,
 820 |             0.2869835398304389,
 821 |             0.1500000000000000,
 822 |             6.1802999999999999,
 823 |             0.1097747074946325,
 824 |             0.3248268063223301,
 825 |             0.0697547542798040,
 826 |         ]
 827 |         # ground truth for kinematic single-track model
 828 |         x_acc_ks_gt = [
 829 |             3.0845676868494927,
 830 |             0.1484249221523042,
 831 |             0.1500000000000000,
 832 |             6.1803000000000017,
 833 |             0.1203664469224163,
 834 |         ]
 835 | 
 836 |         self.assertTrue(all(abs(x_acc_st[-1] - x_acc_st_gt) < 1e-2))
 837 |         self.assertTrue(all(abs(x_acc_ks[-1] - x_acc_ks_gt) < 1e-2))
 838 | 
 839 |     def test_zeroinit_rollleft(self):
 840 |         from scipy.integrate import odeint
 841 | 
 842 |         # testing for zero initial state, rolling and steering left input singularities
 843 |         g = 9.81
 844 |         t_start = 0.0
 845 |         t_final = 1.0
 846 |         delta0 = 0.0
 847 |         vel0 = 0.0
 848 |         Psi0 = 0.0
 849 |         dotPsi0 = 0.0
 850 |         beta0 = 0.0
 851 |         sy0 = 0.0
 852 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 853 | 
 854 |         x0_KS = np.array(initial_state[0:5])
 855 |         x0_ST = np.array(initial_state)
 856 | 
 857 |         # time vector
 858 |         t = np.arange(t_start, t_final, 1e-4)
 859 | 
 860 |         # set decel input
 861 |         u = np.array([0.15, 0.0])
 862 | 
 863 |         # simulate single-track model
 864 |         x_left_st = odeint(
 865 |             func_ST,
 866 |             x0_ST,
 867 |             t,
 868 |             args=(
 869 |                 u,
 870 |                 self.mu,
 871 |                 self.C_Sf,
 872 |                 self.C_Sr,
 873 |                 self.lf,
 874 |                 self.lr,
 875 |                 self.h,
 876 |                 self.m,
 877 |                 self.I,
 878 |                 self.s_min,
 879 |                 self.s_max,
 880 |                 self.sv_min,
 881 |                 self.sv_max,
 882 |                 self.v_switch,
 883 |                 self.a_max,
 884 |                 self.v_min,
 885 |                 self.v_max,
 886 |             ),
 887 |         )
 888 |         # simulate kinematic single-track model
 889 |         x_left_ks = odeint(
 890 |             func_KS,
 891 |             x0_KS,
 892 |             t,
 893 |             args=(
 894 |                 u,
 895 |                 self.mu,
 896 |                 self.C_Sf,
 897 |                 self.C_Sr,
 898 |                 self.lf,
 899 |                 self.lr,
 900 |                 self.h,
 901 |                 self.m,
 902 |                 self.I,
 903 |                 self.s_min,
 904 |                 self.s_max,
 905 |                 self.sv_min,
 906 |                 self.sv_max,
 907 |                 self.v_switch,
 908 |                 self.a_max,
 909 |                 self.v_min,
 910 |                 self.v_max,
 911 |             ),
 912 |         )
 913 | 
 914 |         # ground truth for single-track model
 915 |         x_left_st_gt = [
 916 |             0.0000000000000000,
 917 |             0.0000000000000000,
 918 |             0.1500000000000000,
 919 |             0.0000000000000000,
 920 |             0.0000000000000000,
 921 |             0.0000000000000000,
 922 |             0.0000000000000000,
 923 |         ]
 924 |         # ground truth for kinematic single-track model
 925 |         x_left_ks_gt = [
 926 |             0.0000000000000000,
 927 |             0.0000000000000000,
 928 |             0.1500000000000000,
 929 |             0.0000000000000000,
 930 |             0.0000000000000000,
 931 |         ]
 932 | 
 933 |         self.assertTrue(all(abs(x_left_st[-1] - x_left_st_gt) < 1e-2))
 934 |         self.assertTrue(all(abs(x_left_ks[-1] - x_left_ks_gt) < 1e-2))
 935 | 
 936 | 
 937 | if __name__ == "__main__":
 938 |     unittest.main()

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\f110_env.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | """
  24 | Author: Hongrui Zheng
  25 | """
  26 | 
  27 | # gym imports
  28 | import gym
  29 | from gym import error, spaces, utils
  30 | from gym.utils import seeding
  31 | 
  32 | # base classes
  33 | from f110_gym.envs.base_classes import Simulator, Integrator
  34 | 
  35 | # others
  36 | import numpy as np
  37 | import os
  38 | import time
  39 | 
  40 | # gl
  41 | import pyglet
  42 | 
  43 | pyglet.options["debug_gl"] = False
  44 | from pyglet import gl
  45 | 
  46 | # constants
  47 | 
  48 | # rendering
  49 | VIDEO_W = 600
  50 | VIDEO_H = 400
  51 | WINDOW_W = 1000
  52 | WINDOW_H = 800
  53 | 
  54 | 
  55 | class F110Env(gym.Env):
  56 |     """
  57 |     OpenAI gym environment for F1TENTH
  58 |     
  59 |     Env should be initialized by calling gym.make('f110_gym:f110-v0', **kwargs)
  60 | 
  61 |     Args:
  62 |         kwargs:
  63 |             seed (int, default=12345): seed for random state and reproducibility
  64 |             
  65 |             map (str, default='vegas'): name of the map used for the environment. Currently, available environments include: 'berlin', 'vegas', 'skirk'. You could use a string of the absolute path to the yaml file of your custom map.
  66 |         
  67 |             map_ext (str, default='png'): image extension of the map image file. For example 'png', 'pgm'
  68 |         
  69 |             params (dict, default={'mu': 1.0489, 'C_Sf':, 'C_Sr':, 'lf': 0.15875, 'lr': 0.17145, 'h': 0.074, 'm': 3.74, 'I': 0.04712, 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2, 'v_switch':7.319, 'a_max': 9.51, 'v_min':-5.0, 'v_max': 20.0, 'width': 0.31, 'length': 0.58}): dictionary of vehicle parameters.
  70 |             mu: surface friction coefficient
  71 |             C_Sf: Cornering stiffness coefficient, front
  72 |             C_Sr: Cornering stiffness coefficient, rear
  73 |             lf: Distance from center of gravity to front axle
  74 |             lr: Distance from center of gravity to rear axle
  75 |             h: Height of center of gravity
  76 |             m: Total mass of the vehicle
  77 |             I: Moment of inertial of the entire vehicle about the z axis
  78 |             s_min: Minimum steering angle constraint
  79 |             s_max: Maximum steering angle constraint
  80 |             sv_min: Minimum steering velocity constraint
  81 |             sv_max: Maximum steering velocity constraint
  82 |             v_switch: Switching velocity (velocity at which the acceleration is no longer able to create wheel spin)
  83 |             a_max: Maximum longitudinal acceleration
  84 |             v_min: Minimum longitudinal velocity
  85 |             v_max: Maximum longitudinal velocity
  86 |             width: width of the vehicle in meters
  87 |             length: length of the vehicle in meters
  88 | 
  89 |             num_agents (int, default=2): number of agents in the environment
  90 | 
  91 |             timestep (float, default=0.01): physics timestep
  92 | 
  93 |             ego_idx (int, default=0): ego's index in list of agents
  94 |     """
  95 | 
  96 |     metadata = {"render.modes": ["human", "human_fast"]}
  97 | 
  98 |     # rendering
  99 |     renderer = None
 100 |     current_obs = None
 101 |     render_callbacks = []
 102 | 
 103 |     def __init__(self, **kwargs):
 104 |         # kwargs extraction
 105 |         try:
 106 |             self.seed = kwargs["seed"]
 107 |         except:
 108 |             self.seed = 12345
 109 |         try:
 110 |             self.map_name = kwargs["map"]
 111 |             # different default maps
 112 |             if self.map_name == "berlin":
 113 |                 self.map_path = (
 114 |                     os.path.dirname(os.path.abspath(__file__)) + "/maps/berlin.yaml"
 115 |                 )
 116 |             elif self.map_name == "skirk":
 117 |                 self.map_path = (
 118 |                     os.path.dirname(os.path.abspath(__file__)) + "/maps/skirk.yaml"
 119 |                 )
 120 |             elif self.map_name == "levine":
 121 |                 self.map_path = (
 122 |                     os.path.dirname(os.path.abspath(__file__)) + "/maps/levine.yaml"
 123 |                 )
 124 |             else:
 125 |                 self.map_path = self.map_name + ".yaml"
 126 |         except:
 127 |             self.map_path = (
 128 |                 os.path.dirname(os.path.abspath(__file__)) + "/maps/vegas.yaml"
 129 |             )
 130 | 
 131 |         try:
 132 |             self.map_ext = kwargs["map_ext"]
 133 |         except:
 134 |             self.map_ext = ".png"
 135 | 
 136 |         try:
 137 |             self.params = kwargs["params"]
 138 |         except:
 139 |             self.params = {
 140 |                 "mu": 1.0489,
 141 |                 "C_Sf": 4.718,
 142 |                 "C_Sr": 5.4562,
 143 |                 "lf": 0.15875,
 144 |                 "lr": 0.17145,
 145 |                 "h": 0.074,
 146 |                 "m": 3.74,
 147 |                 "I": 0.04712,
 148 |                 "s_min": -0.4189,
 149 |                 "s_max": 0.4189,
 150 |                 "sv_min": -3.2,
 151 |                 "sv_max": 3.2,
 152 |                 "v_switch": 7.319,
 153 |                 "a_max": 9.51,
 154 |                 "v_min": -5.0,
 155 |                 "v_max": 20.0,
 156 |                 "width": 0.31,
 157 |                 "length": 0.58,
 158 |             }
 159 | 
 160 |         # simulation parameters
 161 |         try:
 162 |             self.num_agents = kwargs["num_agents"]
 163 |         except:
 164 |             self.num_agents = 2
 165 | 
 166 |         try:
 167 |             self.timestep = kwargs["timestep"]
 168 |         except:
 169 |             self.timestep = 0.01
 170 | 
 171 |         # default ego index
 172 |         try:
 173 |             self.ego_idx = kwargs["ego_idx"]
 174 |         except:
 175 |             self.ego_idx = 0
 176 | 
 177 |         # default integrator
 178 |         try:
 179 |             self.integrator = kwargs["integrator"]
 180 |         except:
 181 |             self.integrator = Integrator.RK4
 182 | 
 183 |         # radius to consider done
 184 |         self.start_thresh = 0.5  # 10cm
 185 | 
 186 |         # env states
 187 |         self.poses_x = []
 188 |         self.poses_y = []
 189 |         self.poses_theta = []
 190 |         self.collisions = np.zeros((self.num_agents,))
 191 |         # TODO: collision_idx not used yet
 192 |         # self.collision_idx = -1 * np.ones((self.num_agents, ))
 193 | 
 194 |         # loop completion
 195 |         self.near_start = True
 196 |         self.num_toggles = 0
 197 | 
 198 |         # race info
 199 |         self.lap_times = np.zeros((self.num_agents,))
 200 |         self.lap_counts = np.zeros((self.num_agents,))
 201 |         self.current_time = 0.0
 202 | 
 203 |         # finish line info
 204 |         self.num_toggles = 0
 205 |         self.near_start = True
 206 |         self.near_starts = np.array([True] * self.num_agents)
 207 |         self.toggle_list = np.zeros((self.num_agents,))
 208 |         self.start_xs = np.zeros((self.num_agents,))
 209 |         self.start_ys = np.zeros((self.num_agents,))
 210 |         self.start_thetas = np.zeros((self.num_agents,))
 211 |         self.start_rot = np.eye(2)
 212 | 
 213 |         # initiate stuff
 214 |         self.sim = Simulator(
 215 |             self.params,
 216 |             self.num_agents,
 217 |             self.seed,
 218 |             time_step=self.timestep,
 219 |             integrator=self.integrator,
 220 |         )
 221 |         self.sim.set_map(self.map_path, self.map_ext)
 222 | 
 223 |         # stateful observations for rendering
 224 |         self.render_obs = None
 225 | 
 226 |     def __del__(self):
 227 |         """
 228 |         Finalizer, does cleanup
 229 |         """
 230 |         pass
 231 | 
 232 |     def _check_done(self):
 233 |         """
 234 |         Check if the current rollout is done
 235 |         
 236 |         Args:
 237 |             None
 238 | 
 239 |         Returns:
 240 |             done (bool): whether the rollout is done
 241 |             toggle_list (list[int]): each agent's toggle list for crossing the finish zone
 242 |         """
 243 | 
 244 |         # this is assuming 2 agents
 245 |         # TODO: switch to maybe s-based
 246 |         left_t = 2
 247 |         right_t = 2
 248 | 
 249 |         poses_x = np.array(self.poses_x) - self.start_xs
 250 |         poses_y = np.array(self.poses_y) - self.start_ys
 251 |         delta_pt = np.dot(self.start_rot, np.stack((poses_x, poses_y), axis=0))
 252 |         temp_y = delta_pt[1, :]
 253 |         idx1 = temp_y > left_t
 254 |         idx2 = temp_y < -right_t
 255 |         temp_y[idx1] -= left_t
 256 |         temp_y[idx2] = -right_t - temp_y[idx2]
 257 |         temp_y[np.invert(np.logical_or(idx1, idx2))] = 0
 258 | 
 259 |         dist2 = delta_pt[0, :] ** 2 + temp_y ** 2
 260 |         closes = dist2 <= 0.1
 261 |         for i in range(self.num_agents):
 262 |             if closes[i] and not self.near_starts[i]:
 263 |                 self.near_starts[i] = True
 264 |                 self.toggle_list[i] += 1
 265 |             elif not closes[i] and self.near_starts[i]:
 266 |                 self.near_starts[i] = False
 267 |                 self.toggle_list[i] += 1
 268 |             self.lap_counts[i] = self.toggle_list[i] // 2
 269 |             if self.toggle_list[i] < 4:
 270 |                 self.lap_times[i] = self.current_time
 271 | 
 272 |         done = (self.collisions[self.ego_idx]) or np.all(self.toggle_list >= 4)
 273 | 
 274 |         return bool(done), self.toggle_list >= 4
 275 | 
 276 |     def _update_state(self, obs_dict):
 277 |         """
 278 |         Update the env's states according to observations
 279 |         
 280 |         Args:
 281 |             obs_dict (dict): dictionary of observation
 282 | 
 283 |         Returns:
 284 |             None
 285 |         """
 286 |         self.poses_x = obs_dict["poses_x"]
 287 |         self.poses_y = obs_dict["poses_y"]
 288 |         self.poses_theta = obs_dict["poses_theta"]
 289 |         self.collisions = obs_dict["collisions"]
 290 | 
 291 |     def step(self, action):
 292 |         """
 293 |         Step function for the gym env
 294 | 
 295 |         Args:
 296 |             action (np.ndarray(num_agents, 2))
 297 | 
 298 |         Returns:
 299 |             obs (dict): observation of the current step
 300 |             reward (float, default=self.timestep): step reward, currently is physics timestep
 301 |             done (bool): if the simulation is done
 302 |             info (dict): auxillary information dictionary
 303 |         """
 304 | 
 305 |         # call simulation step
 306 |         obs = self.sim.step(action)
 307 |         obs["lap_times"] = self.lap_times
 308 |         obs["lap_counts"] = self.lap_counts
 309 | 
 310 |         F110Env.current_obs = obs
 311 | 
 312 |         self.render_obs = {
 313 |             "ego_idx": obs["ego_idx"],
 314 |             "poses_x": obs["poses_x"],
 315 |             "poses_y": obs["poses_y"],
 316 |             "poses_theta": obs["poses_theta"],
 317 |             "lap_times": obs["lap_times"],
 318 |             "lap_counts": obs["lap_counts"],
 319 |         }
 320 | 
 321 |         # times
 322 |         reward = self.timestep
 323 |         self.current_time = self.current_time + self.timestep
 324 | 
 325 |         # update data member
 326 |         self._update_state(obs)
 327 | 
 328 |         # check done
 329 |         done, toggle_list = self._check_done()
 330 |         info = {"checkpoint_done": toggle_list}
 331 | 
 332 |         return obs, reward, done, info
 333 | 
 334 |     def reset(self, poses):
 335 |         """
 336 |         Reset the gym environment by given poses
 337 | 
 338 |         Args:
 339 |             poses (np.ndarray (num_agents, 3)): poses to reset agents to
 340 | 
 341 |         Returns:
 342 |             obs (dict): observation of the current step
 343 |             reward (float, default=self.timestep): step reward, currently is physics timestep
 344 |             done (bool): if the simulation is done
 345 |             info (dict): auxillary information dictionary
 346 |         """
 347 |         # reset counters and data members
 348 |         self.current_time = 0.0
 349 |         self.collisions = np.zeros((self.num_agents,))
 350 |         self.num_toggles = 0
 351 |         self.near_start = True
 352 |         self.near_starts = np.array([True] * self.num_agents)
 353 |         self.toggle_list = np.zeros((self.num_agents,))
 354 | 
 355 |         # states after reset
 356 |         self.start_xs = poses[:, 0]
 357 |         self.start_ys = poses[:, 1]
 358 |         self.start_thetas = poses[:, 2]
 359 |         self.start_rot = np.array(
 360 |             [
 361 |                 [
 362 |                     np.cos(-self.start_thetas[self.ego_idx]),
 363 |                     -np.sin(-self.start_thetas[self.ego_idx]),
 364 |                 ],
 365 |                 [
 366 |                     np.sin(-self.start_thetas[self.ego_idx]),
 367 |                     np.cos(-self.start_thetas[self.ego_idx]),
 368 |                 ],
 369 |             ]
 370 |         )
 371 | 
 372 |         # call reset to simulator
 373 |         self.sim.reset(poses)
 374 | 
 375 |         # get no input observations
 376 |         action = np.zeros((self.num_agents, 2))
 377 |         obs, reward, done, info = self.step(action)
 378 | 
 379 |         self.render_obs = {
 380 |             "ego_idx": obs["ego_idx"],
 381 |             "poses_x": obs["poses_x"],
 382 |             "poses_y": obs["poses_y"],
 383 |             "poses_theta": obs["poses_theta"],
 384 |             "lap_times": obs["lap_times"],
 385 |             "lap_counts": obs["lap_counts"],
 386 |         }
 387 | 
 388 |         return obs, reward, done, info
 389 | 
 390 |     def update_map(self, map_path, map_ext):
 391 |         """
 392 |         Updates the map used by simulation
 393 | 
 394 |         Args:
 395 |             map_path (str): absolute path to the map yaml file
 396 |             map_ext (str): extension of the map image file
 397 | 
 398 |         Returns:
 399 |             None
 400 |         """
 401 |         self.sim.set_map(map_path, map_ext)
 402 | 
 403 |     def update_params(self, params, index=-1):
 404 |         """
 405 |         Updates the parameters used by simulation for vehicles
 406 |         
 407 |         Args:
 408 |             params (dict): dictionary of parameters
 409 |             index (int, default=-1): if >= 0 then only update a specific agent's params
 410 | 
 411 |         Returns:
 412 |             None
 413 |         """
 414 |         self.sim.update_params(params, agent_idx=index)
 415 | 
 416 |     def add_render_callback(self, callback_func):
 417 |         """
 418 |         Add extra drawing function to call during rendering.
 419 | 
 420 |         Args:
 421 |             callback_func (function (EnvRenderer) -> None): custom function to called during render()
 422 |         """
 423 | 
 424 |         F110Env.render_callbacks.append(callback_func)
 425 | 
 426 |     def render(self, mode="human"):
 427 |         """
 428 |         Renders the environment with pyglet. Use mouse scroll in the window to zoom in/out, use mouse click drag to pan. Shows the agents, the map, current fps (bottom left corner), and the race information near as text.
 429 | 
 430 |         Args:
 431 |             mode (str, default='human'): rendering mode, currently supports:
 432 |                 'human': slowed down rendering such that the env is rendered in a way that sim time elapsed is close to real time elapsed
 433 |                 'human_fast': render as fast as possible
 434 | 
 435 |         Returns:
 436 |             None
 437 |         """
 438 |         assert mode in ["human", "human_fast"]
 439 | 
 440 |         if F110Env.renderer is None:
 441 |             # first call, initialize everything
 442 |             from f110_gym.envs.rendering import EnvRenderer
 443 | 
 444 |             F110Env.renderer = EnvRenderer(WINDOW_W, WINDOW_H)
 445 |             F110Env.renderer.update_map(self.map_name, self.map_ext)
 446 | 
 447 |         F110Env.renderer.update_obs(self.render_obs)
 448 | 
 449 |         for render_callback in F110Env.render_callbacks:
 450 |             render_callback(F110Env.renderer)
 451 | 
 452 |         F110Env.renderer.dispatch_events()
 453 |         F110Env.renderer.on_draw()
 454 |         F110Env.renderer.flip()
 455 |         if mode == "human":
 456 |             time.sleep(0.005)
 457 |         elif mode == "human_fast":
 458 |             pass

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\f110_env_backup.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | """
  24 | Author: Hongrui Zheng
  25 | """
  26 | 
  27 | # gym imports
  28 | import gym
  29 | from gym import error, spaces, utils
  30 | from gym.utils import seeding
  31 | 
  32 | # zmq imports
  33 | import zmq
  34 | 
  35 | # protobuf import
  36 | import sim_requests_pb2
  37 | 
  38 | # others
  39 | import numpy as np
  40 | 
  41 | from numba import njit
  42 | from scipy.ndimage import distance_transform_edt as edt
  43 | 
  44 | from PIL import Image
  45 | import sys
  46 | import os
  47 | import signal
  48 | import subprocess
  49 | import math
  50 | import yaml
  51 | import csv
  52 | 
  53 | # from matplotlib.pyplot import imshow
  54 | # import matplotlib.pyplot as plt
  55 | 
  56 | 
  57 | class F110Env(gym.Env, utils.EzPickle):
  58 |     """
  59 |     OpenAI gym environment for F1/10 simulator
  60 |     Use 0mq's REQ-REP pattern to communicate to the C++ simulator
  61 |     ONE env has ONE corresponding C++ instance
  62 |     Need to create env with map input, full path to map yaml file, map pgm image and yaml should be in same directory
  63 | 
  64 |     should be initialized with a map, a timestep, and number of agents
  65 |     """
  66 | 
  67 |     metadata = {"render.modes": []}
  68 | 
  69 |     def __init__(self):
  70 |         # simualtor params
  71 |         self.params_set = False
  72 |         self.map_inited = False
  73 |         # params list is [mu, h_cg, l_r, cs_f, cs_r, I_z, mass]
  74 |         self.params = []
  75 |         # TODO: add multi agent stuff, need a _add_agent function of sth
  76 |         self.num_agents = 2
  77 |         self.timestep = 0.01
  78 | 
  79 |         # TODO: clean up the map path stuff, right now it's a init_map function
  80 |         self.map_path = None
  81 |         self.map_img = None
  82 | 
  83 |         # current_dir = os.path.dirname(os.path.abspath(__file__))
  84 |         # map_path = current_dir + '/../../../maps/levine.yaml'
  85 | 
  86 |         # default
  87 |         self.ego_idx = 0
  88 | 
  89 |         # TODO: also set these things in init function?
  90 |         self.timeout = 120.0
  91 |         # radius to consider done
  92 |         self.start_thresh = 0.5  # 10cm
  93 | 
  94 |         # env states
  95 |         # more accurate description should be ego car state
  96 |         # might not need to keep scan
  97 |         self.x = None
  98 |         self.y = None
  99 |         self.theta = None
 100 | 
 101 |         self.in_collision = False
 102 |         self.collision_angle = None
 103 | 
 104 |         # loop completion
 105 |         self.near_start = True
 106 |         self.num_toggles = 0
 107 | 
 108 |         # race info
 109 |         self.lap_times = [0.0, 0.0]
 110 |         self.lap_counts = [0, 0]
 111 | 
 112 |         # TODO: load the map (same as ROS .yaml format)
 113 |         # if not map_path.endswith('.yaml'):
 114 |         #     print('Gym env - Please use a yaml file for map input.')
 115 |         #     sys.exit()
 116 |         # load map img
 117 |         # map_img_path = 'levine.png'
 118 |         # self.map_img = cv2.imread(map_img_path, 0)
 119 |         # self.map_img = cv2.flip(self.map_img, 0)
 120 |         # self.map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
 121 |         # self.map_img = self.map_img.astype(np.float64)
 122 |         # self.map_img = self.map_img[::-1]
 123 |         # self.map_img = np.dot(self.map_img[..., :3], [0.29, 0.57, 0.14])
 124 |         # plt.imshow(self.map_img)
 125 |         # plt.show()
 126 | 
 127 |         # map metadata
 128 |         # self.map_height = self.map_img.shape[0]
 129 |         # self.map_width = self.map_img.shape[1]
 130 |         self.map_height = 0.0
 131 |         self.map_width = 0.0
 132 |         self.map_resolution = 0.0
 133 |         self.free_thresh = 0.0
 134 |         self.origin = []
 135 |         # load map metadata
 136 |         # with open(map_path, 'r') as yaml_stream:
 137 |         #     try:
 138 |         #         map_metadata = yaml.safe_load(yaml_stream)
 139 |         #         self.map_resolution = map_metadata['resolution']
 140 |         #         self.origin = map_metadata['origin']
 141 |         #         # print(self.origin)
 142 |         #         # self.free_thresh?????
 143 |         #     except yaml.YAMLError as ex:
 144 |         #         print(ex)
 145 | 
 146 |         # create zmq stuff
 147 |         # port number range from 6666 - 6766
 148 |         # max 100 tries to connect/bind
 149 |         tries = 0
 150 |         max_tries = 100
 151 |         min_port = 6666
 152 |         self.port = min_port
 153 |         self.context = zmq.Context()
 154 |         self.socket = self.context.socket(zmq.PAIR)
 155 |         while tries < max_tries:
 156 |             try:
 157 |                 self.socket.bind("tcp://*:%s" % str(min_port + tries))
 158 |                 # self.socket.connect('tcp://localhost:6666')
 159 |                 self.port = min_port + tries
 160 |                 break
 161 |             except:
 162 |                 tries = tries + 1
 163 |                 # print('Gym env - retrying for ' + str(tries) + ' times')
 164 | 
 165 |         print("Gym env - Connected env to port: " + str(self.port))
 166 | 
 167 |         # create cpp instance if create then need to pass port number
 168 |         # subprocess call assumes directory structure
 169 |         # init sim with arguments: [ex timestep num_agents port_num]
 170 |         # TODO: include other car params in argument
 171 |         # args = ['../build/sim_server', str(self.timestep), str(self.num_agents), str(self.port)]
 172 |         # self.sim_p = subprocess.Popen(args)
 173 |         self.sim_p = None
 174 | 
 175 |         # print('Gym env - env created, waiting for params...')
 176 | 
 177 |     def __del__(self):
 178 |         """
 179 |         Finalizer, does cleanup
 180 |         """
 181 |         if self.sim_p is None:
 182 |             pass
 183 |         else:
 184 |             os.kill(self.sim_p.pid, signal.SIGTERM)
 185 |             # print('Gym env - Sim child process killed.')
 186 | 
 187 |     def _start_executable(self, path):
 188 |         mu = self.params[0]
 189 |         h_cg = self.params[1]
 190 |         l_r = self.params[2]
 191 |         cs_f = self.params[3]
 192 |         cs_r = self.params[4]
 193 |         I_z = self.params[5]
 194 |         mass = self.params[6]
 195 |         args = [
 196 |             path + "sim_server",
 197 |             str(self.timestep),
 198 |             str(self.num_agents),
 199 |             str(self.port),
 200 |             str(mu),
 201 |             str(h_cg),
 202 |             str(l_r),
 203 |             str(cs_f),
 204 |             str(cs_r),
 205 |             str(I_z),
 206 |             str(mass),
 207 |         ]
 208 |         self.sim_p = subprocess.Popen(args)
 209 | 
 210 |     def _set_map(self):
 211 |         """
 212 |         Sets the map for the simulator instance
 213 |         """
 214 |         if not self.map_inited:
 215 |             print("Gym env - Sim map not initialized, call env.init_map() to init map.")
 216 |         # create and fill in protobuf
 217 |         map_request_proto = sim_requests_pb2.SimRequest()
 218 |         map_request_proto.type = 1
 219 |         map_request_proto.map_request.map.extend(
 220 |             (1.0 - self.map_img / 255.0).flatten().tolist()
 221 |         )
 222 |         map_request_proto.map_request.origin_x = self.origin[0]
 223 |         map_request_proto.map_request.origin_y = self.origin[1]
 224 |         map_request_proto.map_request.map_resolution = self.map_resolution
 225 |         # TODO: double check if this value is valid
 226 |         map_request_proto.map_request.free_threshold = self.free_thresh
 227 |         map_request_proto.map_request.map_height = self.map_height
 228 |         map_request_proto.map_request.map_width = self.map_width
 229 |         # serialization
 230 |         map_request_string = map_request_proto.SerializeToString()
 231 |         # send set map request
 232 |         # print('Gym env - Sending set map request...')
 233 |         self.socket.send(map_request_string)
 234 |         # print('Gym env - Map request sent.')
 235 |         # receive response from sim instance
 236 |         sim_response_string = self.socket.recv()
 237 |         # parse map response proto
 238 |         sim_response_proto = sim_requests_pb2.SimResponse()
 239 |         sim_response_proto.ParseFromString(sim_response_string)
 240 |         # get results
 241 |         set_map_result = sim_response_proto.map_result.result
 242 |         if set_map_result == 1:
 243 |             print("Gym env - Set map failed, exiting...")
 244 |             sys.exit()
 245 | 
 246 |     def _check_done(self):
 247 |         """
 248 |         Check if the episode is done
 249 |         This is in terms of the ego car
 250 |         For our case, whether the car ends up close enough to the starting point
 251 |         And if accumulated time is over the timeout
 252 |         return true if done, false if not
 253 |         This assumes start is always (0, 0)
 254 | 
 255 |         """
 256 |         # TODO: start not always 0, 0
 257 |         # dist_to_start = math.sqrt((self.x-self.start_x) ** 2 + (self.y-self.start_y) ** 2)
 258 |         left_t = 2
 259 |         right_t = 2
 260 |         timeout = self.current_time >= self.timeout
 261 |         if self.double_finish:
 262 |             poses_x = np.array(self.all_x) - self.start_xs
 263 |             poses_y = np.array(self.all_y) - self.start_ys
 264 |             delta_pt = np.dot(self.start_rot, np.stack((poses_x, poses_y), axis=0))
 265 |             temp_y = delta_pt[1, :]
 266 |             idx1 = temp_y > left_t
 267 |             idx2 = temp_y < -right_t
 268 |             temp_y[idx1] -= left_t
 269 |             temp_y[idx2] = -right_t - temp_y[idx2]
 270 |             temp_y[np.invert(np.logical_or(idx1, idx2))] = 0
 271 | 
 272 |             dist2 = delta_pt[0, :] ** 2 + temp_y ** 2
 273 |             closes = dist2 <= 0.1
 274 |             for i in range(self.num_agents):
 275 |                 if closes[i] and not self.near_starts[i]:
 276 |                     self.near_starts[i] = True
 277 |                     self.toggle_list[i] += 1
 278 |                 elif not closes[i] and self.near_starts[i]:
 279 |                     self.near_starts[i] = False
 280 |                     self.toggle_list[i] += 1
 281 |             done = self.in_collision | (timeout) | np.all(self.toggle_list >= 4)
 282 |             # only for two cars atm
 283 |             self.lap_counts[0] = np.floor(self.toggle_list[0] / 2)
 284 |             self.lap_counts[1] = np.floor(self.toggle_list[1] / 2)
 285 |             if self.toggle_list[0] < 4:
 286 |                 self.lap_times[0] = self.current_time
 287 |             if self.toggle_list[1] < 4:
 288 |                 self.lap_times[1] = self.current_time
 289 |             return done, self.toggle_list >= 4
 290 | 
 291 |         delta_pt = np.dot(
 292 |             self.start_rot, np.array([self.x - self.start_x, self.y - self.start_y])
 293 |         )
 294 |         if delta_pt[1] > left_t:  # left
 295 |             temp_y = delta_pt[1] - left_t
 296 |         elif delta_pt[1] < -right_t:  # right
 297 |             temp_y = -right_t - delta_pt[1]
 298 |         else:
 299 |             temp_y = 0
 300 |         dist2 = delta_pt[0] ** 2 + temp_y ** 2
 301 |         close = dist2 <= 0.1
 302 |         # close = dist_to_start <= self.start_thresh
 303 |         if close and not self.near_start:
 304 |             self.near_start = True
 305 |             self.num_toggles += 1
 306 |         elif not close and self.near_start:
 307 |             self.near_start = False
 308 |             self.num_toggles += 1
 309 |         done = self.in_collision | (timeout) | (self.num_toggles >= 4)
 310 |         return done
 311 | 
 312 |     def _check_passed(self):
 313 |         """
 314 |         Returns the times that the ego car overtook the other car
 315 |         """
 316 |         return 0
 317 | 
 318 |     def _update_state(self, obs_dict):
 319 |         """
 320 |         Update the env's states according to observations
 321 |         obs is observation dictionary
 322 |         """
 323 |         self.x = obs_dict["poses_x"][obs_dict["ego_idx"]]
 324 |         self.y = obs_dict["poses_y"][obs_dict["ego_idx"]]
 325 |         if self.double_finish:
 326 |             self.all_x = obs_dict["poses_x"]
 327 |             self.all_y = obs_dict["poses_y"]
 328 | 
 329 |         self.theta = obs_dict["poses_theta"][obs_dict["ego_idx"]]
 330 |         self.in_collision = obs_dict["collisions"][obs_dict["ego_idx"]]
 331 |         self.collision_angle = obs_dict["collision_angles"][obs_dict["ego_idx"]]
 332 | 
 333 |     # TODO: do we do the ray casting here or in C++?
 334 |     # if speed is a concern do it in C++?
 335 |     # numba shouldn't be a dependency of gym env
 336 |     def _raycast_opponents(self, obs_dict):
 337 |         # find the angle of beam of each car in each other's fov
 338 | 
 339 |         # set range of beams to raycast, ego and op
 340 | 
 341 |         # raycast beams, two set
 342 |         new_obs = {}
 343 |         return new_obs
 344 | 
 345 |     def step(self, action):
 346 |         # can't step if params not set
 347 |         if not self.params_set:
 348 |             print(
 349 |                 "ERROR - Gym Env - Params not set, call update params before stepping."
 350 |             )
 351 |             sys.exit()
 352 |         # action is a list of steering angles + command velocities
 353 |         # also a ego car index
 354 |         # action should a DICT with {'ego_idx': int, 'speed':[], 'steer':[]}
 355 |         step_request_proto = sim_requests_pb2.SimRequest()
 356 |         step_request_proto.type = 0
 357 |         step_request_proto.step_request.ego_idx = action["ego_idx"]
 358 |         step_request_proto.step_request.requested_vel.extend(action["speed"])
 359 |         step_request_proto.step_request.requested_ang.extend(action["steer"])
 360 |         # serialization
 361 |         step_request_string = step_request_proto.SerializeToString()
 362 |         # send step request
 363 |         self.socket.send(step_request_string)
 364 |         # receive response from sim instance
 365 |         sim_response_string = self.socket.recv()
 366 |         # print('Gym env - Received response for step request.')
 367 |         # parse map response proto
 368 |         sim_response_proto = sim_requests_pb2.SimResponse()
 369 |         sim_response_proto.ParseFromString(sim_response_string)
 370 |         # get results
 371 |         # make sure we have the right type of response
 372 |         response_type = sim_response_proto.type
 373 |         # TODO: also check for stepping fail
 374 |         if not response_type == 0:
 375 |             print("Gym env - Wrong response type for stepping, exiting...")
 376 |             sys.exit()
 377 |         observations_proto = sim_response_proto.sim_obs
 378 |         # make sure the ego idx matches
 379 |         if not observations_proto.ego_idx == action["ego_idx"]:
 380 |             print("Gym env - Ego index mismatch, exiting...")
 381 |             sys.exit()
 382 |         # get observations
 383 |         carobs_list = observations_proto.observations
 384 |         # construct observation dict
 385 |         # Observation DICT, assume indices consistent: {'ego_idx':int, 'scans':[[]], 'poses_x':[], 'poses_y':[], 'poses_theta':[], 'linear_vels_x':[], 'linear_vels_y':[], 'ang_vels_z':[], 'collisions':[], 'collision_angles':[]}
 386 |         obs = {
 387 |             "ego_idx": observations_proto.ego_idx,
 388 |             "scans": [],
 389 |             "poses_x": [],
 390 |             "poses_y": [],
 391 |             "poses_theta": [],
 392 |             "linear_vels_x": [],
 393 |             "linear_vels_y": [],
 394 |             "ang_vels_z": [],
 395 |             "collisions": [],
 396 |             "collision_angles": [],
 397 |             "lap_times": [],
 398 |             "lap_counts": [],
 399 |         }
 400 |         for car_obs in carobs_list:
 401 |             obs["scans"].append(car_obs.scan)
 402 |             obs["poses_x"].append(car_obs.pose_x)
 403 |             obs["poses_y"].append(car_obs.pose_y)
 404 |             if abs(car_obs.theta) < np.pi:
 405 |                 obs["poses_theta"].append(car_obs.theta)
 406 |             else:
 407 |                 obs["poses_theta"].append(-((2 * np.pi) - car_obs.theta))
 408 |             obs["linear_vels_x"].append(car_obs.linear_vel_x)
 409 |             obs["linear_vels_y"].append(car_obs.linear_vel_y)
 410 |             obs["ang_vels_z"].append(car_obs.ang_vel_z)
 411 |             obs["collisions"].append(car_obs.collision)
 412 |             obs["collision_angles"].append(car_obs.collision_angle)
 413 | 
 414 |         obs["lap_times"] = self.lap_times
 415 |         obs["lap_counts"] = self.lap_counts
 416 | 
 417 |         # TODO: do we need step reward?
 418 |         reward = self.timestep
 419 |         # update accumulated time in env
 420 |         self.current_time = self.current_time + self.timestep
 421 |         # TODO: donezo should be done in simulator? could be done here as well
 422 |         self._update_state(obs)
 423 |         if self.double_finish:
 424 |             done, temp = self._check_done()
 425 |             info = {"checkpoint_done": temp}
 426 |         else:
 427 |             done = self._check_done()
 428 |             info = {}
 429 | 
 430 |         # TODO: return obs, reward, done, info
 431 |         return obs, reward, done, info
 432 | 
 433 |     def reset(self, poses=None):
 434 | 
 435 |         self.current_time = 0.0
 436 |         self.in_collision = False
 437 |         self.collision_angles = None
 438 |         self.num_toggles = 0
 439 |         self.near_start = True
 440 |         self.near_starts = np.array([True] * self.num_agents)
 441 |         self.toggle_list = np.zeros((self.num_agents,))
 442 |         if poses:
 443 |             pose_x = poses["x"]
 444 |             pose_y = poses["y"]
 445 |             pose_theta = poses["theta"]
 446 |             self.start_x = pose_x[0]
 447 |             self.start_y = pose_y[0]
 448 |             self.start_theta = pose_theta[0]
 449 |             self.start_xs = np.array(pose_x)
 450 |             self.start_ys = np.array(pose_y)
 451 |             self.start_thetas = np.array(pose_theta)
 452 |             self.start_rot = np.array(
 453 |                 [
 454 |                     [np.cos(-self.start_theta), -np.sin(-self.start_theta)],
 455 |                     [np.sin(-self.start_theta), np.cos(-self.start_theta)],
 456 |                 ]
 457 |             )
 458 |             # create reset by pose proto
 459 |             reset_request_proto = sim_requests_pb2.SimRequest()
 460 |             reset_request_proto.type = 4
 461 |             reset_request_proto.reset_bypose_request.num_cars = self.num_agents
 462 |             reset_request_proto.reset_bypose_request.ego_idx = 0
 463 |             reset_request_proto.reset_bypose_request.car_x.extend(pose_x)
 464 |             reset_request_proto.reset_bypose_request.car_y.extend(pose_y)
 465 |             reset_request_proto.reset_bypose_request.car_theta.extend(pose_theta)
 466 |             reset_request_string = reset_request_proto.SerializeToString()
 467 |             self.socket.send(reset_request_string)
 468 |         else:
 469 |             # create reset proto
 470 |             self.start_x = 0.0
 471 |             self.start_y = 0.0
 472 |             self.start_theta = 0.0
 473 |             self.start_rot = np.array(
 474 |                 [
 475 |                     [np.cos(-self.start_theta), -np.sin(-self.start_theta)],
 476 |                     [np.sin(-self.start_theta), np.cos(-self.start_theta)],
 477 |                 ]
 478 |             )
 479 |             reset_request_proto = sim_requests_pb2.SimRequest()
 480 |             reset_request_proto.type = 2
 481 |             reset_request_proto.reset_request.num_cars = self.num_agents
 482 |             reset_request_proto.reset_request.ego_idx = 0
 483 |             # serialize reset proto
 484 |             reset_request_string = reset_request_proto.SerializeToString()
 485 |             # send reset proto string
 486 |             self.socket.send(reset_request_string)
 487 |         # receive response from sim
 488 |         reset_response_string = self.socket.recv()
 489 |         reset_response_proto = sim_requests_pb2.SimResponse()
 490 |         reset_response_proto.ParseFromString(reset_response_string)
 491 |         if reset_response_proto.reset_resp.result:
 492 |             print("Gym env - Reset failed")
 493 |             # TODO: failure handling
 494 |             return None
 495 |         # TODO: return with gym convention, one step?
 496 |         vels = [0.0] * self.num_agents
 497 |         angs = [0.0] * self.num_agents
 498 |         action = {"ego_idx": self.ego_idx, "speed": vels, "steer": angs}
 499 |         # print('Gym env - Reset done')
 500 |         obs, reward, done, info = self.step(action)
 501 |         # print('Gym env - step done for reset')
 502 |         return obs, reward, done, info
 503 | 
 504 |     def init_map(self, map_path, img_ext, rgb, flip):
 505 |         """
 506 |             init a map for the gym env
 507 |             map_path: full path for the yaml, same as ROS, img and yaml in same dir
 508 |             rgb: map grayscale or rgb
 509 |             flip: if map needs flipping
 510 |         """
 511 | 
 512 |         self.map_path = map_path
 513 |         if not map_path.endswith(".yaml"):
 514 |             print("Gym env - Please use a yaml file for map initialization.")
 515 |             print("Exiting...")
 516 |             sys.exit()
 517 | 
 518 |         # split yaml ext name
 519 |         map_img_path = os.path.splitext(self.map_path)[0] + img_ext
 520 |         self.map_img = np.array(
 521 |             Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM)
 522 |         )
 523 |         self.map_img = self.map_img.astype(np.float64)
 524 |         if flip:
 525 |             self.map_img = self.map_img[::-1]
 526 | 
 527 |         if rgb:
 528 |             self.map_img = np.dot(self.map_img[..., :3], [0.29, 0.57, 0.14])
 529 | 
 530 |         # update map metadata
 531 |         self.map_height = self.map_img.shape[0]
 532 |         self.map_width = self.map_img.shape[1]
 533 |         self.free_thresh = 0.6  # TODO: double check
 534 |         with open(self.map_path, "r") as yaml_stream:
 535 |             try:
 536 |                 map_metadata = yaml.safe_load(yaml_stream)
 537 |                 self.map_resolution = map_metadata["resolution"]
 538 |                 self.origin = map_metadata["origin"]
 539 |             except yaml.YAMLError as ex:
 540 |                 print(ex)
 541 |         self.map_inited = True
 542 | 
 543 |         # load waypoints
 544 |         # self.csv_path = os.path.splitext(self.map_path)[0] + '.csv'
 545 |         # with open(self.csv_path) as f:
 546 |         #     self.waypoints = [tuple(line) for line in csv.reader(f)]
 547 |         #     # waypoints are [x, y, speed, theta]
 548 |         #     self.waypoints = np.array([(float(pt[0]), float(pt[1]), float(pt[2]), float(pt[3])) for pt in self.waypoints])
 549 | 
 550 |     def render(self, mode="human", close=False):
 551 |         return
 552 | 
 553 |     # def get_min_dist(self, position):
 554 |     #     wpts = self.waypoints[:, 0:2]
 555 |     #      # = position[0:2]
 556 |     #     nearest_point, nearest_dist, t, i = self.nearest_point_on_trajectory(position, wpts)
 557 |     #     # speed = self.waypoints[i, 2]
 558 |     #     return nearest_dist
 559 | 
 560 |     # def nearest_point_on_trajectory(self, point, trajectory):
 561 |     #     '''
 562 |     #     Return the nearest point along the given piecewise linear trajectory.
 563 | 
 564 |     #     Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
 565 |     #     not be an issue so long as trajectories are not insanely long.
 566 | 
 567 |     #         Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)
 568 | 
 569 |     #     point: size 2 numpy array
 570 |     #     trajectory: Nx2 matrix of (x,y) trajectory waypoints
 571 |     #         - these must be unique. If they are not unique, a divide by 0 error will destroy the world
 572 |     #     '''
 573 |     #     diffs = trajectory[1:,:] - trajectory[:-1,:]
 574 |     #     l2s   = diffs[:,0]**2 + diffs[:,1]**2
 575 |     #     # this is equivalent to the elementwise dot product
 576 |     #     dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
 577 |     #     t = np.clip(dots / l2s, 0.0, 1.0)
 578 |     #     projections = trajectory[:-1,:] + (t*diffs.T).T
 579 |     #     dists = np.linalg.norm(point - projections,axis=1)
 580 |     #     min_dist_segment = np.argmin(dists)
 581 |     #     return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment
 582 | 
 583 |     def update_params(
 584 |         self, mu, h_cg, l_r, cs_f, cs_r, I_z, mass, exe_path, double_finish=False
 585 |     ):
 586 |         # if not self.sim_p is None:
 587 |         #     print('Gym env - Sim server exists, killing...')
 588 |         #     self.socket.send(b'dead')
 589 |         #     self.sim_p.kill()
 590 |         #     os.kill(self.sim_p.pid, signal.SIGINT)
 591 |         #     self.sim_p = None
 592 |         # print('in update params')
 593 | 
 594 |         self.params = [mu, h_cg, l_r, cs_f, cs_r, I_z, mass]
 595 |         self.params_set = True
 596 |         if self.sim_p is None:
 597 |             # print('starting ex and setting map')
 598 |             self._start_executable(exe_path)
 599 |             self._set_map()
 600 |         self.double_finish = double_finish
 601 |         # print('before creating proto')
 602 | 
 603 |         # create update proto
 604 |         update_param_proto = sim_requests_pb2.SimRequest()
 605 |         update_param_proto.type = 3
 606 |         update_param_proto.update_request.mu = mu
 607 |         update_param_proto.update_request.h_cg = h_cg
 608 |         update_param_proto.update_request.l_r = l_r
 609 |         update_param_proto.update_request.cs_f = cs_f
 610 |         update_param_proto.update_request.cs_r = cs_r
 611 |         update_param_proto.update_request.I_z = I_z
 612 |         update_param_proto.update_request.mass = mass
 613 |         # serialize reset proto
 614 |         update_param_string = update_param_proto.SerializeToString()
 615 |         # print('proto serialized')
 616 |         # send update param request
 617 |         self.socket.send(update_param_string)
 618 |         # print('Gym env - Update param request sent.')
 619 |         # receive response
 620 |         update_response_string = self.socket.recv()
 621 |         update_response_proto = sim_requests_pb2.SimResponse()
 622 |         update_response_proto.ParseFromString(update_response_string)
 623 |         if update_response_proto.update_resp.result:
 624 |             print("Gym env - Update param failed")
 625 |             return None
 626 | 
 627 |         # print('Gym env - params updated.')
 628 |         # start executable
 629 |         # self._start_executable()
 630 |         # call set map
 631 |         # self._set_map()

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\laser_models.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Prototype of Utility functions and classes for simulating 2D LIDAR scans
  26 | Author: Hongrui Zheng
  27 | """
  28 | 
  29 | import numpy as np
  30 | from numba import njit
  31 | from scipy.ndimage import distance_transform_edt as edt
  32 | from PIL import Image
  33 | import os
  34 | import yaml
  35 | 
  36 | import unittest
  37 | import timeit
  38 | 
  39 | 
  40 | def get_dt(bitmap, resolution):
  41 |     """
  42 |     Distance transformation, returns the distance matrix from the input bitmap.
  43 |     Uses scipy.ndimage, cannot be JITted.
  44 | 
  45 |         Args:
  46 |             bitmap (numpy.ndarray, (n, m)): input binary bitmap of the environment, where 0 is obstacles, and 255 (or anything > 0) is freespace
  47 |             resolution (float): resolution of the input bitmap (m/cell)
  48 | 
  49 |         Returns:
  50 |             dt (numpy.ndarray, (n, m)): output distance matrix, where each cell has the corresponding distance (in meters) to the closest obstacle
  51 |     """
  52 |     dt = resolution * edt(bitmap)
  53 |     return dt
  54 | 
  55 | 
  56 | @njit(cache=True)
  57 | def xy_2_rc(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution):
  58 |     """
  59 |     Translate (x, y) coordinate into (r, c) in the matrix
  60 | 
  61 |         Args:
  62 |             x (float): coordinate in x (m)
  63 |             y (float): coordinate in y (m)
  64 |             orig_x (float): x coordinate of the map origin (m)
  65 |             orig_y (float): y coordinate of the map origin (m)
  66 |         
  67 |         Returns:
  68 |             r (int): row number in the transform matrix of the given point
  69 |             c (int): column number in the transform matrix of the given point
  70 |     """
  71 |     # translation
  72 |     x_trans = x - orig_x
  73 |     y_trans = y - orig_y
  74 | 
  75 |     # rotation
  76 |     x_rot = x_trans * orig_c + y_trans * orig_s
  77 |     y_rot = -x_trans * orig_s + y_trans * orig_c
  78 | 
  79 |     # clip the state to be a cell
  80 |     if (
  81 |         x_rot < 0
  82 |         or x_rot >= width * resolution
  83 |         or y_rot < 0
  84 |         or y_rot >= height * resolution
  85 |     ):
  86 |         c = -1
  87 |         r = -1
  88 |     else:
  89 |         c = int(x_rot / resolution)
  90 |         r = int(y_rot / resolution)
  91 | 
  92 |     return r, c
  93 | 
  94 | 
  95 | @njit(cache=True)
  96 | def distance_transform(
  97 |     x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt
  98 | ):
  99 |     """
 100 |     Look up corresponding distance in the distance matrix
 101 | 
 102 |         Args:
 103 |             x (float): x coordinate of the lookup point
 104 |             y (float): y coordinate of the lookup point
 105 |             orig_x (float): x coordinate of the map origin (m)
 106 |             orig_y (float): y coordinate of the map origin (m)
 107 | 
 108 |         Returns:
 109 |             distance (float): corresponding shortest distance to obstacle in meters
 110 |     """
 111 |     r, c = xy_2_rc(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution)
 112 |     distance = dt[r, c]
 113 |     return distance
 114 | 
 115 | 
 116 | @njit(cache=True)
 117 | def trace_ray(
 118 |     x,
 119 |     y,
 120 |     theta_index,
 121 |     sines,
 122 |     cosines,
 123 |     eps,
 124 |     orig_x,
 125 |     orig_y,
 126 |     orig_c,
 127 |     orig_s,
 128 |     height,
 129 |     width,
 130 |     resolution,
 131 |     dt,
 132 |     max_range,
 133 | ):
 134 |     """
 135 |     Find the length of a specific ray at a specific scan angle theta
 136 |     Purely math calculation and loops, should be JITted.
 137 | 
 138 |         Args:
 139 |             x (float): current x coordinate of the ego (scan) frame
 140 |             y (float): current y coordinate of the ego (scan) frame
 141 |             theta_index(int): current index of the scan beam in the scan range
 142 |             sines (numpy.ndarray (n, )): pre-calculated sines of the angle array
 143 |             cosines (numpy.ndarray (n, )): pre-calculated cosines ...
 144 | 
 145 |         Returns:
 146 |             total_distance (float): the distance to first obstacle on the current scan beam
 147 |     """
 148 | 
 149 |     # int casting, and index precal trigs
 150 |     theta_index_ = int(theta_index)
 151 |     s = sines[theta_index_]
 152 |     c = cosines[theta_index_]
 153 | 
 154 |     # distance to nearest initialization
 155 |     dist_to_nearest = distance_transform(
 156 |         x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt
 157 |     )
 158 |     total_dist = dist_to_nearest
 159 | 
 160 |     # ray tracing iterations
 161 |     while dist_to_nearest > eps and total_dist <= max_range:
 162 |         # move in the direction of the ray by dist_to_nearest
 163 |         x += dist_to_nearest * c
 164 |         y += dist_to_nearest * s
 165 | 
 166 |         # update dist_to_nearest for current point on ray
 167 |         # also keeps track of total ray length
 168 |         dist_to_nearest = distance_transform(
 169 |             x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt
 170 |         )
 171 |         total_dist += dist_to_nearest
 172 | 
 173 |     if total_dist > max_range:
 174 |         total_dist = max_range
 175 | 
 176 |     return total_dist
 177 | 
 178 | 
 179 | @njit(cache=True)
 180 | def get_scan(
 181 |     pose,
 182 |     theta_dis,
 183 |     fov,
 184 |     num_beams,
 185 |     theta_index_increment,
 186 |     sines,
 187 |     cosines,
 188 |     eps,
 189 |     orig_x,
 190 |     orig_y,
 191 |     orig_c,
 192 |     orig_s,
 193 |     height,
 194 |     width,
 195 |     resolution,
 196 |     dt,
 197 |     max_range,
 198 | ):
 199 |     """
 200 |     Perform the scan for each discretized angle of each beam of the laser, loop heavy, should be JITted
 201 | 
 202 |         Args:
 203 |             pose (numpy.ndarray(3, )): current pose of the scan frame in the map
 204 |             theta_dis (int): number of steps to discretize the angles between 0 and 2pi for look up
 205 |             fov (float): field of view of the laser scan
 206 |             num_beams (int): number of beams in the scan
 207 |             theta_index_increment (float): increment between angle indices after discretization
 208 | 
 209 |         Returns:
 210 |             scan (numpy.ndarray(n, )): resulting laser scan at the pose, n=num_beams
 211 |     """
 212 |     # empty scan array init
 213 |     scan = np.empty((num_beams,))
 214 | 
 215 |     # make theta discrete by mapping the range [-pi, pi] onto [0, theta_dis]
 216 |     theta_index = theta_dis * (pose[2] - fov / 2.0) / (2.0 * np.pi)
 217 | 
 218 |     # make sure it's wrapped properly
 219 |     theta_index = np.fmod(theta_index, theta_dis)
 220 |     while theta_index < 0:
 221 |         theta_index += theta_dis
 222 | 
 223 |     # sweep through each beam
 224 |     for i in range(0, num_beams):
 225 |         # trace the current beam
 226 |         scan[i] = trace_ray(
 227 |             pose[0],
 228 |             pose[1],
 229 |             theta_index,
 230 |             sines,
 231 |             cosines,
 232 |             eps,
 233 |             orig_x,
 234 |             orig_y,
 235 |             orig_c,
 236 |             orig_s,
 237 |             height,
 238 |             width,
 239 |             resolution,
 240 |             dt,
 241 |             max_range,
 242 |         )
 243 | 
 244 |         # increment the beam index
 245 |         theta_index += theta_index_increment
 246 | 
 247 |         # make sure it stays in the range [0, theta_dis)
 248 |         while theta_index >= theta_dis:
 249 |             theta_index -= theta_dis
 250 | 
 251 |     return scan
 252 | 
 253 | 
 254 | @njit(cache=True, error_model="numpy")
 255 | def check_ttc_jit(scan, vel, scan_angles, cosines, side_distances, ttc_thresh):
 256 |     """
 257 |     Checks the iTTC of each beam in a scan for collision with environment
 258 | 
 259 |     Args:
 260 |         scan (np.ndarray(num_beams, )): current scan to check
 261 |         vel (float): current velocity
 262 |         scan_angles (np.ndarray(num_beams, )): precomped angles of each beam
 263 |         cosines (np.ndarray(num_beams, )): precomped cosines of the scan angles
 264 |         side_distances (np.ndarray(num_beams, )): precomped distances at each beam from the laser to the sides of the car
 265 |         ttc_thresh (float): threshold for iTTC for collision
 266 | 
 267 |     Returns:
 268 |         in_collision (bool): whether vehicle is in collision with environment
 269 |         collision_angle (float): at which angle the collision happened
 270 |     """
 271 |     in_collision = False
 272 |     if vel != 0.0:
 273 |         num_beams = scan.shape[0]
 274 |         for i in range(num_beams):
 275 |             proj_vel = vel * cosines[i]
 276 |             ttc = (scan[i] - side_distances[i]) / proj_vel
 277 |             if (ttc < ttc_thresh) and (ttc >= 0.0):
 278 |                 in_collision = True
 279 |                 break
 280 |     else:
 281 |         in_collision = False
 282 | 
 283 |     return in_collision
 284 | 
 285 | 
 286 | @njit(cache=True)
 287 | def cross(v1, v2):
 288 |     """
 289 |     Cross product of two 2-vectors
 290 | 
 291 |     Args:
 292 |         v1, v2 (np.ndarray(2, )): input vectors
 293 | 
 294 |     Returns:
 295 |         crossproduct (float): cross product
 296 |     """
 297 |     return v1[0] * v2[1] - v1[1] * v2[0]
 298 | 
 299 | 
 300 | @njit(cache=True)
 301 | def are_collinear(pt_a, pt_b, pt_c):
 302 |     """
 303 |     Checks if three points are collinear in 2D
 304 | 
 305 |     Args:
 306 |         pt_a, pt_b, pt_c (np.ndarray(2, )): points to check in 2D
 307 | 
 308 |     Returns:
 309 |         col (bool): whether three points are collinear
 310 |     """
 311 |     tol = 1e-8
 312 |     ba = pt_b - pt_a
 313 |     ca = pt_a - pt_c
 314 |     col = np.fabs(cross(ba, ca)) < tol
 315 |     return col
 316 | 
 317 | 
 318 | @njit(cache=True)
 319 | def get_range(pose, beam_theta, va, vb):
 320 |     """
 321 |     Get the distance at a beam angle to the vector formed by two of the four vertices of a vehicle
 322 | 
 323 |     Args:
 324 |         pose (np.ndarray(3, )): pose of the scanning vehicle
 325 |         beam_theta (float): angle of the current beam (world frame)
 326 |         va, vb (np.ndarray(2, )): the two vertices forming an edge
 327 | 
 328 |     Returns:
 329 |         distance (float): smallest distance at beam theta from scanning pose to edge
 330 |     """
 331 |     o = pose[0:2]
 332 |     v1 = o - va
 333 |     v2 = vb - va
 334 |     v3 = np.array([np.cos(beam_theta + np.pi / 2.0), np.sin(beam_theta + np.pi / 2.0)])
 335 | 
 336 |     denom = v2.dot(v3)
 337 |     distance = np.inf
 338 | 
 339 |     if np.fabs(denom) > 0.0:
 340 |         d1 = cross(v2, v1) / denom
 341 |         d2 = v1.dot(v3) / denom
 342 |         if d1 >= 0.0 and d2 >= 0.0 and d2 <= 1.0:
 343 |             distance = d1
 344 |     elif are_collinear(o, va, vb):
 345 |         da = np.linalg.norm(va - o)
 346 |         db = np.linalg.norm(vb - o)
 347 |         distance = min(da, db)
 348 | 
 349 |     return distance
 350 | 
 351 | 
 352 | @njit(cache=True)
 353 | def get_blocked_view_indices(pose, vertices, scan_angles):
 354 |     """
 355 |     Get the indices of the start and end of blocked fov in scans by another vehicle
 356 | 
 357 |     Args:
 358 |         pose (np.ndarray(3, )): pose of the scanning vehicle
 359 |         vertices (np.ndarray(4, 2)): four vertices of a vehicle pose
 360 |         scan_angles (np.ndarray(num_beams, )): corresponding beam angles
 361 |     """
 362 |     # find four vectors formed by pose and 4 vertices:
 363 |     vecs = vertices - pose[:2]
 364 |     vec_sq = np.square(vecs)
 365 |     norms = np.sqrt(vec_sq[:, 0] + vec_sq[:, 1])
 366 |     unit_vecs = vecs / norms.reshape(norms.shape[0], 1)
 367 | 
 368 |     # find angles between all four and pose vector
 369 |     ego_x_vec = np.array([[np.cos(pose[2])], [np.sin(pose[2])]])
 370 | 
 371 |     angles_with_x = np.empty((4,))
 372 |     for i in range(4):
 373 |         angle = np.arctan2(ego_x_vec[1], ego_x_vec[0]) - np.arctan2(
 374 |             unit_vecs[i, 1], unit_vecs[i, 0]
 375 |         )
 376 |         if angle > np.pi:
 377 |             angle = angle - 2 * np.pi
 378 |         elif angle < -np.pi:
 379 |             angle = angle + 2 * np.pi
 380 |         angles_with_x[i] = -angle[0]
 381 | 
 382 |     ind1 = int(np.argmin(np.abs(scan_angles - angles_with_x[0])))
 383 |     ind2 = int(np.argmin(np.abs(scan_angles - angles_with_x[1])))
 384 |     ind3 = int(np.argmin(np.abs(scan_angles - angles_with_x[2])))
 385 |     ind4 = int(np.argmin(np.abs(scan_angles - angles_with_x[3])))
 386 |     inds = [ind1, ind2, ind3, ind4]
 387 |     return min(inds), max(inds)
 388 | 
 389 | 
 390 | @njit(cache=True)
 391 | def ray_cast(pose, scan, scan_angles, vertices):
 392 |     """
 393 |     Modify a scan by ray casting onto another agent's four vertices
 394 | 
 395 |     Args:
 396 |         pose (np.ndarray(3, )): pose of the vehicle performing scan
 397 |         scan (np.ndarray(num_beams, )): original scan to modify
 398 |         scan_angles (np.ndarray(num_beams, )): corresponding beam angles
 399 |         vertices (np.ndarray(4, 2)): four vertices of a vehicle pose
 400 |     
 401 |     Returns:
 402 |         new_scan (np.ndarray(num_beams, )): modified scan
 403 |     """
 404 |     # pad vertices so loops around
 405 |     looped_vertices = np.empty((5, 2))
 406 |     looped_vertices[0:4, :] = vertices
 407 |     looped_vertices[4, :] = vertices[0, :]
 408 | 
 409 |     min_ind, max_ind = get_blocked_view_indices(pose, vertices, scan_angles)
 410 |     # looping over beams
 411 |     for i in range(min_ind, max_ind + 1):
 412 |         # looping over vertices
 413 |         for j in range(4):
 414 |             # check if original scan is longer than ray casted distance
 415 |             scan_range = get_range(
 416 |                 pose,
 417 |                 pose[2] + scan_angles[i],
 418 |                 looped_vertices[j, :],
 419 |                 looped_vertices[j + 1, :],
 420 |             )
 421 |             if scan_range < scan[i]:
 422 |                 scan[i] = scan_range
 423 |     return scan
 424 | 
 425 | 
 426 | class ScanSimulator2D(object):
 427 |     """
 428 |     2D LIDAR scan simulator class
 429 | 
 430 |     Init params:
 431 |         num_beams (int): number of beams in the scan
 432 |         fov (float): field of view of the laser scan
 433 |         eps (float, default=0.0001): ray tracing iteration termination condition
 434 |         theta_dis (int, default=2000): number of steps to discretize the angles between 0 and 2pi for look up
 435 |         max_range (float, default=30.0): maximum range of the laser
 436 |     """
 437 | 
 438 |     def __init__(self, num_beams, fov, eps=0.0001, theta_dis=2000, max_range=30.0):
 439 |         # initialization
 440 |         self.num_beams = num_beams
 441 |         self.fov = fov
 442 |         self.eps = eps
 443 |         self.theta_dis = theta_dis
 444 |         self.max_range = max_range
 445 |         self.angle_increment = self.fov / (self.num_beams - 1)
 446 |         self.theta_index_increment = theta_dis * self.angle_increment / (2.0 * np.pi)
 447 |         self.orig_c = None
 448 |         self.orig_s = None
 449 |         self.orig_x = None
 450 |         self.orig_y = None
 451 |         self.map_height = None
 452 |         self.map_width = None
 453 |         self.map_resolution = None
 454 |         self.dt = None
 455 | 
 456 |         # precomputing corresponding cosines and sines of the angle array
 457 |         theta_arr = np.linspace(0.0, 2 * np.pi, num=theta_dis)
 458 |         self.sines = np.sin(theta_arr)
 459 |         self.cosines = np.cos(theta_arr)
 460 | 
 461 |     def set_map(self, map_path, map_ext):
 462 |         """
 463 |         Set the bitmap of the scan simulator by path
 464 | 
 465 |             Args:
 466 |                 map_path (str): path to the map yaml file
 467 |                 map_ext (str): extension (image type) of the map image
 468 | 
 469 |             Returns:
 470 |                 flag (bool): if image reading and loading is successful
 471 |         """
 472 |         # TODO: do we open the option to flip the images, and turn rgb into grayscale? or specify the exact requirements in documentation.
 473 |         # TODO: throw error if image specification isn't met
 474 | 
 475 |         # load map image
 476 |         map_img_path = os.path.splitext(map_path)[0] + map_ext
 477 |         self.map_img = np.array(
 478 |             Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM)
 479 |         )
 480 |         self.map_img = self.map_img.astype(np.float64)
 481 | 
 482 |         # grayscale -> binary
 483 |         self.map_img[self.map_img <= 128.0] = 0.0
 484 |         self.map_img[self.map_img > 128.0] = 255.0
 485 | 
 486 |         self.map_height = self.map_img.shape[0]
 487 |         self.map_width = self.map_img.shape[1]
 488 | 
 489 |         # load map yaml
 490 |         with open(map_path, "r") as yaml_stream:
 491 |             try:
 492 |                 map_metadata = yaml.safe_load(yaml_stream)
 493 |                 self.map_resolution = map_metadata["resolution"]
 494 |                 self.origin = map_metadata["origin"]
 495 |             except yaml.YAMLError as ex:
 496 |                 print(ex)
 497 | 
 498 |         # calculate map parameters
 499 |         self.orig_x = self.origin[0]
 500 |         self.orig_y = self.origin[1]
 501 |         self.orig_s = np.sin(self.origin[2])
 502 |         self.orig_c = np.cos(self.origin[2])
 503 | 
 504 |         # get the distance transform
 505 |         self.dt = get_dt(self.map_img, self.map_resolution)
 506 | 
 507 |         return True
 508 | 
 509 |     def scan(self, pose, rng, std_dev=0.01):
 510 |         """
 511 |         Perform simulated 2D scan by pose on the given map
 512 | 
 513 |             Args:
 514 |                 pose (numpy.ndarray (3, )): pose of the scan frame (x, y, theta)
 515 |                 rng (numpy.random.Generator): random number generator to use for whitenoise in scan, or None
 516 |                 std_dev (float, default=0.01): standard deviation of the generated whitenoise in the scan
 517 | 
 518 |             Returns:
 519 |                 scan (numpy.ndarray (n, )): data array of the laserscan, n=num_beams
 520 | 
 521 |             Raises:
 522 |                 ValueError: when scan is called before a map is set
 523 |         """
 524 | 
 525 |         if self.map_height is None:
 526 |             raise ValueError("Map is not set for scan simulator.")
 527 | 
 528 |         scan = get_scan(
 529 |             pose,
 530 |             self.theta_dis,
 531 |             self.fov,
 532 |             self.num_beams,
 533 |             self.theta_index_increment,
 534 |             self.sines,
 535 |             self.cosines,
 536 |             self.eps,
 537 |             self.orig_x,
 538 |             self.orig_y,
 539 |             self.orig_c,
 540 |             self.orig_s,
 541 |             self.map_height,
 542 |             self.map_width,
 543 |             self.map_resolution,
 544 |             self.dt,
 545 |             self.max_range,
 546 |         )
 547 | 
 548 |         if rng is not None:
 549 |             noise = rng.normal(0.0, std_dev, size=self.num_beams)
 550 |             scan += noise
 551 | 
 552 |         return scan
 553 | 
 554 |     def get_increment(self):
 555 |         return self.angle_increment
 556 | 
 557 | 
 558 | """
 559 | Unit tests for the 2D scan simulator class
 560 | Author: Hongrui Zheng
 561 | 
 562 | Test cases:
 563 |     1, 2: Comparison between generated scan array of the new simulator and the legacy C++ simulator, generated data used, MSE is used as the metric
 564 |     2. FPS test, should be greater than 500
 565 | """
 566 | 
 567 | 
 568 | class ScanTests(unittest.TestCase):
 569 |     def setUp(self):
 570 |         # test params
 571 |         self.num_beams = 1080
 572 |         self.fov = 4.7
 573 | 
 574 |         self.num_test = 10
 575 |         self.test_poses = np.zeros((self.num_test, 3))
 576 |         self.test_poses[:, 2] = np.linspace(-1.0, 1.0, num=self.num_test)
 577 | 
 578 |         # # legacy gym data
 579 |         # sample_scan = np.load('legacy_scan.npz')
 580 |         # self.berlin_scan = sample_scan['berlin']
 581 |         # self.skirk_scan = sample_scan['skirk']
 582 | 
 583 |     # def test_map_berlin(self):
 584 |     #     scan_rng = np.random.default_rng(seed=12345)
 585 |     #     scan_sim = ScanSimulator2D(self.num_beams, self.fov)
 586 |     #     new_berlin = np.empty((self.num_test, self.num_beams))
 587 |     #     map_path = '../../../maps/berlin.yaml'
 588 |     #     map_ext = '.png'
 589 |     #     scan_sim.set_map(map_path, map_ext)
 590 |     #     # scan gen loop
 591 |     #     for i in range(self.num_test):
 592 |     #         test_pose = self.test_poses[i]
 593 |     #         new_berlin[i,:] = scan_sim.scan(test_pose, scan_rng)
 594 |     #     diff = self.berlin_scan - new_berlin
 595 |     #     mse = np.mean(diff**2)
 596 |     #     # print('Levine distance test, norm: ' + str(norm))
 597 | 
 598 |     #     # plotting
 599 |     #     import matplotlib.pyplot as plt
 600 |     #     theta = np.linspace(-self.fov/2., self.fov/2., num=self.num_beams)
 601 |     #     plt.polar(theta, new_berlin[1,:], '.', lw=0)
 602 |     #     plt.polar(theta, self.berlin_scan[1,:], '.', lw=0)
 603 |     #     plt.show()
 604 | 
 605 |     #     self.assertLess(mse, 2.)
 606 | 
 607 |     # def test_map_skirk(self):
 608 |     #     scan_rng = np.random.default_rng(seed=12345)
 609 |     #     scan_sim = ScanSimulator2D(self.num_beams, self.fov)
 610 |     #     new_skirk = np.empty((self.num_test, self.num_beams))
 611 |     #     map_path = '../../../maps/skirk.yaml'
 612 |     #     map_ext = '.png'
 613 |     #     scan_sim.set_map(map_path, map_ext)
 614 |     #     print('map set')
 615 |     #     # scan gen loop
 616 |     #     for i in range(self.num_test):
 617 |     #         test_pose = self.test_poses[i]
 618 |     #         new_skirk[i,:] = scan_sim.scan(test_pose, scan_rng)
 619 |     #     diff = self.skirk_scan - new_skirk
 620 |     #     mse = np.mean(diff**2)
 621 |     #     print('skirk distance test, mse: ' + str(mse))
 622 | 
 623 |     #     # plotting
 624 |     #     import matplotlib.pyplot as plt
 625 |     #     theta = np.linspace(-self.fov/2., self.fov/2., num=self.num_beams)
 626 |     #     plt.polar(theta, new_skirk[1,:], '.', lw=0)
 627 |     #     plt.polar(theta, self.skirk_scan[1,:], '.', lw=0)
 628 |     #     plt.show()
 629 | 
 630 |     #     self.assertLess(mse, 2.)
 631 | 
 632 |     def test_fps(self):
 633 |         # scan fps should be greater than 500
 634 | 
 635 |         scan_rng = np.random.default_rng(seed=12345)
 636 |         scan_sim = ScanSimulator2D(self.num_beams, self.fov)
 637 |         map_path = "../envs/maps/berlin.yaml"
 638 |         map_ext = ".png"
 639 |         scan_sim.set_map(map_path, map_ext)
 640 | 
 641 |         import time
 642 | 
 643 |         start = time.time()
 644 |         for i in range(10000):
 645 |             x_test = i / 10000
 646 |             scan = scan_sim.scan(np.array([x_test, 0.0, 0.0]), scan_rng)
 647 |         end = time.time()
 648 |         fps = 10000 / (end - start)
 649 |         # print('FPS test')
 650 |         # print('Elapsed time: ' + str(end-start) + ' , FPS: ' + str(1/fps))
 651 |         self.assertGreater(fps, 500.0)
 652 | 
 653 |     def test_rng(self):
 654 |         num_beams = 1080
 655 |         fov = 4.7
 656 |         map_path = "../envs/maps/berlin.yaml"
 657 |         map_ext = ".png"
 658 |         it = 100
 659 | 
 660 |         scan_rng = np.random.default_rng(seed=12345)
 661 |         scan_sim = ScanSimulator2D(num_beams, fov)
 662 |         scan_sim.set_map(map_path, map_ext)
 663 |         scan1 = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 664 |         scan2 = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 665 |         for i in range(it):
 666 |             scan3 = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 667 |         scan4 = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 668 | 
 669 |         scan_rng = np.random.default_rng(seed=12345)
 670 |         scan5 = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 671 |         scan2 = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 672 |         for i in range(it):
 673 |             _ = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 674 |         scan6 = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 675 | 
 676 |         self.assertTrue(np.allclose(scan1, scan5))
 677 |         self.assertFalse(np.allclose(scan1, scan2))
 678 |         self.assertFalse(np.allclose(scan1, scan3))
 679 |         self.assertTrue(np.allclose(scan4, scan6))
 680 | 
 681 | 
 682 | def main():
 683 |     num_beams = 1080
 684 |     fov = 4.7
 685 |     # map_path = '../envs/maps/berlin.yaml'
 686 |     map_path = "../../../examples/example_map.yaml"
 687 |     map_ext = ".png"
 688 |     scan_rng = np.random.default_rng(seed=12345)
 689 |     scan_sim = ScanSimulator2D(num_beams, fov)
 690 |     scan_sim.set_map(map_path, map_ext)
 691 |     scan = scan_sim.scan(np.array([0.0, 0.0, 0.0]), scan_rng)
 692 | 
 693 |     # fps test
 694 |     import time
 695 | 
 696 |     start = time.time()
 697 |     for i in range(10000):
 698 |         x_test = i / 10000
 699 |         scan = scan_sim.scan(np.array([x_test, 0.0, 0.0]), scan_rng)
 700 |     end = time.time()
 701 |     fps = (end - start) / 10000
 702 |     print("FPS test")
 703 |     print("Elapsed time: " + str(end - start) + " , FPS: " + str(1 / fps))
 704 | 
 705 |     # visualization
 706 |     import matplotlib.pyplot as plt
 707 |     from matplotlib.animation import FuncAnimation
 708 | 
 709 |     num_iter = 100
 710 |     theta = np.linspace(-fov / 2.0, fov / 2.0, num=num_beams)
 711 |     fig = plt.figure()
 712 |     ax = fig.add_subplot(111, projection="polar")
 713 |     ax.set_ylim(0, 31)
 714 |     line, = ax.plot([], [], ".", lw=0)
 715 | 
 716 |     def update(i):
 717 |         # x_ani = i * 3. / num_iter
 718 |         theta_ani = -i * 2 * np.pi / num_iter
 719 |         x_ani = 0.0
 720 |         current_scan = scan_sim.scan(np.array([x_ani, 0.0, theta_ani]), scan_rng)
 721 |         print(np.max(current_scan))
 722 |         line.set_data(theta, current_scan)
 723 |         return (line,)
 724 | 
 725 |     ani = FuncAnimation(fig, update, frames=num_iter, blit=True)
 726 |     plt.show()
 727 | 
 728 | 
 729 | if __name__ == "__main__":
 730 |     unittest.main()
 731 |     # main()
 732 | 
 733 |     # import time
 734 |     # pt_a = np.array([1., 1.])
 735 |     # pt_b = np.array([1., 2.])
 736 |     # pt_c = np.array([1., 3.])
 737 |     # col = are_collinear(pt_a, pt_b, pt_c)
 738 |     # print(col)
 739 | 
 740 |     # pose = np.array([0., 0., -1.])
 741 |     # beam_theta = 0.
 742 |     # start = time.time()
 743 |     # dist = get_range(pose, beam_theta, pt_a, pt_b)
 744 |     # print(dist, time.time()-start)
 745 | 
 746 |     # num_beams = 1080
 747 |     # scan = 100.*np.ones((num_beams, ))
 748 |     # scan_angles = np.linspace(-2.35, 2.35, num=num_beams)
 749 |     # assert scan.shape[0] == scan_angles.shape[0]
 750 |     # vertices = np.asarray([[4,11.],[5,5],[9,9],[10,10]])
 751 |     # start = time.time()
 752 |     # new_scan = ray_cast(pose, scan, scan_angles, vertices)
 753 |     # print(time.time()-start)

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\maps\berlin.yaml`:

```yaml
   1 | image: berlin.png
   2 | resolution: 0.050000
   3 | origin: [-11.606540, -26.520793, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196
   7 | 

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\maps\levine.yaml`:

```yaml
   1 | image: levine.png
   2 | resolution: 0.050000
   3 | origin: [-51.224998, -51.224998, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196
   7 | 

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\maps\skirk.yaml`:

```yaml
   1 | image: skirk.png
   2 | resolution: 0.050000
   3 | origin: [-7.801, -16.388, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196
   7 | 

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\maps\stata_basement.yaml`:

```yaml
   1 | image: stata_basement.png
   2 | resolution: 0.0504
   3 | origin: [-26.900000, -16.50000, 0.0]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\maps\vegas.yaml`:

```yaml
   1 | image: vegas.png
   2 | resolution: 0.050000
   3 | origin: [-11.606540, -27.320793, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196
   7 | 

```

`src\simulator\f1tenth_gym\gym\f110_gym\envs\rendering.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Rendering engine for f1tenth gym env based on pyglet and OpenGL
  26 | Author: Hongrui Zheng
  27 | """
  28 | 
  29 | # opengl stuff
  30 | import pyglet
  31 | from pyglet.gl import *
  32 | 
  33 | # other
  34 | import numpy as np
  35 | from PIL import Image
  36 | import yaml
  37 | 
  38 | # helpers
  39 | from f110_gym.envs.collision_models import get_vertices
  40 | 
  41 | # zooming constants
  42 | ZOOM_IN_FACTOR = 1.2
  43 | ZOOM_OUT_FACTOR = 1 / ZOOM_IN_FACTOR
  44 | 
  45 | # vehicle shape constants
  46 | CAR_LENGTH = 0.58
  47 | CAR_WIDTH = 0.31
  48 | 
  49 | 
  50 | class EnvRenderer(pyglet.window.Window):
  51 |     """
  52 |     A window class inherited from pyglet.window.Window, handles the camera/projection interaction, resizing window, and rendering the environment
  53 |     """
  54 | 
  55 |     def __init__(self, width, height, *args, **kwargs):
  56 |         """
  57 |         Class constructor
  58 | 
  59 |         Args:
  60 |             width (int): width of the window
  61 |             height (int): height of the window
  62 | 
  63 |         Returns:
  64 |             None
  65 |         """
  66 |         conf = Config(sample_buffers=1, samples=4, depth_size=16, double_buffer=True)
  67 |         super().__init__(
  68 |             width, height, config=conf, resizable=True, vsync=False, *args, **kwargs
  69 |         )
  70 | 
  71 |         # gl init
  72 |         glClearColor(9 / 255, 32 / 255, 87 / 255, 1.0)
  73 | 
  74 |         # initialize camera values
  75 |         self.left = -width / 2
  76 |         self.right = width / 2
  77 |         self.bottom = -height / 2
  78 |         self.top = height / 2
  79 |         self.zoom_level = 1.2
  80 |         self.zoomed_width = width
  81 |         self.zoomed_height = height
  82 | 
  83 |         # current batch that keeps track of all graphics
  84 |         self.batch = pyglet.graphics.Batch()
  85 | 
  86 |         # current env map
  87 |         self.map_points = None
  88 | 
  89 |         # current env agent poses, (num_agents, 3), columns are (x, y, theta)
  90 |         self.poses = None
  91 | 
  92 |         # current env agent vertices, (num_agents, 4, 2), 2nd and 3rd dimensions are the 4 corners in 2D
  93 |         self.vertices = None
  94 | 
  95 |         # current score label
  96 |         self.score_label = pyglet.text.Label(
  97 |             "Lap Time: {laptime:.2f}, Ego Lap Count: {count:.0f}".format(
  98 |                 laptime=0.0, count=0.0
  99 |             ),
 100 |             font_size=36,
 101 |             x=0,
 102 |             y=-800,
 103 |             anchor_x="center",
 104 |             anchor_y="center",
 105 |             # width=0.01,
 106 |             # height=0.01,
 107 |             color=(255, 255, 255, 255),
 108 |             batch=self.batch,
 109 |         )
 110 | 
 111 |         self.fps_display = pyglet.window.FPSDisplay(self)
 112 | 
 113 |     def update_map(self, map_path, map_ext):
 114 |         """
 115 |         Update the map being drawn by the renderer. Converts image to a list of 3D points representing each obstacle pixel in the map.
 116 | 
 117 |         Args:
 118 |             map_path (str): absolute path to the map without extensions
 119 |             map_ext (str): extension for the map image file
 120 | 
 121 |         Returns:
 122 |             None
 123 |         """
 124 | 
 125 |         # load map metadata
 126 |         with open(map_path + ".yaml", "r") as yaml_stream:
 127 |             try:
 128 |                 map_metadata = yaml.safe_load(yaml_stream)
 129 |                 map_resolution = map_metadata["resolution"]
 130 |                 origin = map_metadata["origin"]
 131 |                 origin_x = origin[0]
 132 |                 origin_y = origin[1]
 133 |             except yaml.YAMLError as ex:
 134 |                 print(ex)
 135 | 
 136 |         # load map image
 137 |         map_img = np.array(
 138 |             Image.open(map_path + map_ext).transpose(Image.FLIP_TOP_BOTTOM)
 139 |         ).astype(np.float64)
 140 |         map_height = map_img.shape[0]
 141 |         map_width = map_img.shape[1]
 142 | 
 143 |         # convert map pixels to coordinates
 144 |         range_x = np.arange(map_width)
 145 |         range_y = np.arange(map_height)
 146 |         map_x, map_y = np.meshgrid(range_x, range_y)
 147 |         map_x = (map_x * map_resolution + origin_x).flatten()
 148 |         map_y = (map_y * map_resolution + origin_y).flatten()
 149 |         map_z = np.zeros(map_y.shape)
 150 |         map_coords = np.vstack((map_x, map_y, map_z))
 151 | 
 152 |         # mask and only leave the obstacle points
 153 |         map_mask = map_img == 0.0
 154 |         map_mask_flat = map_mask.flatten()
 155 |         map_points = 50.0 * map_coords[:, map_mask_flat].T
 156 |         for i in range(map_points.shape[0]):
 157 |             self.batch.add(
 158 |                 1,
 159 |                 GL_POINTS,
 160 |                 None,
 161 |                 ("v3f/stream", [map_points[i, 0], map_points[i, 1], map_points[i, 2]]),
 162 |                 ("c3B/stream", [183, 193, 222]),
 163 |             )
 164 |         self.map_points = map_points
 165 | 
 166 |     def on_resize(self, width, height):
 167 |         """
 168 |         Callback function on window resize, overrides inherited method, and updates camera values on top of the inherited on_resize() method.
 169 | 
 170 |         Potential improvements on current behavior: zoom/pan resets on window resize.
 171 | 
 172 |         Args:
 173 |             width (int): new width of window
 174 |             height (int): new height of window
 175 | 
 176 |         Returns:
 177 |             None
 178 |         """
 179 | 
 180 |         # call overrided function
 181 |         super().on_resize(width, height)
 182 | 
 183 |         # update camera value
 184 |         (width, height) = self.get_size()
 185 |         self.left = -self.zoom_level * width / 2
 186 |         self.right = self.zoom_level * width / 2
 187 |         self.bottom = -self.zoom_level * height / 2
 188 |         self.top = self.zoom_level * height / 2
 189 |         self.zoomed_width = self.zoom_level * width
 190 |         self.zoomed_height = self.zoom_level * height
 191 | 
 192 |     def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
 193 |         """
 194 |         Callback function on mouse drag, overrides inherited method.
 195 | 
 196 |         Args:
 197 |             x (int): Distance in pixels from the left edge of the window.
 198 |             y (int): Distance in pixels from the bottom edge of the window.
 199 |             dx (int): Relative X position from the previous mouse position.
 200 |             dy (int): Relative Y position from the previous mouse position.
 201 |             buttons (int): Bitwise combination of the mouse buttons currently pressed.
 202 |             modifiers (int): Bitwise combination of any keyboard modifiers currently active.
 203 | 
 204 |         Returns:
 205 |             None
 206 |         """
 207 | 
 208 |         # pan camera
 209 |         self.left -= dx * self.zoom_level
 210 |         self.right -= dx * self.zoom_level
 211 |         self.bottom -= dy * self.zoom_level
 212 |         self.top -= dy * self.zoom_level
 213 | 
 214 |     def on_mouse_scroll(self, x, y, dx, dy):
 215 |         """
 216 |         Callback function on mouse scroll, overrides inherited method.
 217 | 
 218 |         Args:
 219 |             x (int): Distance in pixels from the left edge of the window.
 220 |             y (int): Distance in pixels from the bottom edge of the window.
 221 |             scroll_x (float): Amount of movement on the horizontal axis.
 222 |             scroll_y (float): Amount of movement on the vertical axis.
 223 | 
 224 |         Returns:
 225 |             None
 226 |         """
 227 | 
 228 |         # Get scale factor
 229 |         f = ZOOM_IN_FACTOR if dy > 0 else ZOOM_OUT_FACTOR if dy < 0 else 1
 230 | 
 231 |         # If zoom_level is in the proper range
 232 |         if 0.01 < self.zoom_level * f < 10:
 233 | 
 234 |             self.zoom_level *= f
 235 | 
 236 |             (width, height) = self.get_size()
 237 | 
 238 |             mouse_x = x / width
 239 |             mouse_y = y / height
 240 | 
 241 |             mouse_x_in_world = self.left + mouse_x * self.zoomed_width
 242 |             mouse_y_in_world = self.bottom + mouse_y * self.zoomed_height
 243 | 
 244 |             self.zoomed_width *= f
 245 |             self.zoomed_height *= f
 246 | 
 247 |             self.left = mouse_x_in_world - mouse_x * self.zoomed_width
 248 |             self.right = mouse_x_in_world + (1 - mouse_x) * self.zoomed_width
 249 |             self.bottom = mouse_y_in_world - mouse_y * self.zoomed_height
 250 |             self.top = mouse_y_in_world + (1 - mouse_y) * self.zoomed_height
 251 | 
 252 |     def on_close(self):
 253 |         """
 254 |         Callback function when the 'x' is clicked on the window, overrides inherited method. Also throws exception to end the python program when in a loop.
 255 | 
 256 |         Args:
 257 |             None
 258 | 
 259 |         Returns:
 260 |             None
 261 | 
 262 |         Raises:
 263 |             Exception: with a message that indicates the rendering window was closed
 264 |         """
 265 | 
 266 |         super().on_close()
 267 |         raise Exception("Rendering window was closed.")
 268 | 
 269 |     def on_draw(self):
 270 |         """
 271 |         Function when the pyglet is drawing. The function draws the batch created that includes the map points, the agent polygons, and the information text, and the fps display.
 272 |         
 273 |         Args:
 274 |             None
 275 | 
 276 |         Returns:
 277 |             None
 278 |         """
 279 | 
 280 |         # if map and poses doesn't exist, raise exception
 281 |         if self.map_points is None:
 282 |             raise Exception("Map not set for renderer.")
 283 |         if self.poses is None:
 284 |             raise Exception("Agent poses not updated for renderer.")
 285 | 
 286 |         # Initialize Projection matrix
 287 |         glMatrixMode(GL_PROJECTION)
 288 |         glLoadIdentity()
 289 | 
 290 |         # Initialize Modelview matrix
 291 |         glMatrixMode(GL_MODELVIEW)
 292 |         glLoadIdentity()
 293 |         # Save the default modelview matrix
 294 |         glPushMatrix()
 295 | 
 296 |         # Clear window with ClearColor
 297 |         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
 298 | 
 299 |         # Set orthographic projection matrix
 300 |         glOrtho(self.left, self.right, self.bottom, self.top, 1, -1)
 301 | 
 302 |         # Draw all batches
 303 |         self.batch.draw()
 304 |         self.fps_display.draw()
 305 |         # Remove default modelview matrix
 306 |         glPopMatrix()
 307 | 
 308 |     def update_obs(self, obs):
 309 |         """
 310 |         Updates the renderer with the latest observation from the gym environment, including the agent poses, and the information text.
 311 | 
 312 |         Args:
 313 |             obs (dict): observation dict from the gym env
 314 | 
 315 |         Returns:
 316 |             None
 317 |         """
 318 | 
 319 |         self.ego_idx = obs["ego_idx"]
 320 |         poses_x = obs["poses_x"]
 321 |         poses_y = obs["poses_y"]
 322 |         poses_theta = obs["poses_theta"]
 323 | 
 324 |         num_agents = len(poses_x)
 325 |         if self.poses is None:
 326 |             self.cars = []
 327 |             for i in range(num_agents):
 328 |                 if i == self.ego_idx:
 329 |                     vertices_np = get_vertices(
 330 |                         np.array([0.0, 0.0, 0.0]), CAR_LENGTH, CAR_WIDTH
 331 |                     )
 332 |                     vertices = list(vertices_np.flatten())
 333 |                     car = self.batch.add(
 334 |                         4,
 335 |                         GL_QUADS,
 336 |                         None,
 337 |                         ("v2f", vertices),
 338 |                         (
 339 |                             "c3B",
 340 |                             [172, 97, 185, 172, 97, 185, 172, 97, 185, 172, 97, 185],
 341 |                         ),
 342 |                     )
 343 |                     self.cars.append(car)
 344 |                 else:
 345 |                     vertices_np = get_vertices(
 346 |                         np.array([0.0, 0.0, 0.0]), CAR_LENGTH, CAR_WIDTH
 347 |                     )
 348 |                     vertices = list(vertices_np.flatten())
 349 |                     car = self.batch.add(
 350 |                         4,
 351 |                         GL_QUADS,
 352 |                         None,
 353 |                         ("v2f", vertices),
 354 |                         ("c3B", [99, 52, 94, 99, 52, 94, 99, 52, 94, 99, 52, 94]),
 355 |                     )
 356 |                     self.cars.append(car)
 357 | 
 358 |         poses = np.stack((poses_x, poses_y, poses_theta)).T
 359 |         for j in range(poses.shape[0]):
 360 |             vertices_np = 50.0 * get_vertices(poses[j, :], CAR_LENGTH, CAR_WIDTH)
 361 |             vertices = list(vertices_np.flatten())
 362 |             self.cars[j].vertices = vertices
 363 |         self.poses = poses
 364 | 
 365 |         self.score_label.text = "Lap Time: {laptime:.2f}, Ego Lap Count: {count:.0f}".format(
 366 |             laptime=obs["lap_times"][0], count=obs["lap_counts"][obs["ego_idx"]]
 367 |         )

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\__init__.py`:

```py
   1 | from gym.envs.unittest.scan_sim import *

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\collision_checks.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Prototype of Utility functions and GJK algorithm for Collision checks between vehicles
  26 | Originally from https://github.com/kroitor/gjk.c
  27 | Author: Hongrui Zheng
  28 | """
  29 | 
  30 | import numpy as np
  31 | from numba import njit
  32 | 
  33 | 
  34 | @njit(cache=True)
  35 | def perpendicular(pt):
  36 |     """
  37 |     Return a 2-vector's perpendicular vector
  38 | 
  39 |     Args:
  40 |         pt (np.ndarray, (2,)): input vector
  41 | 
  42 |     Returns:
  43 |         pt (np.ndarray, (2,)): perpendicular vector
  44 |     """
  45 |     temp = pt[0]
  46 |     pt[0] = pt[1]
  47 |     pt[1] = -1 * temp
  48 |     return pt
  49 | 
  50 | 
  51 | @njit(cache=True)
  52 | def tripleProduct(a, b, c):
  53 |     """
  54 |     Return triple product of three vectors
  55 | 
  56 |     Args:
  57 |         a, b, c (np.ndarray, (2,)): input vectors
  58 | 
  59 |     Returns:
  60 |         (np.ndarray, (2,)): triple product
  61 |     """
  62 |     ac = a.dot(c)
  63 |     bc = b.dot(c)
  64 |     return b * ac - a * bc
  65 | 
  66 | 
  67 | @njit(cache=True)
  68 | def avgPoint(vertices):
  69 |     """
  70 |     Return the average point of multiple vertices
  71 | 
  72 |     Args:
  73 |         vertices (np.ndarray, (n, 2)): the vertices we want to find avg on
  74 | 
  75 |     Returns:
  76 |         avg (np.ndarray, (2,)): average point of the vertices
  77 |     """
  78 |     return np.sum(vertices, axis=0) / vertices.shape[0]
  79 | 
  80 | 
  81 | @njit(cache=True)
  82 | def indexOfFurthestPoint(vertices, d):
  83 |     """
  84 |     Return the index of the vertex furthest away along a direction in the list of vertices
  85 | 
  86 |     Args:
  87 |         vertices (np.ndarray, (n, 2)): the vertices we want to find avg on
  88 | 
  89 |     Returns:
  90 |         idx (int): index of the furthest point
  91 |     """
  92 |     return np.argmax(vertices.dot(d))
  93 | 
  94 | 
  95 | @njit(cache=True)
  96 | def support(vertices1, vertices2, d):
  97 |     """
  98 |     Minkowski sum support function for GJK
  99 | 
 100 |     Args:
 101 |         vertices1 (np.ndarray, (n, 2)): vertices of the first body
 102 |         vertices2 (np.ndarray, (n, 2)): vertices of the second body
 103 |         d (np.ndarray, (2, )): direction to find the support along
 104 | 
 105 |     Returns:
 106 |         support (np.ndarray, (n, 2)): Minkowski sum
 107 |     """
 108 |     i = indexOfFurthestPoint(vertices1, d)
 109 |     j = indexOfFurthestPoint(vertices2, -d)
 110 |     return vertices1[i] - vertices2[j]
 111 | 
 112 | 
 113 | @njit(cache=True)
 114 | def collision(vertices1, vertices2):
 115 |     """
 116 |     GJK test to see whether two bodies overlap
 117 | 
 118 |     Args:
 119 |         vertices1 (np.ndarray, (n, 2)): vertices of the first body
 120 |         vertices2 (np.ndarray, (n, 2)): vertices of the second body
 121 | 
 122 |     Returns:
 123 |         overlap (boolean): True if two bodies collide
 124 |     """
 125 |     index = 0
 126 |     simplex = np.empty((3, 2))
 127 | 
 128 |     position1 = avgPoint(vertices1)
 129 |     position2 = avgPoint(vertices2)
 130 | 
 131 |     d = position1 - position2
 132 | 
 133 |     if d[0] == 0 and d[1] == 0:
 134 |         d[0] = 1.0
 135 | 
 136 |     a = support(vertices1, vertices2, d)
 137 |     simplex[index, :] = a
 138 | 
 139 |     if d.dot(a) <= 0:
 140 |         return False
 141 | 
 142 |     d = -a
 143 | 
 144 |     iter_count = 0
 145 |     while iter_count < 1e3:
 146 |         a = support(vertices1, vertices2, d)
 147 |         index += 1
 148 |         simplex[index, :] = a
 149 |         if d.dot(a) <= 0:
 150 |             return False
 151 | 
 152 |         ao = -a
 153 | 
 154 |         if index < 2:
 155 |             b = simplex[0, :]
 156 |             ab = b - a
 157 |             d = tripleProduct(ab, ao, ab)
 158 |             if np.linalg.norm(d) < 1e-10:
 159 |                 d = perpendicular(ab)
 160 |             continue
 161 | 
 162 |         b = simplex[1, :]
 163 |         c = simplex[0, :]
 164 |         ab = b - a
 165 |         ac = c - a
 166 | 
 167 |         acperp = tripleProduct(ab, ac, ac)
 168 | 
 169 |         if acperp.dot(ao) >= 0:
 170 |             d = acperp
 171 |         else:
 172 |             abperp = tripleProduct(ac, ab, ab)
 173 |             if abperp.dot(ao) < 0:
 174 |                 return True
 175 |             simplex[0, :] = simplex[1, :]
 176 |             d = abperp
 177 | 
 178 |         simplex[1, :] = simplex[2, :]
 179 |         index -= 1
 180 | 
 181 |         iter_count += 1
 182 |     return False
 183 | 
 184 | 
 185 | """
 186 | Utility functions for getting vertices by pose and shape
 187 | """
 188 | 
 189 | 
 190 | @njit(cache=True)
 191 | def get_trmtx(pose):
 192 |     """
 193 |     Get transformation matrix of vehicle frame -> global frame
 194 | 
 195 |     Args:
 196 |         pose (np.ndarray (3, )): current pose of the vehicle
 197 | 
 198 |     return:
 199 |         H (np.ndarray (4, 4)): transformation matrix
 200 |     """
 201 |     x = pose[0]
 202 |     y = pose[1]
 203 |     th = pose[2]
 204 |     cos = np.cos(th)
 205 |     sin = np.sin(th)
 206 |     H = np.array(
 207 |         [
 208 |             [cos, -sin, 0.0, x],
 209 |             [sin, cos, 0.0, y],
 210 |             [0.0, 0.0, 1.0, 0.0],
 211 |             [0.0, 0.0, 0.0, 1.0],
 212 |         ]
 213 |     )
 214 |     return H
 215 | 
 216 | 
 217 | @njit(cache=True)
 218 | def get_vertices(pose, length, width):
 219 |     """
 220 |     Utility function to return vertices of the car body given pose and size
 221 | 
 222 |     Args:
 223 |         pose (np.ndarray, (3, )): current world coordinate pose of the vehicle
 224 |         length (float): car length
 225 |         width (float): car width
 226 | 
 227 |     Returns:
 228 |         vertices (np.ndarray, (4, 2)): corner vertices of the vehicle body
 229 |     """
 230 |     H = get_trmtx(pose)
 231 |     rl = H.dot(np.asarray([[-length / 2], [width / 2], [0.0], [1.0]])).flatten()
 232 |     rr = H.dot(np.asarray([[-length / 2], [-width / 2], [0.0], [1.0]])).flatten()
 233 |     fl = H.dot(np.asarray([[length / 2], [width / 2], [0.0], [1.0]])).flatten()
 234 |     fr = H.dot(np.asarray([[length / 2], [-width / 2], [0.0], [1.0]])).flatten()
 235 |     rl = rl / rl[3]
 236 |     rr = rr / rr[3]
 237 |     fl = fl / fl[3]
 238 |     fr = fr / fr[3]
 239 |     vertices = np.asarray(
 240 |         [[rl[0], rl[1]], [rr[0], rr[1]], [fr[0], fr[1]], [fl[0], fl[1]]]
 241 |     )
 242 |     return vertices
 243 | 
 244 | 
 245 | """
 246 | Unit tests for GJK collision checks
 247 | Author: Hongrui Zheng
 248 | """
 249 | 
 250 | import time
 251 | import unittest
 252 | 
 253 | 
 254 | class CollisionTests(unittest.TestCase):
 255 |     def setUp(self):
 256 |         # test params
 257 |         np.random.seed(1234)
 258 | 
 259 |         # Collision check body
 260 |         self.vertices1 = np.asarray([[4, 11.0], [5, 5], [9, 9], [10, 10]])
 261 | 
 262 |         # car size
 263 |         self.length = 0.32
 264 |         self.width = 0.22
 265 | 
 266 |     def test_get_vert(self):
 267 |         test_pose = np.array([2.3, 6.7, 0.8])
 268 |         vertices = get_vertices(test_pose, self.length, self.width)
 269 |         rect = np.vstack((vertices, vertices[0, :]))
 270 |         import matplotlib.pyplot as plt
 271 | 
 272 |         plt.scatter(test_pose[0], test_pose[1], c="red")
 273 |         plt.plot(rect[:, 0], rect[:, 1])
 274 |         plt.xlim([1, 4])
 275 |         plt.ylim([5, 8])
 276 |         plt.axes().set_aspect("equal")
 277 |         plt.show()
 278 |         self.assertTrue(vertices.shape == (4, 2))
 279 | 
 280 |     def test_get_vert_fps(self):
 281 |         test_pose = np.array([2.3, 6.7, 0.8])
 282 |         start = time.time()
 283 |         for _ in range(1000):
 284 |             vertices = get_vertices(test_pose, self.length, self.width)
 285 |         elapsed = time.time() - start
 286 |         fps = 1000 / elapsed
 287 |         print("get vertices fps:", fps)
 288 |         self.assertTrue(fps > 500)
 289 | 
 290 |     def test_random_collision(self):
 291 |         # perturb the body by a small amount and make sure it all collides with the original body
 292 |         for _ in range(1000):
 293 |             a = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 294 |             b = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 295 |             self.assertTrue(collision(a, b))
 296 | 
 297 |     def test_fps(self):
 298 |         # also perturb the body but mainly want to test GJK speed
 299 |         start = time.time()
 300 |         for _ in range(1000):
 301 |             a = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 302 |             b = self.vertices1 + np.random.normal(size=(self.vertices1.shape)) / 100.0
 303 |             collision(a, b)
 304 |         elapsed = time.time() - start
 305 |         fps = 1000 / elapsed
 306 |         print("gjk fps:", fps)
 307 |         self.assertTrue(fps > 500)
 308 | 
 309 | 
 310 | if __name__ == "__main__":
 311 |     unittest.main()

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\dynamics_test.py`:

```py
   1 | # Copyright 2020 Technical University of Munich, Professorship of Cyber-Physical Systems, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   2 | 
   3 | # Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
   4 | 
   5 | # 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   6 | 
   7 | # 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   8 | 
   9 | # 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
  10 | 
  11 | # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  12 | 
  13 | 
  14 | """
  15 | Prototype of vehicle dynamics functions and classes for simulating 2D Single 
  16 | Track dynamic model
  17 | Following the implementation of commanroad's Single Track Dynamics model
  18 | Original implementation: https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/
  19 | Author: Hongrui Zheng
  20 | """
  21 | 
  22 | import numpy as np
  23 | from numba import njit
  24 | 
  25 | import unittest
  26 | import time
  27 | 
  28 | 
  29 | @njit(cache=True)
  30 | def accl_constraints(vel, accl, v_switch, a_max, v_min, v_max):
  31 |     """
  32 |     Acceleration constraints, adjusts the acceleration based on constraints
  33 | 
  34 |         Args:
  35 |             vel (float): current velocity of the vehicle
  36 |             accl (float): unconstraint desired acceleration
  37 |             v_switch (float): switching velocity (velocity at which the acceleration is no longer able to create wheel spin)
  38 |             a_max (float): maximum allowed acceleration
  39 |             v_min (float): minimum allowed velocity
  40 |             v_max (float): maximum allowed velocity
  41 | 
  42 |         Returns:
  43 |             accl (float): adjusted acceleration
  44 |     """
  45 | 
  46 |     # positive accl limit
  47 |     if vel > v_switch:
  48 |         pos_limit = a_max * v_switch / vel
  49 |     else:
  50 |         pos_limit = a_max
  51 | 
  52 |     # accl limit reached?
  53 |     if (vel <= v_min and accl <= 0) or (vel >= v_max and accl >= 0):
  54 |         accl = 0.0
  55 |     elif accl <= -a_max:
  56 |         accl = -a_max
  57 |     elif accl >= pos_limit:
  58 |         accl = pos_limit
  59 | 
  60 |     return accl
  61 | 
  62 | 
  63 | @njit(cache=True)
  64 | def steering_constraint(
  65 |     steering_angle, steering_velocity, s_min, s_max, sv_min, sv_max
  66 | ):
  67 |     """
  68 |     Steering constraints, adjusts the steering velocity based on constraints
  69 | 
  70 |         Args:
  71 |             steering_angle (float): current steering_angle of the vehicle
  72 |             steering_velocity (float): unconstraint desired steering_velocity
  73 |             s_min (float): minimum steering angle
  74 |             s_max (float): maximum steering angle
  75 |             sv_min (float): minimum steering velocity
  76 |             sv_max (float): maximum steering velocity
  77 | 
  78 |         Returns:
  79 |             steering_velocity (float): adjusted steering velocity
  80 |     """
  81 | 
  82 |     # constraint steering velocity
  83 |     if (steering_angle <= s_min and steering_velocity <= 0) or (
  84 |         steering_angle >= s_max and steering_velocity >= 0
  85 |     ):
  86 |         steering_velocity = 0.0
  87 |     elif steering_velocity <= sv_min:
  88 |         steering_velocity = sv_min
  89 |     elif steering_velocity >= sv_max:
  90 |         steering_velocity = sv_max
  91 | 
  92 |     return steering_velocity
  93 | 
  94 | 
  95 | @njit(cache=True)
  96 | def vehicle_dynamics_ks(
  97 |     x,
  98 |     u_init,
  99 |     mu,
 100 |     C_Sf,
 101 |     C_Sr,
 102 |     lf,
 103 |     lr,
 104 |     h,
 105 |     m,
 106 |     I,
 107 |     s_min,
 108 |     s_max,
 109 |     sv_min,
 110 |     sv_max,
 111 |     v_switch,
 112 |     a_max,
 113 |     v_min,
 114 |     v_max,
 115 | ):
 116 |     """
 117 |     Single Track Kinematic Vehicle Dynamics.
 118 | 
 119 |         Args:
 120 |             x (numpy.ndarray (3, )): vehicle state vector (x1, x2, x3, x4, x5)
 121 |                 x1: x position in global coordinates
 122 |                 x2: y position in global coordinates
 123 |                 x3: steering angle of front wheels
 124 |                 x4: velocity in x direction
 125 |                 x5: yaw angle
 126 |             u (numpy.ndarray (2, )): control input vector (u1, u2)
 127 |                 u1: steering angle velocity of front wheels
 128 |                 u2: longitudinal acceleration 
 129 | 
 130 |         Returns:
 131 |             f (numpy.ndarray): right hand side of differential equations
 132 |     """
 133 |     # wheelbase
 134 |     lwb = lf + lr
 135 | 
 136 |     # constraints
 137 |     u = np.array(
 138 |         [
 139 |             steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max),
 140 |             accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max),
 141 |         ]
 142 |     )
 143 | 
 144 |     # system dynamics
 145 |     f = np.array(
 146 |         [
 147 |             x[3] * np.cos(x[4]),
 148 |             x[3] * np.sin(x[4]),
 149 |             u[0],
 150 |             u[1],
 151 |             x[3] / lwb * np.tan(x[2]),
 152 |         ]
 153 |     )
 154 |     return f
 155 | 
 156 | 
 157 | @njit(cache=True)
 158 | def vehicle_dynamics_st(
 159 |     x,
 160 |     u_init,
 161 |     mu,
 162 |     C_Sf,
 163 |     C_Sr,
 164 |     lf,
 165 |     lr,
 166 |     h,
 167 |     m,
 168 |     I,
 169 |     s_min,
 170 |     s_max,
 171 |     sv_min,
 172 |     sv_max,
 173 |     v_switch,
 174 |     a_max,
 175 |     v_min,
 176 |     v_max,
 177 | ):
 178 |     """
 179 |     Single Track Dynamic Vehicle Dynamics.
 180 | 
 181 |         Args:
 182 |             x (numpy.ndarray (3, )): vehicle state vector (x1, x2, x3, x4, x5, x6, x7)
 183 |                 x1: x position in global coordinates
 184 |                 x2: y position in global coordinates
 185 |                 x3: steering angle of front wheels
 186 |                 x4: velocity in x direction
 187 |                 x5: yaw angle
 188 |                 x6: yaw rate
 189 |                 x7: slip angle at vehicle center
 190 |             u (numpy.ndarray (2, )): control input vector (u1, u2)
 191 |                 u1: steering angle velocity of front wheels
 192 |                 u2: longitudinal acceleration 
 193 | 
 194 |         Returns:
 195 |             f (numpy.ndarray): right hand side of differential equations
 196 |     """
 197 | 
 198 |     # gravity constant m/s^2
 199 |     g = 9.81
 200 | 
 201 |     # constraints
 202 |     u = np.array(
 203 |         [
 204 |             steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max),
 205 |             accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max),
 206 |         ]
 207 |     )
 208 | 
 209 |     # switch to kinematic model for small velocities
 210 |     if abs(x[3]) < 0.1:
 211 |         # wheelbase
 212 |         lwb = lf + lr
 213 | 
 214 |         # system dynamics
 215 |         x_ks = x[0:5]
 216 |         f_ks = vehicle_dynamics_ks(
 217 |             x_ks,
 218 |             u,
 219 |             mu,
 220 |             C_Sf,
 221 |             C_Sr,
 222 |             lf,
 223 |             lr,
 224 |             h,
 225 |             m,
 226 |             I,
 227 |             s_min,
 228 |             s_max,
 229 |             sv_min,
 230 |             sv_max,
 231 |             v_switch,
 232 |             a_max,
 233 |             v_min,
 234 |             v_max,
 235 |         )
 236 |         f = np.hstack(
 237 |             (
 238 |                 f_ks,
 239 |                 np.array(
 240 |                     [
 241 |                         u[1] / lwb * np.tan(x[2])
 242 |                         + x[3] / (lwb * np.cos(x[2]) ** 2) * u[0],
 243 |                         0,
 244 |                     ]
 245 |                 ),
 246 |             )
 247 |         )
 248 | 
 249 |     else:
 250 |         # system dynamics
 251 |         f = np.array(
 252 |             [
 253 |                 x[3] * np.cos(x[6] + x[4]),
 254 |                 x[3] * np.sin(x[6] + x[4]),
 255 |                 u[0],
 256 |                 u[1],
 257 |                 x[5],
 258 |                 -mu
 259 |                 * m
 260 |                 / (x[3] * I * (lr + lf))
 261 |                 * (
 262 |                     lf ** 2 * C_Sf * (g * lr - u[1] * h)
 263 |                     + lr ** 2 * C_Sr * (g * lf + u[1] * h)
 264 |                 )
 265 |                 * x[5]
 266 |                 + mu
 267 |                 * m
 268 |                 / (I * (lr + lf))
 269 |                 * (lr * C_Sr * (g * lf + u[1] * h) - lf * C_Sf * (g * lr - u[1] * h))
 270 |                 * x[6]
 271 |                 + mu * m / (I * (lr + lf)) * lf * C_Sf * (g * lr - u[1] * h) * x[2],
 272 |                 (
 273 |                     mu
 274 |                     / (x[3] ** 2 * (lr + lf))
 275 |                     * (
 276 |                         C_Sr * (g * lf + u[1] * h) * lr
 277 |                         - C_Sf * (g * lr - u[1] * h) * lf
 278 |                     )
 279 |                     - 1
 280 |                 )
 281 |                 * x[5]
 282 |                 - mu
 283 |                 / (x[3] * (lr + lf))
 284 |                 * (C_Sr * (g * lf + u[1] * h) + C_Sf * (g * lr - u[1] * h))
 285 |                 * x[6]
 286 |                 + mu / (x[3] * (lr + lf)) * (C_Sf * (g * lr - u[1] * h)) * x[2],
 287 |             ]
 288 |         )
 289 | 
 290 |     return f
 291 | 
 292 | 
 293 | @njit(cache=True)
 294 | def pid(speed, steer):
 295 |     """
 296 |     Basic controller for speed/steer -> accl./steer vel.
 297 | 
 298 |         Args:
 299 |             speed (float): desired input speed
 300 |             steer (float): desired input steering angle
 301 | 
 302 |         Returns:
 303 |             accl (float): desired input acceleration
 304 |             sv (float): desired input steering velocity
 305 |     """
 306 |     return
 307 | 
 308 | 
 309 | def func_KS(
 310 |     x,
 311 |     t,
 312 |     u,
 313 |     mu,
 314 |     C_Sf,
 315 |     C_Sr,
 316 |     lf,
 317 |     lr,
 318 |     h,
 319 |     m,
 320 |     I,
 321 |     s_min,
 322 |     s_max,
 323 |     sv_min,
 324 |     sv_max,
 325 |     v_switch,
 326 |     a_max,
 327 |     v_min,
 328 |     v_max,
 329 | ):
 330 |     f = vehicle_dynamics_ks(
 331 |         x,
 332 |         u,
 333 |         mu,
 334 |         C_Sf,
 335 |         C_Sr,
 336 |         lf,
 337 |         lr,
 338 |         h,
 339 |         m,
 340 |         I,
 341 |         s_min,
 342 |         s_max,
 343 |         sv_min,
 344 |         sv_max,
 345 |         v_switch,
 346 |         a_max,
 347 |         v_min,
 348 |         v_max,
 349 |     )
 350 |     return f
 351 | 
 352 | 
 353 | def func_ST(
 354 |     x,
 355 |     t,
 356 |     u,
 357 |     mu,
 358 |     C_Sf,
 359 |     C_Sr,
 360 |     lf,
 361 |     lr,
 362 |     h,
 363 |     m,
 364 |     I,
 365 |     s_min,
 366 |     s_max,
 367 |     sv_min,
 368 |     sv_max,
 369 |     v_switch,
 370 |     a_max,
 371 |     v_min,
 372 |     v_max,
 373 | ):
 374 |     f = vehicle_dynamics_st(
 375 |         x,
 376 |         u,
 377 |         mu,
 378 |         C_Sf,
 379 |         C_Sr,
 380 |         lf,
 381 |         lr,
 382 |         h,
 383 |         m,
 384 |         I,
 385 |         s_min,
 386 |         s_max,
 387 |         sv_min,
 388 |         sv_max,
 389 |         v_switch,
 390 |         a_max,
 391 |         v_min,
 392 |         v_max,
 393 |     )
 394 |     return f
 395 | 
 396 | 
 397 | class DynamicsTest(unittest.TestCase):
 398 |     def setUp(self):
 399 |         # test params
 400 |         self.mu = 1.0489
 401 |         self.C_Sf = 21.92 / 1.0489
 402 |         self.C_Sr = 21.92 / 1.0489
 403 |         self.lf = 0.3048 * 3.793293
 404 |         self.lr = 0.3048 * 4.667707
 405 |         self.h = 0.3048 * 2.01355
 406 |         self.m = 4.4482216152605 / 0.3048 * 74.91452
 407 |         self.I = 4.4482216152605 * 0.3048 * 1321.416
 408 | 
 409 |         # steering constraints
 410 |         self.s_min = -1.066  # minimum steering angle [rad]
 411 |         self.s_max = 1.066  # maximum steering angle [rad]
 412 |         self.sv_min = -0.4  # minimum steering velocity [rad/s]
 413 |         self.sv_max = 0.4  # maximum steering velocity [rad/s]
 414 | 
 415 |         # longitudinal constraints
 416 |         self.v_min = -13.6  # minimum velocity [m/s]
 417 |         self.v_max = 50.8  # minimum velocity [m/s]
 418 |         self.v_switch = 7.319  # switching velocity [m/s]
 419 |         self.a_max = 11.5  # maximum absolute acceleration [m/s^2]
 420 | 
 421 |     def test_derivatives(self):
 422 |         # ground truth derivatives
 423 |         f_ks_gt = [
 424 |             16.3475935934250209,
 425 |             0.4819314886013121,
 426 |             0.1500000000000000,
 427 |             5.1464424102339752,
 428 |             0.2401426578627629,
 429 |         ]
 430 |         f_st_gt = [
 431 |             15.7213512030862397,
 432 |             0.0925527979719355,
 433 |             0.1500000000000000,
 434 |             5.3536773276413925,
 435 |             0.0529001056654038,
 436 |             0.6435589397748606,
 437 |             0.0313297971641291,
 438 |         ]
 439 | 
 440 |         # system dynamics
 441 |         g = 9.81
 442 |         x_ks = np.array(
 443 |             [
 444 |                 3.9579422297936526,
 445 |                 0.0391650102771405,
 446 |                 0.0378491427211811,
 447 |                 16.3546957860883566,
 448 |                 0.0294717351052816,
 449 |             ]
 450 |         )
 451 |         x_st = np.array(
 452 |             [
 453 |                 2.0233348142065677,
 454 |                 0.0041907137716636,
 455 |                 0.0197545248559617,
 456 |                 15.7216236334290116,
 457 |                 0.0025857914776859,
 458 |                 0.0529001056654038,
 459 |                 0.0033012170610298,
 460 |             ]
 461 |         )
 462 |         v_delta = 0.15
 463 |         acc = 0.63 * g
 464 |         u = np.array([v_delta, acc])
 465 | 
 466 |         f_ks = vehicle_dynamics_ks(
 467 |             x_ks,
 468 |             u,
 469 |             self.mu,
 470 |             self.C_Sf,
 471 |             self.C_Sr,
 472 |             self.lf,
 473 |             self.lr,
 474 |             self.h,
 475 |             self.m,
 476 |             self.I,
 477 |             self.s_min,
 478 |             self.s_max,
 479 |             self.sv_min,
 480 |             self.sv_max,
 481 |             self.v_switch,
 482 |             self.a_max,
 483 |             self.v_min,
 484 |             self.v_max,
 485 |         )
 486 |         f_st = vehicle_dynamics_st(
 487 |             x_st,
 488 |             u,
 489 |             self.mu,
 490 |             self.C_Sf,
 491 |             self.C_Sr,
 492 |             self.lf,
 493 |             self.lr,
 494 |             self.h,
 495 |             self.m,
 496 |             self.I,
 497 |             self.s_min,
 498 |             self.s_max,
 499 |             self.sv_min,
 500 |             self.sv_max,
 501 |             self.v_switch,
 502 |             self.a_max,
 503 |             self.v_min,
 504 |             self.v_max,
 505 |         )
 506 | 
 507 |         start = time.time()
 508 |         for i in range(10000):
 509 |             f_st = vehicle_dynamics_st(
 510 |                 x_st,
 511 |                 u,
 512 |                 self.mu,
 513 |                 self.C_Sf,
 514 |                 self.C_Sr,
 515 |                 self.lf,
 516 |                 self.lr,
 517 |                 self.h,
 518 |                 self.m,
 519 |                 self.I,
 520 |                 self.s_min,
 521 |                 self.s_max,
 522 |                 self.sv_min,
 523 |                 self.sv_max,
 524 |                 self.v_switch,
 525 |                 self.a_max,
 526 |                 self.v_min,
 527 |                 self.v_max,
 528 |             )
 529 |         duration = time.time() - start
 530 |         avg_fps = 10000 / duration
 531 | 
 532 |         self.assertAlmostEqual(np.max(np.abs(f_ks_gt - f_ks)), 0.0)
 533 |         self.assertAlmostEqual(np.max(np.abs(f_st_gt - f_st)), 0.0)
 534 |         self.assertGreater(avg_fps, 5000)
 535 | 
 536 |     def test_zeroinit_roll(self):
 537 |         from scipy.integrate import odeint
 538 | 
 539 |         # testing for zero initial state, zero input singularities
 540 |         g = 9.81
 541 |         t_start = 0.0
 542 |         t_final = 1.0
 543 |         delta0 = 0.0
 544 |         vel0 = 0.0
 545 |         Psi0 = 0.0
 546 |         dotPsi0 = 0.0
 547 |         beta0 = 0.0
 548 |         sy0 = 0.0
 549 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 550 | 
 551 |         x0_KS = np.array(initial_state[0:5])
 552 |         x0_ST = np.array(initial_state)
 553 | 
 554 |         # time vector
 555 |         t = np.arange(t_start, t_final, 1e-4)
 556 | 
 557 |         # set input: rolling car (velocity should stay constant)
 558 |         u = np.array([0.0, 0.0])
 559 | 
 560 |         # simulate single-track model
 561 |         x_roll_st = odeint(
 562 |             func_ST,
 563 |             x0_ST,
 564 |             t,
 565 |             args=(
 566 |                 u,
 567 |                 self.mu,
 568 |                 self.C_Sf,
 569 |                 self.C_Sr,
 570 |                 self.lf,
 571 |                 self.lr,
 572 |                 self.h,
 573 |                 self.m,
 574 |                 self.I,
 575 |                 self.s_min,
 576 |                 self.s_max,
 577 |                 self.sv_min,
 578 |                 self.sv_max,
 579 |                 self.v_switch,
 580 |                 self.a_max,
 581 |                 self.v_min,
 582 |                 self.v_max,
 583 |             ),
 584 |         )
 585 |         # simulate kinematic single-track model
 586 |         x_roll_ks = odeint(
 587 |             func_KS,
 588 |             x0_KS,
 589 |             t,
 590 |             args=(
 591 |                 u,
 592 |                 self.mu,
 593 |                 self.C_Sf,
 594 |                 self.C_Sr,
 595 |                 self.lf,
 596 |                 self.lr,
 597 |                 self.h,
 598 |                 self.m,
 599 |                 self.I,
 600 |                 self.s_min,
 601 |                 self.s_max,
 602 |                 self.sv_min,
 603 |                 self.sv_max,
 604 |                 self.v_switch,
 605 |                 self.a_max,
 606 |                 self.v_min,
 607 |                 self.v_max,
 608 |             ),
 609 |         )
 610 | 
 611 |         self.assertTrue(all(x_roll_st[-1] == x0_ST))
 612 |         self.assertTrue(all(x_roll_ks[-1] == x0_KS))
 613 | 
 614 |     def test_zeroinit_dec(self):
 615 |         from scipy.integrate import odeint
 616 | 
 617 |         # testing for zero initial state, decelerating input singularities
 618 |         g = 9.81
 619 |         t_start = 0.0
 620 |         t_final = 1.0
 621 |         delta0 = 0.0
 622 |         vel0 = 0.0
 623 |         Psi0 = 0.0
 624 |         dotPsi0 = 0.0
 625 |         beta0 = 0.0
 626 |         sy0 = 0.0
 627 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 628 | 
 629 |         x0_KS = np.array(initial_state[0:5])
 630 |         x0_ST = np.array(initial_state)
 631 | 
 632 |         # time vector
 633 |         t = np.arange(t_start, t_final, 1e-4)
 634 | 
 635 |         # set decel input
 636 |         u = np.array([0.0, -0.7 * g])
 637 | 
 638 |         # simulate single-track model
 639 |         x_dec_st = odeint(
 640 |             func_ST,
 641 |             x0_ST,
 642 |             t,
 643 |             args=(
 644 |                 u,
 645 |                 self.mu,
 646 |                 self.C_Sf,
 647 |                 self.C_Sr,
 648 |                 self.lf,
 649 |                 self.lr,
 650 |                 self.h,
 651 |                 self.m,
 652 |                 self.I,
 653 |                 self.s_min,
 654 |                 self.s_max,
 655 |                 self.sv_min,
 656 |                 self.sv_max,
 657 |                 self.v_switch,
 658 |                 self.a_max,
 659 |                 self.v_min,
 660 |                 self.v_max,
 661 |             ),
 662 |         )
 663 |         # simulate kinematic single-track model
 664 |         x_dec_ks = odeint(
 665 |             func_KS,
 666 |             x0_KS,
 667 |             t,
 668 |             args=(
 669 |                 u,
 670 |                 self.mu,
 671 |                 self.C_Sf,
 672 |                 self.C_Sr,
 673 |                 self.lf,
 674 |                 self.lr,
 675 |                 self.h,
 676 |                 self.m,
 677 |                 self.I,
 678 |                 self.s_min,
 679 |                 self.s_max,
 680 |                 self.sv_min,
 681 |                 self.sv_max,
 682 |                 self.v_switch,
 683 |                 self.a_max,
 684 |                 self.v_min,
 685 |                 self.v_max,
 686 |             ),
 687 |         )
 688 | 
 689 |         # ground truth for single-track model
 690 |         x_dec_st_gt = [
 691 |             -3.4335000000000013,
 692 |             0.0000000000000000,
 693 |             0.0000000000000000,
 694 |             -6.8670000000000018,
 695 |             0.0000000000000000,
 696 |             0.0000000000000000,
 697 |             0.0000000000000000,
 698 |         ]
 699 |         # ground truth for kinematic single-track model
 700 |         x_dec_ks_gt = [
 701 |             -3.4335000000000013,
 702 |             0.0000000000000000,
 703 |             0.0000000000000000,
 704 |             -6.8670000000000018,
 705 |             0.0000000000000000,
 706 |         ]
 707 | 
 708 |         self.assertTrue(all(abs(x_dec_st[-1] - x_dec_st_gt) < 1e-2))
 709 |         self.assertTrue(all(abs(x_dec_ks[-1] - x_dec_ks_gt) < 1e-2))
 710 | 
 711 |     def test_zeroinit_acc(self):
 712 |         from scipy.integrate import odeint
 713 | 
 714 |         # testing for zero initial state, accelerating with left steer input singularities
 715 |         # wheel spin and velocity should increase more wheel spin at rear
 716 |         g = 9.81
 717 |         t_start = 0.0
 718 |         t_final = 1.0
 719 |         delta0 = 0.0
 720 |         vel0 = 0.0
 721 |         Psi0 = 0.0
 722 |         dotPsi0 = 0.0
 723 |         beta0 = 0.0
 724 |         sy0 = 0.0
 725 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 726 | 
 727 |         x0_KS = np.array(initial_state[0:5])
 728 |         x0_ST = np.array(initial_state)
 729 | 
 730 |         # time vector
 731 |         t = np.arange(t_start, t_final, 1e-4)
 732 | 
 733 |         # set decel input
 734 |         u = np.array([0.15, 0.63 * g])
 735 | 
 736 |         # simulate single-track model
 737 |         x_acc_st = odeint(
 738 |             func_ST,
 739 |             x0_ST,
 740 |             t,
 741 |             args=(
 742 |                 u,
 743 |                 self.mu,
 744 |                 self.C_Sf,
 745 |                 self.C_Sr,
 746 |                 self.lf,
 747 |                 self.lr,
 748 |                 self.h,
 749 |                 self.m,
 750 |                 self.I,
 751 |                 self.s_min,
 752 |                 self.s_max,
 753 |                 self.sv_min,
 754 |                 self.sv_max,
 755 |                 self.v_switch,
 756 |                 self.a_max,
 757 |                 self.v_min,
 758 |                 self.v_max,
 759 |             ),
 760 |         )
 761 |         # simulate kinematic single-track model
 762 |         x_acc_ks = odeint(
 763 |             func_KS,
 764 |             x0_KS,
 765 |             t,
 766 |             args=(
 767 |                 u,
 768 |                 self.mu,
 769 |                 self.C_Sf,
 770 |                 self.C_Sr,
 771 |                 self.lf,
 772 |                 self.lr,
 773 |                 self.h,
 774 |                 self.m,
 775 |                 self.I,
 776 |                 self.s_min,
 777 |                 self.s_max,
 778 |                 self.sv_min,
 779 |                 self.sv_max,
 780 |                 self.v_switch,
 781 |                 self.a_max,
 782 |                 self.v_min,
 783 |                 self.v_max,
 784 |             ),
 785 |         )
 786 | 
 787 |         # ground truth for single-track model
 788 |         x_acc_st_gt = [
 789 |             3.0731976046859715,
 790 |             0.2869835398304389,
 791 |             0.1500000000000000,
 792 |             6.1802999999999999,
 793 |             0.1097747074946325,
 794 |             0.3248268063223301,
 795 |             0.0697547542798040,
 796 |         ]
 797 |         # ground truth for kinematic single-track model
 798 |         x_acc_ks_gt = [
 799 |             3.0845676868494927,
 800 |             0.1484249221523042,
 801 |             0.1500000000000000,
 802 |             6.1803000000000017,
 803 |             0.1203664469224163,
 804 |         ]
 805 | 
 806 |         self.assertTrue(all(abs(x_acc_st[-1] - x_acc_st_gt) < 1e-2))
 807 |         self.assertTrue(all(abs(x_acc_ks[-1] - x_acc_ks_gt) < 1e-2))
 808 | 
 809 |     def test_zeroinit_rollleft(self):
 810 |         from scipy.integrate import odeint
 811 | 
 812 |         # testing for zero initial state, rolling and steering left input singularities
 813 |         g = 9.81
 814 |         t_start = 0.0
 815 |         t_final = 1.0
 816 |         delta0 = 0.0
 817 |         vel0 = 0.0
 818 |         Psi0 = 0.0
 819 |         dotPsi0 = 0.0
 820 |         beta0 = 0.0
 821 |         sy0 = 0.0
 822 |         initial_state = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]
 823 | 
 824 |         x0_KS = np.array(initial_state[0:5])
 825 |         x0_ST = np.array(initial_state)
 826 | 
 827 |         # time vector
 828 |         t = np.arange(t_start, t_final, 1e-4)
 829 | 
 830 |         # set decel input
 831 |         u = np.array([0.15, 0.0])
 832 | 
 833 |         # simulate single-track model
 834 |         x_left_st = odeint(
 835 |             func_ST,
 836 |             x0_ST,
 837 |             t,
 838 |             args=(
 839 |                 u,
 840 |                 self.mu,
 841 |                 self.C_Sf,
 842 |                 self.C_Sr,
 843 |                 self.lf,
 844 |                 self.lr,
 845 |                 self.h,
 846 |                 self.m,
 847 |                 self.I,
 848 |                 self.s_min,
 849 |                 self.s_max,
 850 |                 self.sv_min,
 851 |                 self.sv_max,
 852 |                 self.v_switch,
 853 |                 self.a_max,
 854 |                 self.v_min,
 855 |                 self.v_max,
 856 |             ),
 857 |         )
 858 |         # simulate kinematic single-track model
 859 |         x_left_ks = odeint(
 860 |             func_KS,
 861 |             x0_KS,
 862 |             t,
 863 |             args=(
 864 |                 u,
 865 |                 self.mu,
 866 |                 self.C_Sf,
 867 |                 self.C_Sr,
 868 |                 self.lf,
 869 |                 self.lr,
 870 |                 self.h,
 871 |                 self.m,
 872 |                 self.I,
 873 |                 self.s_min,
 874 |                 self.s_max,
 875 |                 self.sv_min,
 876 |                 self.sv_max,
 877 |                 self.v_switch,
 878 |                 self.a_max,
 879 |                 self.v_min,
 880 |                 self.v_max,
 881 |             ),
 882 |         )
 883 | 
 884 |         # ground truth for single-track model
 885 |         x_left_st_gt = [
 886 |             0.0000000000000000,
 887 |             0.0000000000000000,
 888 |             0.1500000000000000,
 889 |             0.0000000000000000,
 890 |             0.0000000000000000,
 891 |             0.0000000000000000,
 892 |             0.0000000000000000,
 893 |         ]
 894 |         # ground truth for kinematic single-track model
 895 |         x_left_ks_gt = [
 896 |             0.0000000000000000,
 897 |             0.0000000000000000,
 898 |             0.1500000000000000,
 899 |             0.0000000000000000,
 900 |             0.0000000000000000,
 901 |         ]
 902 | 
 903 |         self.assertTrue(all(abs(x_left_st[-1] - x_left_st_gt) < 1e-2))
 904 |         self.assertTrue(all(abs(x_left_ks[-1] - x_left_ks_gt) < 1e-2))
 905 | 
 906 | 
 907 | if __name__ == "__main__":
 908 |     unittest.main()

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\legacy_scan_gen.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Utility functions to generate sample scan data from legacy C++ backend
  26 | Author: Hongrui Zheng
  27 | 
  28 | The script generates sample scan data for 3 different maps used in the unit tests.
  29 | 
  30 | Map 1: Levine
  31 | 
  32 | Map 2: Berlin
  33 | 
  34 | Map 3: Skirkanich
  35 | """
  36 | 
  37 | import numpy as np
  38 | import gym
  39 | import matplotlib.pyplot as plt
  40 | 
  41 | thetas = np.linspace(-2.35, 2.35, num=1080)
  42 | 
  43 | # init
  44 | executable_dir = "../../../build/"
  45 | mass = 3.74
  46 | l_r = 0.17145
  47 | I_z = 0.04712
  48 | mu = 0.523
  49 | h_cg = 0.074
  50 | cs_f = 4.718
  51 | cs_r = 5.4562
  52 | 
  53 | # test poses
  54 | num_test = 10
  55 | test_poses = np.zeros((num_test, 3))
  56 | test_poses[:, 2] = np.linspace(-1.0, 1.0, num=num_test)
  57 | 
  58 | # map 1: vegas
  59 | map_path = "../../../maps/vegas.yaml"
  60 | map_ext = ".png"
  61 | racecar_env = gym.make("f110_gym:f110-v0")
  62 | racecar_env.init_map(map_path, map_ext, False, False)
  63 | racecar_env.update_params(
  64 |     mu, h_cg, l_r, cs_f, cs_r, I_z, mass, executable_dir, double_finish=True
  65 | )
  66 | vegas_scan = np.empty((num_test, 1080))
  67 | for i in range(test_poses.shape[0]):
  68 |     x = [test_poses[i, 0], 200.0]
  69 |     y = [test_poses[i, 1], 200.0]
  70 |     theta = [test_poses[i, 2], 0.0]
  71 |     obs, _, _, _ = racecar_env.reset({"x": x, "y": y, "theta": theta})
  72 |     vegas_scan[i, :] = obs["scans"][0]
  73 | 
  74 | # map 2: berlin
  75 | map_path = "../../../maps/berlin.yaml"
  76 | map_ext = ".png"
  77 | racecar_env = gym.make("f110_gym:f110-v0")
  78 | racecar_env.init_map(map_path, map_ext, False, False)
  79 | racecar_env.update_params(
  80 |     mu, h_cg, l_r, cs_f, cs_r, I_z, mass, executable_dir, double_finish=True
  81 | )
  82 | berlin_scan = np.empty((num_test, 1080))
  83 | for i in range(test_poses.shape[0]):
  84 |     x = [test_poses[i, 0], 200.0]
  85 |     y = [test_poses[i, 1], 200.0]
  86 |     theta = [test_poses[i, 2], 0.0]
  87 |     obs, _, _, _ = racecar_env.reset({"x": x, "y": y, "theta": theta})
  88 |     berlin_scan[i, :] = obs["scans"][0]
  89 | 
  90 | # map 3: skirk
  91 | map_path = "../../../maps/skirk.yaml"
  92 | map_ext = ".png"
  93 | racecar_env = gym.make("f110_gym:f110-v0")
  94 | racecar_env.init_map(map_path, map_ext, False, False)
  95 | racecar_env.update_params(
  96 |     mu, h_cg, l_r, cs_f, cs_r, I_z, mass, executable_dir, double_finish=True
  97 | )
  98 | skirk_scan = np.empty((num_test, 1080))
  99 | for i in range(test_poses.shape[0]):
 100 |     x = [test_poses[i, 0], 200.0]
 101 |     y = [test_poses[i, 1], 200.0]
 102 |     theta = [test_poses[i, 2], 0.0]
 103 |     obs, _, _, _ = racecar_env.reset({"x": x, "y": y, "theta": theta})
 104 |     skirk_scan[i, :] = obs["scans"][0]
 105 | 
 106 | # package data
 107 | np.savez_compressed(
 108 |     "legacy_scan.npz", vegas=vegas_scan, berlin=berlin_scan, skirk=skirk_scan
 109 | )

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\pyglet_test.py`:

```py
   1 | import numpy as np
   2 | from PIL import Image
   3 | import yaml
   4 | 
   5 | from pyglet.gl import *
   6 | import pyglet
   7 | from pyglet import font, graphics, window
   8 | 
   9 | import argparse
  10 | 
  11 | 
  12 | class Camera:
  13 |     """ A simple 2D camera that contains the speed and offset."""
  14 | 
  15 |     def __init__(
  16 |         self, window: pyglet.window.Window, scroll_speed=1, min_zoom=1, max_zoom=4
  17 |     ):
  18 |         assert (
  19 |             min_zoom <= max_zoom
  20 |         ), "Minimum zoom must not be greater than maximum zoom"
  21 |         self._window = window
  22 |         self.scroll_speed = scroll_speed
  23 |         self.max_zoom = max_zoom
  24 |         self.min_zoom = min_zoom
  25 |         self.offset_x = 0
  26 |         self.offset_y = 0
  27 |         self._zoom = max(min(1, self.max_zoom), self.min_zoom)
  28 | 
  29 |     @property
  30 |     def zoom(self):
  31 |         return self._zoom
  32 | 
  33 |     @zoom.setter
  34 |     def zoom(self, value):
  35 |         """ Here we set zoom, clamp value to minimum of min_zoom and max of max_zoom."""
  36 |         self._zoom = max(min(value, self.max_zoom), self.min_zoom)
  37 | 
  38 |     @property
  39 |     def position(self):
  40 |         """Query the current offset."""
  41 |         return self.offset_x, self.offset_y
  42 | 
  43 |     @position.setter
  44 |     def position(self, value):
  45 |         """Set the scroll offset directly."""
  46 |         self.offset_x, self.offset_y = value
  47 | 
  48 |     def move(self, axis_x, axis_y):
  49 |         """ Move axis direction with scroll_speed.
  50 |             Example: Move left -> move(-1, 0)
  51 |          """
  52 |         self.offset_x += self.scroll_speed * axis_x
  53 |         self.offset_y += self.scroll_speed * axis_y
  54 | 
  55 |     def begin(self):
  56 |         # Set the current camera offset so you can draw your scene.
  57 | 
  58 |         # Translate using the offset.
  59 |         view_matrix = self._window.view.translate(
  60 |             -self.offset_x * self._zoom, -self.offset_y * self._zoom, 0
  61 |         )
  62 |         # Scale by zoom level.
  63 |         view_matrix = view_matrix.scale(self._zoom, self._zoom, 1)
  64 | 
  65 |         self._window.view = view_matrix
  66 | 
  67 |     def end(self):
  68 |         # Since this is a matrix, you will need to reverse the translate after rendering otherwise
  69 |         # it will multiply the current offset every draw update pushing it further and further away.
  70 | 
  71 |         # Reverse scale, since that was the last transform.
  72 |         view_matrix = self._window.view.scale(1 / self._zoom, 1 / self._zoom, 1)
  73 |         # Reverse translate.
  74 |         view_matrix = view_matrix.translate(
  75 |             self.offset_x * self._zoom, self.offset_y * self._zoom, 0
  76 |         )
  77 | 
  78 |         self._window.view = view_matrix
  79 | 
  80 |     def __enter__(self):
  81 |         self.begin()
  82 | 
  83 |     def __exit__(self, exception_type, exception_value, traceback):
  84 |         self.end()
  85 | 
  86 | 
  87 | class CenteredCamera(Camera):
  88 |     """A simple 2D camera class. 0, 0 will be the center of the screen, as opposed to the bottom left."""
  89 | 
  90 |     def begin(self):
  91 |         x = -self._window.width // 2 / self._zoom + self.offset_x
  92 |         y = -self._window.height // 2 / self._zoom + self.offset_y
  93 | 
  94 |         view_matrix = self._window.view.translate(-x * self._zoom, -y * self._zoom, 0)
  95 |         view_matrix = view_matrix.scale(self._zoom, self._zoom, 1)
  96 |         self._window.view = view_matrix
  97 | 
  98 |     def end(self):
  99 |         x = -self._window.width // 2 / self._zoom + self.offset_x
 100 |         y = -self._window.height // 2 / self._zoom + self.offset_y
 101 | 
 102 |         view_matrix = self._window.view.scale(1 / self._zoom, 1 / self._zoom, 1)
 103 |         view_matrix = view_matrix.translate(x * self._zoom, y * self._zoom, 0)
 104 |         self._window.view = view_matrix
 105 | 
 106 | 
 107 | parser = argparse.ArgumentParser()
 108 | parser.add_argument(
 109 |     "--map_path", type=str, required=True, help="Path to the map without extensions"
 110 | )
 111 | parser.add_argument(
 112 |     "--map_ext", type=str, required=True, help="Extension of the map image file"
 113 | )
 114 | args = parser.parse_args()
 115 | 
 116 | # load map yaml
 117 | with open(args.map_path + ".yaml", "r") as yaml_stream:
 118 |     try:
 119 |         map_metada = yaml.safe_load(yaml_stream)
 120 |         map_resolution = map_metada["resolution"]
 121 |         origin = map_metada["origin"]
 122 |         origin_x = origin[0]
 123 |         origin_y = origin[1]
 124 |     except yaml.YAMLError as ex:
 125 |         print(ex)
 126 | 
 127 | # load map image
 128 | map_img = np.array(
 129 |     Image.open(args.map_path + args.map_ext).transpose(Image.FLIP_TOP_BOTTOM)
 130 | ).astype(np.float64)
 131 | map_height = map_img.shape[0]
 132 | map_width = map_img.shape[1]
 133 | 
 134 | # convert map pixels to coordinates
 135 | range_x = np.arange(map_width)
 136 | range_y = np.arange(map_height)
 137 | map_x, map_y = np.meshgrid(range_x, range_y)
 138 | map_x = (map_x * map_resolution + origin_x).flatten()
 139 | map_y = (map_y * map_resolution + origin_y).flatten()
 140 | map_z = np.zeros(map_y.shape)
 141 | map_coords = np.vstack((map_x, map_y, map_z))
 142 | 
 143 | # mask and only leave the obstacle points
 144 | map_mask = map_img == 0.0
 145 | map_mask_flat = map_mask.flatten()
 146 | map_points = map_coords[:, map_mask_flat].T
 147 | 
 148 | # prep opengl
 149 | try:
 150 |     # Try and create a window with multisampling (antialiasing)
 151 |     config = Config(sample_buffers=1, samples=4, depth_size=16, double_buffer=True)
 152 |     window = window.Window(resizable=True, config=config)
 153 | except window.NoSuchConfigException:
 154 |     # Fall back to no multisampling for old hardware
 155 |     window = window.Window(resizable=True)
 156 | 
 157 | glClearColor(18 / 255, 4 / 255, 88 / 255, 1.0)
 158 | glEnable(GL_DEPTH_TEST)
 159 | glTranslatef(25, -5, -60)
 160 | 
 161 | cam = Camera(window)
 162 | 
 163 | 
 164 | @window.event
 165 | def on_resize(width, height):
 166 |     # Override the default on_resize handler to create a 3D projection
 167 |     glViewport(0, 0, width, height)
 168 |     glMatrixMode(GL_PROJECTION)
 169 |     glLoadIdentity()
 170 |     gluPerspective(60.0, width / float(height), 0.1, 1000.0)
 171 |     glMatrixMode(GL_MODELVIEW)
 172 |     return pyglet.event.EVENT_HANDLED
 173 | 
 174 | 
 175 | batch = graphics.Batch()
 176 | 
 177 | points = []
 178 | for i in range(map_points.shape[0]):
 179 |     particle = batch.add(
 180 |         1,
 181 |         GL_POINTS,
 182 |         None,
 183 |         ("v3f/stream", [map_points[i, 0], map_points[i, 1], map_points[i, 2]]),
 184 |     )
 185 |     points.append(particle)
 186 | 
 187 | 
 188 | def loop(dt):
 189 |     print(pyglet.clock.get_fps())
 190 |     pass
 191 | 
 192 | 
 193 | @window.event
 194 | def on_draw():
 195 |     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
 196 |     glColor3f(254 / 255, 117 / 255, 254 / 255)
 197 |     cam.begin()
 198 |     batch.draw()
 199 |     cam.end()
 200 | 
 201 | 
 202 | pyglet.clock.schedule(loop)
 203 | pyglet.app.run()

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\pyglet_test_camera.py`:

```py
   1 | import pyglet
   2 | from pyglet.gl import *
   3 | 
   4 | # Zooming constants
   5 | ZOOM_IN_FACTOR = 1.2
   6 | ZOOM_OUT_FACTOR = 1 / ZOOM_IN_FACTOR
   7 | 
   8 | 
   9 | class App(pyglet.window.Window):
  10 |     def __init__(self, width, height, *args, **kwargs):
  11 |         conf = Config(sample_buffers=1, samples=4, depth_size=16, double_buffer=True)
  12 |         super().__init__(width, height, config=conf, *args, **kwargs)
  13 | 
  14 |         # Initialize camera values
  15 |         self.left = 0
  16 |         self.right = width
  17 |         self.bottom = 0
  18 |         self.top = height
  19 |         self.zoom_level = 1
  20 |         self.zoomed_width = width
  21 |         self.zoomed_height = height
  22 | 
  23 |     def init_gl(self, width, height):
  24 |         # Set clear color
  25 |         glClearColor(0 / 255, 0 / 255, 0 / 255, 0 / 255)
  26 | 
  27 |         # Set antialiasing
  28 |         glEnable(GL_LINE_SMOOTH)
  29 |         glEnable(GL_POLYGON_SMOOTH)
  30 |         glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
  31 | 
  32 |         # Set alpha blending
  33 |         glEnable(GL_BLEND)
  34 |         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
  35 | 
  36 |         # Set viewport
  37 |         glViewport(0, 0, width, height)
  38 | 
  39 |     def on_resize(self, width, height):
  40 |         super().on_resize(width, height)
  41 |         size = self.get_size()
  42 |         self.left = 0
  43 |         self.right = size[0]
  44 |         self.bottom = 0
  45 |         self.top = size[1]
  46 |         self.zoomed_width = size[0]
  47 |         self.zoomed_height = size[1]
  48 | 
  49 |         # # Set window values
  50 |         # self.width  = width
  51 |         # self.height = height
  52 |         # # Initialize OpenGL context
  53 |         # self.init_gl(width, height)
  54 |         # self.width = width
  55 |         # self.height = height
  56 |         # pass
  57 | 
  58 |     def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
  59 |         # Move camera
  60 |         self.left -= dx * self.zoom_level
  61 |         self.right -= dx * self.zoom_level
  62 |         self.bottom -= dy * self.zoom_level
  63 |         self.top -= dy * self.zoom_level
  64 | 
  65 |     def on_mouse_scroll(self, x, y, dx, dy):
  66 |         # Get scale factor
  67 |         f = ZOOM_IN_FACTOR if dy > 0 else ZOOM_OUT_FACTOR if dy < 0 else 1
  68 |         # If zoom_level is in the proper range
  69 |         if 0.2 < self.zoom_level * f < 5:
  70 | 
  71 |             self.zoom_level *= f
  72 | 
  73 |             size = self.get_size()
  74 | 
  75 |             mouse_x = x / size[0]
  76 |             mouse_y = y / size[1]
  77 | 
  78 |             mouse_x_in_world = self.left + mouse_x * self.zoomed_width
  79 |             mouse_y_in_world = self.bottom + mouse_y * self.zoomed_height
  80 | 
  81 |             self.zoomed_width *= f
  82 |             self.zoomed_height *= f
  83 | 
  84 |             self.left = mouse_x_in_world - mouse_x * self.zoomed_width
  85 |             self.right = mouse_x_in_world + (1 - mouse_x) * self.zoomed_width
  86 |             self.bottom = mouse_y_in_world - mouse_y * self.zoomed_height
  87 |             self.top = mouse_y_in_world + (1 - mouse_y) * self.zoomed_height
  88 | 
  89 |     def on_draw(self):
  90 |         # Initialize Projection matrix
  91 |         glMatrixMode(GL_PROJECTION)
  92 |         glLoadIdentity()
  93 | 
  94 |         # Initialize Modelview matrix
  95 |         glMatrixMode(GL_MODELVIEW)
  96 |         glLoadIdentity()
  97 |         # Save the default modelview matrix
  98 |         glPushMatrix()
  99 | 
 100 |         # Clear window with ClearColor
 101 |         glClear(GL_COLOR_BUFFER_BIT)
 102 | 
 103 |         # Set orthographic projection matrix
 104 |         glOrtho(self.left, self.right, self.bottom, self.top, 1, -1)
 105 | 
 106 |         # Draw quad
 107 |         glBegin(GL_QUADS)
 108 |         glColor3ub(0xFF, 0, 0)
 109 |         glVertex2i(10, 10)
 110 | 
 111 |         glColor3ub(0xFF, 0xFF, 0)
 112 |         glVertex2i(110, 10)
 113 | 
 114 |         glColor3ub(0, 0xFF, 0)
 115 |         glVertex2i(110, 110)
 116 | 
 117 |         glColor3ub(0, 0, 0xFF)
 118 |         glVertex2i(10, 110)
 119 |         glEnd()
 120 | 
 121 |         # Remove default modelview matrix
 122 |         glPopMatrix()
 123 | 
 124 |     def run(self):
 125 |         pyglet.app.run()
 126 | 
 127 | 
 128 | App(800, 800, resizable=True).run()

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\random_trackgen.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Generates random tracks.
  26 | Adapted from https://gym.openai.com/envs/CarRacing-v0
  27 | Author: Hongrui Zheng  
  28 | """
  29 | 
  30 | import cv2
  31 | import os
  32 | import math
  33 | import numpy as np
  34 | import shapely.geometry as shp
  35 | import matplotlib.pyplot as plt
  36 | from matplotlib.patches import Polygon
  37 | from matplotlib.collections import PatchCollection
  38 | import argparse
  39 | 
  40 | parser = argparse.ArgumentParser()
  41 | parser.add_argument("--seed", type=int, default=123, help="Seed for the numpy rng.")
  42 | parser.add_argument(
  43 |     "--num_maps", type=int, default=1, help="Number of maps to generate."
  44 | )
  45 | args = parser.parse_args()
  46 | 
  47 | np.random.seed(args.seed)
  48 | 
  49 | if not os.path.exists("maps"):
  50 |     print("Creating maps/ directory.")
  51 |     os.makedirs("maps")
  52 | if not os.path.exists("centerline"):
  53 |     print("Creating centerline/ directory.")
  54 |     os.makedirs("centerline")
  55 | 
  56 | NUM_MAPS = args.num_maps
  57 | WIDTH = 10.0
  58 | 
  59 | 
  60 | def create_track():
  61 |     CHECKPOINTS = 16
  62 |     SCALE = 6.0
  63 |     TRACK_RAD = 900 / SCALE
  64 |     TRACK_DETAIL_STEP = 21 / SCALE
  65 |     TRACK_TURN_RATE = 0.31
  66 | 
  67 |     start_alpha = 0.0
  68 | 
  69 |     # Create checkpoints
  70 |     checkpoints = []
  71 |     for c in range(CHECKPOINTS):
  72 |         alpha = 2 * math.pi * c / CHECKPOINTS + np.random.uniform(
  73 |             0, 2 * math.pi * 1 / CHECKPOINTS
  74 |         )
  75 |         rad = np.random.uniform(TRACK_RAD / 3, TRACK_RAD)
  76 |         if c == 0:
  77 |             alpha = 0
  78 |             rad = 1.5 * TRACK_RAD
  79 |         if c == CHECKPOINTS - 1:
  80 |             alpha = 2 * math.pi * c / CHECKPOINTS
  81 |             start_alpha = 2 * math.pi * (-0.5) / CHECKPOINTS
  82 |             rad = 1.5 * TRACK_RAD
  83 |         checkpoints.append((alpha, rad * math.cos(alpha), rad * math.sin(alpha)))
  84 |     road = []
  85 | 
  86 |     # Go from one checkpoint to another to create track
  87 |     x, y, beta = 1.5 * TRACK_RAD, 0, 0
  88 |     dest_i = 0
  89 |     laps = 0
  90 |     track = []
  91 |     no_freeze = 2500
  92 |     visited_other_side = False
  93 |     while True:
  94 |         alpha = math.atan2(y, x)
  95 |         if visited_other_side and alpha > 0:
  96 |             laps += 1
  97 |             visited_other_side = False
  98 |         if alpha < 0:
  99 |             visited_other_side = True
 100 |             alpha += 2 * math.pi
 101 |         while True:
 102 |             failed = True
 103 |             while True:
 104 |                 dest_alpha, dest_x, dest_y = checkpoints[dest_i % len(checkpoints)]
 105 |                 if alpha <= dest_alpha:
 106 |                     failed = False
 107 |                     break
 108 |                 dest_i += 1
 109 |                 if dest_i % len(checkpoints) == 0:
 110 |                     break
 111 |             if not failed:
 112 |                 break
 113 |             alpha -= 2 * math.pi
 114 |             continue
 115 |         r1x = math.cos(beta)
 116 |         r1y = math.sin(beta)
 117 |         p1x = -r1y
 118 |         p1y = r1x
 119 |         dest_dx = dest_x - x
 120 |         dest_dy = dest_y - y
 121 |         proj = r1x * dest_dx + r1y * dest_dy
 122 |         while beta - alpha > 1.5 * math.pi:
 123 |             beta -= 2 * math.pi
 124 |         while beta - alpha < -1.5 * math.pi:
 125 |             beta += 2 * math.pi
 126 |         prev_beta = beta
 127 |         proj *= SCALE
 128 |         if proj > 0.3:
 129 |             beta -= min(TRACK_TURN_RATE, abs(0.001 * proj))
 130 |         if proj < -0.3:
 131 |             beta += min(TRACK_TURN_RATE, abs(0.001 * proj))
 132 |         x += p1x * TRACK_DETAIL_STEP
 133 |         y += p1y * TRACK_DETAIL_STEP
 134 |         track.append((alpha, prev_beta * 0.5 + beta * 0.5, x, y))
 135 |         if laps > 4:
 136 |             break
 137 |         no_freeze -= 1
 138 |         if no_freeze == 0:
 139 |             break
 140 | 
 141 |     # Find closed loop
 142 |     i1, i2 = -1, -1
 143 |     i = len(track)
 144 |     while True:
 145 |         i -= 1
 146 |         if i == 0:
 147 |             return False
 148 |         pass_through_start = (
 149 |             track[i][0] > start_alpha and track[i - 1][0] <= start_alpha
 150 |         )
 151 |         if pass_through_start and i2 == -1:
 152 |             i2 = i
 153 |         elif pass_through_start and i1 == -1:
 154 |             i1 = i
 155 |             break
 156 |     print("Track generation: %i..%i -> %i-tiles track" % (i1, i2, i2 - i1))
 157 |     assert i1 != -1
 158 |     assert i2 != -1
 159 | 
 160 |     track = track[i1 : i2 - 1]
 161 |     first_beta = track[0][1]
 162 |     first_perp_x = math.cos(first_beta)
 163 |     first_perp_y = math.sin(first_beta)
 164 | 
 165 |     # Length of perpendicular jump to put together head and tail
 166 |     well_glued_together = np.sqrt(
 167 |         np.square(first_perp_x * (track[0][2] - track[-1][2]))
 168 |         + np.square(first_perp_y * (track[0][3] - track[-1][3]))
 169 |     )
 170 |     if well_glued_together > TRACK_DETAIL_STEP:
 171 |         return False
 172 | 
 173 |     # post processing, converting to numpy, finding exterior and interior walls
 174 |     track_xy = [(x, y) for (a1, b1, x, y) in track]
 175 |     track_xy = np.asarray(track_xy)
 176 |     track_poly = shp.Polygon(track_xy)
 177 |     track_xy_offset_in = track_poly.buffer(WIDTH)
 178 |     track_xy_offset_out = track_poly.buffer(-WIDTH)
 179 |     track_xy_offset_in_np = np.array(track_xy_offset_in.exterior)
 180 |     track_xy_offset_out_np = np.array(track_xy_offset_out.exterior)
 181 |     return track_xy, track_xy_offset_in_np, track_xy_offset_out_np
 182 | 
 183 | 
 184 | def convert_track(track, track_int, track_ext, iter):
 185 | 
 186 |     # converts track to image and saves the centerline as waypoints
 187 |     fig, ax = plt.subplots()
 188 |     fig.set_size_inches(20, 20)
 189 |     ax.plot(*track_int.T, color="black", linewidth=3)
 190 |     ax.plot(*track_ext.T, color="black", linewidth=3)
 191 |     plt.tight_layout()
 192 |     ax.set_aspect("equal")
 193 |     ax.set_xlim(-180, 300)
 194 |     ax.set_ylim(-300, 300)
 195 |     plt.axis("off")
 196 |     plt.savefig("maps/map" + str(iter) + ".png", dpi=80)
 197 | 
 198 |     map_width, map_height = fig.canvas.get_width_height()
 199 |     print("map size: ", map_width, map_height)
 200 | 
 201 |     # transform the track center line into pixel coordinates
 202 |     xy_pixels = ax.transData.transform(track)
 203 |     origin_x_pix = xy_pixels[0, 0]
 204 |     origin_y_pix = xy_pixels[0, 1]
 205 | 
 206 |     xy_pixels = xy_pixels - np.array([[origin_x_pix, origin_y_pix]])
 207 | 
 208 |     map_origin_x = -origin_x_pix * 0.05
 209 |     map_origin_y = -origin_y_pix * 0.05
 210 | 
 211 |     # convert image using cv2
 212 |     cv_img = cv2.imread("maps/map" + str(iter) + ".png", -1)
 213 |     # convert to bw
 214 |     cv_img_bw = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
 215 |     # saving to img
 216 |     cv2.imwrite("maps/map" + str(iter) + ".png", cv_img_bw)
 217 |     cv2.imwrite("maps/map" + str(iter) + ".pgm", cv_img_bw)
 218 | 
 219 |     # create yaml file
 220 |     yaml = open("maps/map" + str(iter) + ".yaml", "w")
 221 |     yaml.write("image: map" + str(iter) + ".pgm\n")
 222 |     yaml.write("resolution: 0.062500\n")
 223 |     yaml.write(
 224 |         "origin: [" + str(map_origin_x) + "," + str(map_origin_y) + ", 0.000000]\n"
 225 |     )
 226 |     yaml.write("negate: 0\noccupied_thresh: 0.45\nfree_thresh: 0.196")
 227 |     yaml.close()
 228 |     plt.close()
 229 | 
 230 |     # saving track centerline as a csv in ros coords
 231 |     waypoints_csv = open("centerline/map" + str(iter) + ".csv", "w")
 232 |     for row in xy_pixels:
 233 |         waypoints_csv.write(str(0.05 * row[0]) + ", " + str(0.05 * row[1]) + "\n")
 234 |     waypoints_csv.close()
 235 | 
 236 | 
 237 | if __name__ == "__main__":
 238 |     for i in range(NUM_MAPS):
 239 |         try:
 240 |             track, track_int, track_ext = create_track()
 241 |         except:
 242 |             print("Random generator failed, retrying")
 243 |             continue
 244 |         convert_track(track, track_int, track_ext, i)

```

`src\simulator\f1tenth_gym\gym\f110_gym\unittest\scan_sim.py`:

```py
   1 | # MIT License
   2 | 
   3 | # Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
   4 | 
   5 | # Permission is hereby granted, free of charge, to any person obtaining a copy
   6 | # of this software and associated documentation files (the "Software"), to deal
   7 | # in the Software without restriction, including without limitation the rights
   8 | # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   9 | # copies of the Software, and to permit persons to whom the Software is
  10 | # furnished to do so, subject to the following conditions:
  11 | 
  12 | # The above copyright notice and this permission notice shall be included in all
  13 | # copies or substantial portions of the Software.
  14 | 
  15 | # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  16 | # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  17 | # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  18 | # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  19 | # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  20 | # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  21 | # SOFTWARE.
  22 | 
  23 | 
  24 | """
  25 | Prototype of Utility functions and classes for simulating 2D LIDAR scans
  26 | Author: Hongrui Zheng
  27 | """
  28 | 
  29 | import numpy as np
  30 | from numba import njit
  31 | from scipy.ndimage import distance_transform_edt as edt
  32 | from PIL import Image
  33 | import os
  34 | import yaml
  35 | 
  36 | import unittest
  37 | import timeit
  38 | 
  39 | 
  40 | def get_dt(bitmap, resolution):
  41 |     """
  42 |     Distance transformation, returns the distance matrix from the input bitmap.
  43 |     Uses scipy.ndimage, cannot be JITted.
  44 | 
  45 |         Args:
  46 |             bitmap (numpy.ndarray, (n, m)): input binary bitmap of the environment, where 0 is obstacles, and 255 (or anything > 0) is freespace
  47 |             resolution (float): resolution of the input bitmap (m/cell)
  48 | 
  49 |         Returns:
  50 |             dt (numpy.ndarray, (n, m)): output distance matrix, where each cell has the corresponding distance (in meters) to the closest obstacle
  51 |     """
  52 |     dt = resolution * edt(bitmap)
  53 |     return dt
  54 | 
  55 | 
  56 | @njit(cache=True)
  57 | def xy_2_rc(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution):
  58 |     """
  59 |     Translate (x, y) coordinate into (r, c) in the matrix
  60 | 
  61 |         Args:
  62 |             x (float): coordinate in x (m)
  63 |             y (float): coordinate in y (m)
  64 |             orig_x (float): x coordinate of the map origin (m)
  65 |             orig_y (float): y coordinate of the map origin (m)
  66 |         
  67 |         Returns:
  68 |             r (int): row number in the transform matrix of the given point
  69 |             c (int): column number in the transform matrix of the given point
  70 |     """
  71 |     # translation
  72 |     x_trans = x - orig_x
  73 |     y_trans = y - orig_y
  74 | 
  75 |     # rotation
  76 |     x_rot = x_trans * orig_c + y_trans * orig_s
  77 |     y_rot = -x_trans * orig_s + y_trans * orig_c
  78 | 
  79 |     # clip the state to be a cell
  80 |     if (
  81 |         x_rot < 0
  82 |         or x_rot >= width * resolution
  83 |         or y_rot < 0
  84 |         or y_rot >= height * resolution
  85 |     ):
  86 |         c = -1
  87 |         r = -1
  88 |     else:
  89 |         c = int(x_rot / resolution)
  90 |         r = int(y_rot / resolution)
  91 | 
  92 |     return r, c
  93 | 
  94 | 
  95 | @njit(cache=True)
  96 | def distance_transform(
  97 |     x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt
  98 | ):
  99 |     """
 100 |     Look up corresponding distance in the distance matrix
 101 | 
 102 |         Args:
 103 |             x (float): x coordinate of the lookup point
 104 |             y (float): y coordinate of the lookup point
 105 |             orig_x (float): x coordinate of the map origin (m)
 106 |             orig_y (float): y coordinate of the map origin (m)
 107 | 
 108 |         Returns:
 109 |             distance (float): corresponding shortest distance to obstacle in meters
 110 |     """
 111 |     r, c = xy_2_rc(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution)
 112 |     distance = dt[r, c]
 113 |     return distance
 114 | 
 115 | 
 116 | @njit(cache=True)
 117 | def trace_ray(
 118 |     x,
 119 |     y,
 120 |     theta_index,
 121 |     sines,
 122 |     cosines,
 123 |     eps,
 124 |     orig_x,
 125 |     orig_y,
 126 |     orig_c,
 127 |     orig_s,
 128 |     height,
 129 |     width,
 130 |     resolution,
 131 |     dt,
 132 |     max_range,
 133 | ):
 134 |     """
 135 |     Find the length of a specific ray at a specific scan angle theta
 136 |     Purely math calculation and loops, should be JITted.
 137 | 
 138 |         Args:
 139 |             x (float): current x coordinate of the ego (scan) frame
 140 |             y (float): current y coordinate of the ego (scan) frame
 141 |             theta_index(int): current index of the scan beam in the scan range
 142 |             sines (numpy.ndarray (n, )): pre-calculated sines of the angle array
 143 |             cosines (numpy.ndarray (n, )): pre-calculated cosines ...
 144 | 
 145 |         Returns:
 146 |             total_distance (float): the distance to first obstacle on the current scan beam
 147 |     """
 148 | 
 149 |     # int casting, and index precal trigs
 150 |     theta_index_ = int(theta_index)
 151 |     s = sines[theta_index_]
 152 |     c = cosines[theta_index_]
 153 | 
 154 |     # distance to nearest initialization
 155 |     dist_to_nearest = distance_transform(
 156 |         x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt
 157 |     )
 158 |     total_dist = dist_to_nearest
 159 | 
 160 |     # ray tracing iterations
 161 |     while dist_to_nearest > eps and total_dist <= max_range:
 162 |         # move in the direction of the ray by dist_to_nearest
 163 |         x += dist_to_nearest * c
 164 |         y += dist_to_nearest * s
 165 | 
 166 |         # update dist_to_nearest for current point on ray
 167 |         # also keeps track of total ray length
 168 |         dist_to_nearest = distance_transform(
 169 |             x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt
 170 |         )
 171 |         total_dist += dist_to_nearest
 172 | 
 173 |     return total_dist
 174 | 
 175 | 
 176 | @njit(cache=True)
 177 | def get_scan(
 178 |     pose,
 179 |     theta_dis,
 180 |     fov,
 181 |     num_beams,
 182 |     theta_index_increment,
 183 |     sines,
 184 |     cosines,
 185 |     eps,
 186 |     orig_x,
 187 |     orig_y,
 188 |     orig_c,
 189 |     orig_s,
 190 |     height,
 191 |     width,
 192 |     resolution,
 193 |     dt,
 194 |     max_range,
 195 | ):
 196 |     """
 197 |     Perform the scan for each discretized angle of each beam of the laser, loop heavy, should be JITted
 198 | 
 199 |         Args:
 200 |             pose (numpy.ndarray(3, )): current pose of the scan frame in the map
 201 |             theta_dis (int): number of steps to discretize the angles between 0 and 2pi for look up
 202 |             fov (float): field of view of the laser scan
 203 |             num_beams (int): number of beams in the scan
 204 |             theta_index_increment (float): increment between angle indices after discretization
 205 | 
 206 |         Returns:
 207 |             scan (numpy.ndarray(n, )): resulting laser scan at the pose, n=num_beams
 208 |     """
 209 |     # empty scan array init
 210 |     scan = np.empty((num_beams,))
 211 | 
 212 |     # make theta discrete by mapping the range [-pi, pi] onto [0, theta_dis]
 213 |     theta_index = theta_dis * (pose[2] - fov / 2.0) / (2.0 * np.pi)
 214 | 
 215 |     # make sure it's wrapped properly
 216 |     theta_index = np.fmod(theta_index, theta_dis)
 217 |     while theta_index < 0:
 218 |         theta_index += theta_dis
 219 | 
 220 |     # sweep through each beam
 221 |     for i in range(0, num_beams):
 222 |         # trace the current beam
 223 |         scan[i] = trace_ray(
 224 |             pose[0],
 225 |             pose[1],
 226 |             theta_index,
 227 |             sines,
 228 |             cosines,
 229 |             eps,
 230 |             orig_x,
 231 |             orig_y,
 232 |             orig_c,
 233 |             orig_s,
 234 |             height,
 235 |             width,
 236 |             resolution,
 237 |             dt,
 238 |             max_range,
 239 |         )
 240 | 
 241 |         # increment the beam index
 242 |         theta_index += theta_index_increment
 243 | 
 244 |         # make sure it stays in the range [0, theta_dis)
 245 |         while theta_index >= theta_dis:
 246 |             theta_index -= theta_dis
 247 | 
 248 |     return scan
 249 | 
 250 | 
 251 | class ScanSimulator2D(object):
 252 |     """
 253 |     2D LIDAR scan simulator class
 254 | 
 255 |     Init params:
 256 |         num_beams (int): number of beams in the scan
 257 |         fov (float): field of view of the laser scan
 258 |         std_dev (float, default=0.01): standard deviation of the generated whitenoise in the scan
 259 |         eps (float, default=0.0001): ray tracing iteration termination condition
 260 |         theta_dis (int, default=2000): number of steps to discretize the angles between 0 and 2pi for look up
 261 |         max_range (float, default=30.0): maximum range of the laser
 262 |         seed (int, default=123): seed for random number generator for the whitenoise in scan
 263 |     """
 264 | 
 265 |     def __init__(
 266 |         self,
 267 |         num_beams,
 268 |         fov,
 269 |         std_dev=0.01,
 270 |         eps=0.0001,
 271 |         theta_dis=2000,
 272 |         max_range=30.0,
 273 |         seed=123,
 274 |     ):
 275 |         # initialization
 276 |         self.num_beams = num_beams
 277 |         self.fov = fov
 278 |         self.std_dev = std_dev
 279 |         self.eps = eps
 280 |         self.theta_dis = theta_dis
 281 |         self.max_range = max_range
 282 |         self.angle_increment = self.fov / (self.num_beams - 1)
 283 |         self.theta_index_increment = theta_dis * self.angle_increment / (2.0 * np.pi)
 284 |         self.orig_c = None
 285 |         self.orig_s = None
 286 |         self.orig_x = None
 287 |         self.orig_y = None
 288 |         self.map_height = None
 289 |         self.map_width = None
 290 |         self.map_resolution = None
 291 |         self.dt = None
 292 | 
 293 |         # white noise generator
 294 |         self.rng = np.random.default_rng(seed=seed)
 295 | 
 296 |         # precomputing corresponding cosines and sines of the angle array
 297 |         theta_arr = np.linspace(0.0, 2 * np.pi, num=theta_dis)
 298 |         self.sines = np.sin(theta_arr)
 299 |         self.cosines = np.cos(theta_arr)
 300 | 
 301 |     def set_map(self, map_path, map_ext):
 302 |         """
 303 |         Set the bitmap of the scan simulator by path
 304 | 
 305 |             Args:
 306 |                 map_path (str): path to the map yaml file
 307 |                 map_ext (str): extension (image type) of the map image
 308 | 
 309 |             Returns:
 310 |                 flag (bool): if image reading and loading is successful
 311 |         """
 312 |         # TODO: do we open the option to flip the images, and turn rgb into grayscale? or specify the exact requirements in documentation.
 313 |         # TODO: throw error if image specification isn't met
 314 | 
 315 |         # load map image
 316 |         map_img_path = os.path.splitext(map_path)[0] + map_ext
 317 |         self.map_img = np.array(
 318 |             Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM)
 319 |         )
 320 |         self.map_img = self.map_img.astype(np.float64)
 321 | 
 322 |         # grayscale -> binary
 323 |         self.map_img[self.map_img <= 128.0] = 0.0
 324 |         self.map_img[self.map_img > 128.0] = 255.0
 325 | 
 326 |         self.map_height = self.map_img.shape[0]
 327 |         self.map_width = self.map_img.shape[1]
 328 | 
 329 |         # load map yaml
 330 |         with open(map_path, "r") as yaml_stream:
 331 |             try:
 332 |                 map_metadata = yaml.safe_load(yaml_stream)
 333 |                 self.map_resolution = map_metadata["resolution"]
 334 |                 self.origin = map_metadata["origin"]
 335 |             except yaml.YAMLError as ex:
 336 |                 print(ex)
 337 | 
 338 |         # calculate map parameters
 339 |         self.orig_x = self.origin[0]
 340 |         self.orig_y = self.origin[1]
 341 |         self.orig_s = np.sin(self.origin[2])
 342 |         self.orig_c = np.cos(self.origin[2])
 343 | 
 344 |         # get the distance transform
 345 |         self.dt = get_dt(self.map_img, self.map_resolution)
 346 | 
 347 |         return True
 348 | 
 349 |     def scan(self, pose):
 350 |         """
 351 |         Perform simulated 2D scan by pose on the given map
 352 | 
 353 |             Args:
 354 |                 pose (numpy.ndarray (3, )): pose of the scan frame (x, y, theta)
 355 | 
 356 |             Returns:
 357 |                 scan (numpy.ndarray (n, )): data array of the laserscan, n=num_beams
 358 | 
 359 |             Raises:
 360 |                 ValueError: when scan is called before a map is set
 361 |         """
 362 |         if self.map_height is None:
 363 |             raise ValueError("Map is not set for scan simulator.")
 364 |         scan = get_scan(
 365 |             pose,
 366 |             self.theta_dis,
 367 |             self.fov,
 368 |             self.num_beams,
 369 |             self.theta_index_increment,
 370 |             self.sines,
 371 |             self.cosines,
 372 |             self.eps,
 373 |             self.orig_x,
 374 |             self.orig_y,
 375 |             self.orig_c,
 376 |             self.orig_s,
 377 |             self.map_height,
 378 |             self.map_width,
 379 |             self.map_resolution,
 380 |             self.dt,
 381 |             self.max_range,
 382 |         )
 383 |         noise = self.rng.normal(0.0, self.std_dev, size=self.num_beams)
 384 |         final_scan = scan + noise
 385 |         return final_scan
 386 | 
 387 |     def get_increment(self):
 388 |         return self.angle_increment
 389 | 
 390 | 
 391 | """
 392 | Unit tests for the 2D scan simulator class
 393 | Author: Hongrui Zheng
 394 | 
 395 | Test cases:
 396 |     1, 2: Comparison between generated scan array of the new simulator and the legacy C++ simulator, generated data used, MSE is used as the metric
 397 |     2. FPS test, should be greater than 500
 398 | """
 399 | 
 400 | 
 401 | class ScanTests(unittest.TestCase):
 402 |     def setUp(self):
 403 |         # test params
 404 |         self.num_beams = 1080
 405 |         self.fov = 4.7
 406 | 
 407 |         self.num_test = 10
 408 |         self.test_poses = np.zeros((self.num_test, 3))
 409 |         self.test_poses[:, 2] = np.linspace(-1.0, 1.0, num=self.num_test)
 410 | 
 411 |         # legacy gym data
 412 |         sample_scan = np.load("legacy_scan.npz")
 413 |         self.berlin_scan = sample_scan["berlin"]
 414 |         self.skirk_scan = sample_scan["skirk"]
 415 | 
 416 |     def test_map_berlin(self):
 417 |         scan_sim = ScanSimulator2D(self.num_beams, self.fov)
 418 |         new_berlin = np.empty((self.num_test, self.num_beams))
 419 |         map_path = "../../../maps/berlin.yaml"
 420 |         map_ext = ".png"
 421 |         scan_sim.set_map(map_path, map_ext)
 422 |         # scan gen loop
 423 |         for i in range(self.num_test):
 424 |             test_pose = self.test_poses[i]
 425 |             new_berlin[i, :] = scan_sim.scan(test_pose)
 426 |         diff = self.berlin_scan - new_berlin
 427 |         mse = np.mean(diff ** 2)
 428 |         # print('Levine distance test, norm: ' + str(norm))
 429 | 
 430 |         # plotting
 431 |         import matplotlib.pyplot as plt
 432 | 
 433 |         theta = np.linspace(-self.fov / 2.0, self.fov / 2.0, num=self.num_beams)
 434 |         plt.polar(theta, new_berlin[1, :], ".", lw=0)
 435 |         plt.polar(theta, self.berlin_scan[1, :], ".", lw=0)
 436 |         plt.show()
 437 | 
 438 |         self.assertLess(mse, 2.0)
 439 | 
 440 |     def test_map_skirk(self):
 441 |         scan_sim = ScanSimulator2D(self.num_beams, self.fov)
 442 |         new_skirk = np.empty((self.num_test, self.num_beams))
 443 |         map_path = "../../../maps/skirk.yaml"
 444 |         map_ext = ".png"
 445 |         scan_sim.set_map(map_path, map_ext)
 446 |         print("map set")
 447 |         # scan gen loop
 448 |         for i in range(self.num_test):
 449 |             test_pose = self.test_poses[i]
 450 |             new_skirk[i, :] = scan_sim.scan(test_pose)
 451 |         diff = self.skirk_scan - new_skirk
 452 |         mse = np.mean(diff ** 2)
 453 |         print("skirk distance test, mse: " + str(mse))
 454 | 
 455 |         # plotting
 456 |         import matplotlib.pyplot as plt
 457 | 
 458 |         theta = np.linspace(-self.fov / 2.0, self.fov / 2.0, num=self.num_beams)
 459 |         plt.polar(theta, new_skirk[1, :], ".", lw=0)
 460 |         plt.polar(theta, self.skirk_scan[1, :], ".", lw=0)
 461 |         plt.show()
 462 | 
 463 |         self.assertLess(mse, 2.0)
 464 | 
 465 |     def test_fps(self):
 466 |         # scan fps should be greater than 500
 467 |         scan_sim = ScanSimulator2D(self.num_beams, self.fov)
 468 |         map_path = "../../../maps/skirk.yaml"
 469 |         map_ext = ".png"
 470 |         scan_sim.set_map(map_path, map_ext)
 471 | 
 472 |         import time
 473 | 
 474 |         start = time.time()
 475 |         for i in range(10000):
 476 |             x_test = i / 10000
 477 |             scan = scan_sim.scan(np.array([x_test, 0.0, 0.0]))
 478 |         end = time.time()
 479 |         fps = 10000 / (end - start)
 480 |         # print('FPS test')
 481 |         # print('Elapsed time: ' + str(end-start) + ' , FPS: ' + str(1/fps))
 482 |         self.assertGreater(fps, 500.0)
 483 | 
 484 | 
 485 | def main():
 486 |     num_beams = 1080
 487 |     fov = 4.7
 488 |     # map_path = '../envs/maps/berlin.yaml'
 489 |     map_path = "/home/f1tenth-eval/tunercar/es/maps/map0.yaml"
 490 |     map_ext = ".png"
 491 |     scan_sim = ScanSimulator2D(num_beams, fov)
 492 |     scan_sim.set_map(map_path, map_ext)
 493 |     scan = scan_sim.scan(np.array([0.0, 0.0, 0.0]))
 494 | 
 495 |     # fps test
 496 |     import time
 497 | 
 498 |     start = time.time()
 499 |     for i in range(10000):
 500 |         x_test = i / 10000
 501 |         scan = scan_sim.scan(np.array([x_test, 0.0, 0.0]))
 502 |     end = time.time()
 503 |     fps = (end - start) / 10000
 504 |     print("FPS test")
 505 |     print("Elapsed time: " + str(end - start) + " , FPS: " + str(1 / fps))
 506 | 
 507 |     # visualization
 508 |     import matplotlib.pyplot as plt
 509 |     from matplotlib.animation import FuncAnimation
 510 | 
 511 |     num_iter = 100
 512 |     theta = np.linspace(-fov / 2.0, fov / 2.0, num=num_beams)
 513 |     fig = plt.figure()
 514 |     ax = fig.add_subplot(111, projection="polar")
 515 |     ax.set_ylim(0, 70)
 516 |     line, = ax.plot([], [], ".", lw=0)
 517 | 
 518 |     def update(i):
 519 |         # x_ani = i * 3. / num_iter
 520 |         theta_ani = -i * 2 * np.pi / num_iter
 521 |         x_ani = 0.0
 522 |         current_scan = scan_sim.scan(np.array([x_ani, 0.0, theta_ani]))
 523 |         print(np.max(current_scan))
 524 |         line.set_data(theta, current_scan)
 525 |         return (line,)
 526 | 
 527 |     ani = FuncAnimation(fig, update, frames=num_iter, blit=True)
 528 |     plt.show()
 529 | 
 530 | 
 531 | if __name__ == "__main__":
 532 |     # unittest.main()
 533 |     main()

```

`src\simulator\f1tenth_gym\setup.py`:

```py
   1 | from setuptools import setup
   2 | 
   3 | setup(
   4 |     name="f110_gym",
   5 |     version="0.2.1",
   6 |     author="Hongrui Zheng",
   7 |     author_email="billyzheng.bz@gmail.com",
   8 |     url="https://f1tenth.org",
   9 |     package_dir={"": "gym"},
  10 |     install_requires=[
  11 |         "gym==0.19.0",
  12 |         "numpy<=1.22.0,>=1.18.0",
  13 |         "Pillow>=9.0.1",
  14 |         "scipy>=1.7.3",
  15 |         "numba>=0.55.2",
  16 |         "pyyaml>=5.3.1",
  17 |         "pyglet<1.5",
  18 |         "pyopengl",
  19 |     ],
  20 | )

```

`src\simulator\launch\agent_template.launch`:

```launch
   1 | <?xml version="1.0"?>
   2 | <launch>
   3 | 
   4 | <!-- launch simulator on ros1 native -->
   5 | <include file="$(find f1tenth_gym_ros)/launch/gym_bridge_host.launch"/>
   6 | 
   7 | <!-- add dummy input to the ego car -->
   8 | <!-- <node pkg="f1tenth_gym_ros" type="dummy_agent_node.py" name="dummy_agent" output="screen"/> -->
   9 | 
  10 | </launch>

```

`src\simulator\launch\gym_bridge.launch`:

```launch
   1 | <?xml version="1.0"?>
   2 | <launch>
   3 |   <arg name="map" default="$(find f1tenth_gym_ros)/maps/map.yaml"/>
   4 |   <arg name="num_static_obstacles" default="5"/>
   5 | 
   6 |   <!-- Load rosparam -->
   7 |   <rosparam command="load" file="$(find f1tenth_gym_ros)/params.yaml"/>
   8 | 
   9 |   <!-- Launch a map from the maps folder-->
  10 |   <node pkg="map_server" name="map_server_sim" type="map_server" args="$(arg map)"/>
  11 | 
  12 |   <!-- Launch robot model -->
  13 |   <include file="$(find f1tenth_gym_ros)/launch/racecar_model.launch"/>
  14 | 
  15 |   <!-- Launch gym bridge node -->
  16 |   <param name="map_path" value="$(arg map)"/>
  17 |   <node pkg="f1tenth_gym_ros" name="gym_bridge" type="gym_bridge.py" output="screen">
  18 |     <param name="num_static_obstacles" value="$(arg num_static_obstacles)"/>
  19 |   </node>
  20 | 
  21 | </launch>

```

`src\simulator\launch\gym_bridge.rviz`:

```rviz
   1 | Panels:
   2 |   - Class: rviz/Displays
   3 |     Help Height: 78
   4 |     Name: Displays
   5 |     Property Tree Widget:
   6 |       Expanded:
   7 |         - /Global Options1
   8 |         - /Status1
   9 |         - /LaserScan1
  10 |       Splitter Ratio: 0.5
  11 |     Tree Height: 1079
  12 |   - Class: rviz/Selection
  13 |     Name: Selection
  14 |   - Class: rviz/Tool Properties
  15 |     Expanded:
  16 |       - /2D Pose Estimate1
  17 |       - /2D Nav Goal1
  18 |       - /Publish Point1
  19 |     Name: Tool Properties
  20 |     Splitter Ratio: 0.5886790156364441
  21 |   - Class: rviz/Views
  22 |     Expanded:
  23 |       - /Current View1
  24 |     Name: Views
  25 |     Splitter Ratio: 0.5
  26 |   - Class: rviz/Time
  27 |     Name: Time
  28 |     SyncMode: 0
  29 |     SyncSource: LaserScan
  30 | Preferences:
  31 |   PromptSaveOnExit: true
  32 | Toolbars:
  33 |   toolButtonStyle: 2
  34 | Visualization Manager:
  35 |   Class: ""
  36 |   Displays:
  37 |     - Alpha: 0.699999988079071
  38 |       Class: rviz/Map
  39 |       Color Scheme: map
  40 |       Draw Behind: false
  41 |       Enabled: true
  42 |       Name: Map
  43 |       Topic: /map
  44 |       Unreliable: false
  45 |       Use Timestamp: false
  46 |       Value: true
  47 |     - Alpha: 1
  48 |       Class: rviz/RobotModel
  49 |       Collision Enabled: false
  50 |       Enabled: true
  51 |       Links:
  52 |         All Links Enabled: true
  53 |         Expand Joint Details: false
  54 |         Expand Link Details: false
  55 |         Expand Tree: false
  56 |         Link Tree Style: Links in Alphabetic Order
  57 |         back_left_wheel:
  58 |           Alpha: 1
  59 |           Show Axes: false
  60 |           Show Trail: false
  61 |           Value: true
  62 |         back_right_wheel:
  63 |           Alpha: 1
  64 |           Show Axes: false
  65 |           Show Trail: false
  66 |           Value: true
  67 |         base_link:
  68 |           Alpha: 1
  69 |           Show Axes: false
  70 |           Show Trail: false
  71 |           Value: true
  72 |         front_left_hinge:
  73 |           Alpha: 1
  74 |           Show Axes: false
  75 |           Show Trail: false
  76 |         front_left_wheel:
  77 |           Alpha: 1
  78 |           Show Axes: false
  79 |           Show Trail: false
  80 |           Value: true
  81 |         front_right_hinge:
  82 |           Alpha: 1
  83 |           Show Axes: false
  84 |           Show Trail: false
  85 |         front_right_wheel:
  86 |           Alpha: 1
  87 |           Show Axes: false
  88 |           Show Trail: false
  89 |           Value: true
  90 |         laser_model:
  91 |           Alpha: 1
  92 |           Show Axes: false
  93 |           Show Trail: false
  94 |           Value: true
  95 |       Name: RobotModel
  96 |       Robot Description: ego_racecar/robot_description
  97 |       TF Prefix: ego_racecar
  98 |       Update Interval: 0
  99 |       Value: true
 100 |       Visual Enabled: true
 101 |     - Alpha: 1
 102 |       Class: rviz/RobotModel
 103 |       Collision Enabled: false
 104 |       Enabled: true
 105 |       Links:
 106 |         All Links Enabled: true
 107 |         Expand Joint Details: false
 108 |         Expand Link Details: false
 109 |         Expand Tree: false
 110 |         Link Tree Style: Links in Alphabetic Order
 111 |         back_left_wheel:
 112 |           Alpha: 1
 113 |           Show Axes: false
 114 |           Show Trail: false
 115 |           Value: true
 116 |         back_right_wheel:
 117 |           Alpha: 1
 118 |           Show Axes: false
 119 |           Show Trail: false
 120 |           Value: true
 121 |         base_link:
 122 |           Alpha: 1
 123 |           Show Axes: false
 124 |           Show Trail: false
 125 |           Value: true
 126 |         front_left_hinge:
 127 |           Alpha: 1
 128 |           Show Axes: false
 129 |           Show Trail: false
 130 |         front_left_wheel:
 131 |           Alpha: 1
 132 |           Show Axes: false
 133 |           Show Trail: false
 134 |           Value: true
 135 |         front_right_hinge:
 136 |           Alpha: 1
 137 |           Show Axes: false
 138 |           Show Trail: false
 139 |         front_right_wheel:
 140 |           Alpha: 1
 141 |           Show Axes: false
 142 |           Show Trail: false
 143 |           Value: true
 144 |         laser_model:
 145 |           Alpha: 1
 146 |           Show Axes: false
 147 |           Show Trail: false
 148 |           Value: true
 149 |       Name: RobotModel
 150 |       Robot Description: opp_racecar/robot_description
 151 |       TF Prefix: opp_racecar
 152 |       Update Interval: 0
 153 |       Value: true
 154 |       Visual Enabled: true
 155 |     - Alpha: 1
 156 |       Autocompute Intensity Bounds: true
 157 |       Autocompute Value Bounds:
 158 |         Max Value: 0
 159 |         Min Value: 0
 160 |         Value: true
 161 |       Axis: Z
 162 |       Channel Name: intensity
 163 |       Class: rviz/LaserScan
 164 |       Color: 255; 255; 255
 165 |       Color Transformer: AxisColor
 166 |       Decay Time: 0
 167 |       Enabled: true
 168 |       Invert Rainbow: false
 169 |       Max Color: 255; 255; 255
 170 |       Min Color: 0; 0; 0
 171 |       Name: LaserScan
 172 |       Position Transformer: XYZ
 173 |       Queue Size: 10
 174 |       Selectable: true
 175 |       Size (Pixels): 3
 176 |       Size (m): 0.05000000074505806
 177 |       Style: Flat Squares
 178 |       Topic: /scan
 179 |       Unreliable: false
 180 |       Use Fixed Frame: true
 181 |       Use rainbow: true
 182 |       Value: true
 183 |   Enabled: true
 184 |   Global Options:
 185 |     Background Color: 48; 48; 48
 186 |     Default Light: true
 187 |     Fixed Frame: map
 188 |     Frame Rate: 30
 189 |   Name: root
 190 |   Tools:
 191 |     - Class: rviz/Interact
 192 |       Hide Inactive Objects: true
 193 |     - Class: rviz/MoveCamera
 194 |     - Class: rviz/Select
 195 |     - Class: rviz/FocusCamera
 196 |     - Class: rviz/Measure
 197 |     - Class: rviz/SetInitialPose
 198 |       Theta std deviation: 0.2617993950843811
 199 |       Topic: /initialpose
 200 |       X std deviation: 0.5
 201 |       Y std deviation: 0.5
 202 |     - Class: rviz/SetGoal
 203 |       Topic: /move_base_simple/goal
 204 |     - Class: rviz/PublishPoint
 205 |       Single click: true
 206 |       Topic: /clicked_point
 207 |   Value: true
 208 |   Views:
 209 |     Current:
 210 |       Class: rviz/Orbit
 211 |       Distance: 16.213916778564453
 212 |       Enable Stereo Rendering:
 213 |         Stereo Eye Separation: 0.05999999865889549
 214 |         Stereo Focal Distance: 1
 215 |         Swap Stereo Eyes: false
 216 |         Value: false
 217 |       Field of View: 0.7853981852531433
 218 |       Focal Point:
 219 |         X: 0
 220 |         Y: 0
 221 |         Z: 0
 222 |       Focal Shape Fixed Size: true
 223 |       Focal Shape Size: 0.05000000074505806
 224 |       Invert Z Axis: false
 225 |       Name: Current View
 226 |       Near Clip Distance: 0.009999999776482582
 227 |       Pitch: 0.7797971963882446
 228 |       Target Frame: <Fixed Frame>
 229 |       Yaw: 3.325406551361084
 230 |     Saved: ~
 231 | Window Geometry:
 232 |   Displays:
 233 |     collapsed: false
 234 |   Height: 1376
 235 |   Hide Left Dock: false
 236 |   Hide Right Dock: false
 237 |   QMainWindow State: 000000ff00000000fd000000040000000000000156000004c2fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000004c2000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000004c2fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000004c2000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000006940000003efc0100000002fb0000000800540069006d0065010000000000000694000003bc00fffffffb0000000800540069006d0065010000000000000450000000000000000000000423000004c200000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
 238 |   Selection:
 239 |     collapsed: false
 240 |   Time:
 241 |     collapsed: false
 242 |   Tool Properties:
 243 |     collapsed: false
 244 |   Views:
 245 |     collapsed: false
 246 |   Width: 1684
 247 |   X: 72
 248 |   Y: 27

```

`src\simulator\launch\gym_bridge_host.launch`:

```launch
   1 | <?xml version="1.0"?>
   2 | <launch>
   3 | 
   4 |   <!-- Launch Rviz -->
   5 |   <node pkg="rviz" type="rviz" name="rviz_sim" args="-d $(find f1tenth_gym_ros)/launch/gym_bridge.rviz" output="screen"/>
   6 | 
   7 | </launch>

```

`src\simulator\launch\racecar_model.launch`:

```launch
   1 | <?xml version="1.0"?>
   2 | <launch>
   3 | 
   4 |   <!-- group for ego racecar -->
   5 |   <group ns="ego_racecar">
   6 |   <!-- Open the model file -->
   7 |   <arg name="racecar_xacro" default="$(find f1tenth_gym_ros)/ego_racecar.xacro"/>
   8 |   <param name="tf_prefix" value="ego_racecar"/>
   9 |   <param name="robot_description" command="xacro --inorder '$(arg racecar_xacro)'"/>
  10 |   <!-- Add the transformations -->
  11 |   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  12 |   </group>
  13 | 
  14 |   <!-- group for opponent racecar -->
  15 |   <group ns="opp_racecar">
  16 |   <!-- Open the model file -->
  17 |   <arg name="racecar_xacro" default="$(find f1tenth_gym_ros)/opp_racecar.xacro"/>
  18 |   <param name="tf_prefix" value="opp_racecar"/>
  19 |   <param name="robot_description" command="xacro --inorder '$(arg racecar_xacro)'"/>
  20 |   <!-- Add the transformations -->
  21 |   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  22 |   </group>
  23 | 
  24 | </launch>

```

`src\simulator\maps\backups\Spielberg.yaml`:

```yaml
   1 | image: Spielberg.png
   2 | resolution: 0.05796
   3 | origin: [-84.85359914210505, -36.30299725862132, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.45
   6 | free_thresh: 0.196

```

`src\simulator\maps\backups\skirk.yaml`:

```yaml
   1 | image: skirk.png
   2 | resolution: 0.050000
   3 | origin: [-7.801, -16.388, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196
   7 | 

```

`src\simulator\maps\backups\vegas.yaml`:

```yaml
   1 | image: vegas.png
   2 | resolution: 0.050000
   3 | origin: [-11.606540, -27.320793, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196
   7 | 

```

`src\simulator\maps\map.yaml`:

```yaml
   1 | image: map.png
   2 | resolution: 0.050000
   3 | origin: [-11.606540, -26.520793, 0.000000]
   4 | negate: 0
   5 | occupied_thresh: 0.65
   6 | free_thresh: 0.196

```

`src\simulator\msg\RaceInfo.msg`:

```msg
   1 | Header header
   2 | int32 ego_lap_count
   3 | float32 ego_elapsed_time
   4 | bool ego_collision
   5 | int32 num_obstacles
   6 | bool[] static_obstacle_collisions

```

`src\simulator\opp_racecar.xacro`:

```xacro
   1 | <?xml version="1.0"?>
   2 | 
   3 | <!-- A simple model of the racecar for rviz -->
   4 | 
   5 | <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="racecar">
   6 | 
   7 |   <xacro:property name="wheelbase" value="0.3302" />
   8 |   <xacro:property name="width" value="0.2032" />
   9 |   <xacro:property name="height" value="0.1" />
  10 |   <xacro:property name="ground_offset" value="0.04" />
  11 |   <xacro:property name="wheel_radius" value="0.0508" />
  12 |   <xacro:property name="wheel_length" value="0.0381" />
  13 |   <xacro:property name="laser_distance_from_base_link" value="0.275" />
  14 |   <xacro:property name="laser_height" value="0.05" />
  15 |   <xacro:property name="laser_radius" value="0.026" />
  16 | 
  17 |   <material name="black">
  18 |     <color rgba="0.2 0.2 0.2 1."/>
  19 |   </material>
  20 | 
  21 |   <material name="red">
  22 |     <color rgba="1. 0.57 0.1 1."/>
  23 |   </material>
  24 | 
  25 |   <link name="base_link">
  26 |     <visual>
  27 |       <origin xyz="${wheelbase/2} 0 ${ground_offset+height/2}"/>
  28 |       <geometry>
  29 |         <box size="${wheelbase} ${width} ${height}"/>
  30 |       </geometry>
  31 |       <material name="red"/>
  32 |     </visual>
  33 |   </link>
  34 | 
  35 |   <joint name="base_to_laser_model" type="fixed">
  36 |   <parent link="base_link"/>
  37 |   <child link="laser_model"/>
  38 |     <origin xyz="${laser_distance_from_base_link} 0 ${ground_offset+height+(laser_height/2)}"/>
  39 | </joint>
  40 | 
  41 |   <link name="laser_model">
  42 |     <visual>
  43 |       <geometry>
  44 |         <cylinder radius="${laser_radius}" length="${laser_height}"/>
  45 |       </geometry>
  46 |       <material name="black"/>
  47 |     </visual>
  48 |   </link>
  49 | 
  50 |   <joint name="base_to_back_left_wheel" type="fixed">
  51 |     <parent link="base_link"/>
  52 |     <child link="back_left_wheel"/>
  53 |     <origin xyz="0 ${(wheel_length+width)/2} ${wheel_radius}"/>
  54 |   </joint>
  55 | 
  56 |   <link name="back_left_wheel">
  57 |     <visual>
  58 |       <geometry>
  59 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  60 |       </geometry>
  61 |       <material name="black"/>
  62 |       <origin rpy="${pi/2} 0 0"/>
  63 |     </visual>
  64 |   </link>
  65 | 
  66 |   <joint name="base_to_back_right_wheel" type="fixed">
  67 |     <parent link="base_link"/>
  68 |     <child link="back_right_wheel"/>
  69 |     <origin xyz="0 ${-(wheel_length+width)/2} ${wheel_radius}"/>
  70 |   </joint>
  71 | 
  72 |   <link name="back_right_wheel">
  73 |     <visual>
  74 |       <geometry>
  75 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  76 |       </geometry>
  77 |       <material name="black"/>
  78 |       <origin rpy="${pi/2} 0 0"/>
  79 |     </visual>
  80 |   </link>
  81 | 
  82 |   <joint name="base_to_front_left_hinge" type="fixed">
  83 |     <parent link="base_link"/>
  84 |     <child link="front_left_hinge"/>
  85 |     <origin xyz="${wheelbase} ${(wheel_length+width)/2} ${wheel_radius}"/>
  86 |   </joint>
  87 | 
  88 |   <link name="front_left_hinge"/>
  89 | 
  90 |   <joint name="front_left_hinge_to_wheel" type="continuous">
  91 |     <parent link="front_left_hinge"/>
  92 |     <child link="front_left_wheel"/>
  93 |   </joint>
  94 | 
  95 |   <link name="front_left_wheel">
  96 |     <visual>
  97 |       <geometry>
  98 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
  99 |       </geometry>
 100 |       <material name="black"/>
 101 |       <origin rpy="${pi/2} 0 0"/>
 102 |     </visual>
 103 |   </link>
 104 | 
 105 |   <joint name="base_to_front_right_hinge" type="fixed">
 106 |     <parent link="base_link"/>
 107 |     <child link="front_right_hinge"/>
 108 |     <origin xyz="${wheelbase} ${-(wheel_length+width)/2} ${wheel_radius}"/>
 109 |   </joint>
 110 | 
 111 |   <link name="front_right_hinge"/>
 112 | 
 113 |   <joint name="front_right_hinge_to_wheel" type="continuous">
 114 |     <parent link="front_right_hinge"/>
 115 |     <child link="front_right_wheel"/>
 116 |   </joint>
 117 | 
 118 |   <link name="front_right_wheel">
 119 |     <visual>
 120 |       <geometry>
 121 |         <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
 122 |       </geometry>
 123 |       <material name="black"/>
 124 |       <origin rpy="${pi/2} 0 0"/>
 125 |     </visual>
 126 |   </link>
 127 | 
 128 | </robot>

```

`src\simulator\package.xml`:

```xml
   1 | <?xml version="1.0"?>
   2 | <package format="2">
   3 |   <name>f1tenth_gym_ros</name>
   4 |   <version>0.0.0</version>
   5 |   <description>The f1tenth_gym_ros package</description>
   6 | 
   7 |   <maintainer email="billyzheng.bz@gmail.com">Hongrui Zheng</maintainer>
   8 | 
   9 | 
  10 |   <!-- One license tag required, multiple allowed, one license per tag -->
  11 |   <!-- Commonly used license strings: -->
  12 |   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  13 |   <license>TODO</license>
  14 | 
  15 |   <buildtool_depend>catkin</buildtool_depend>
  16 |   <build_depend>message_filters</build_depend>
  17 |   <build_depend>nav_msgs</build_depend>
  18 |   <build_depend>roscpp</build_depend>
  19 |   <build_depend>rospy</build_depend>
  20 |   <build_depend>sensor_msgs</build_depend>
  21 |   <build_depend>std_msgs</build_depend>
  22 |   <build_depend>visualization_msgs</build_depend>
  23 |   <build_depend>message_generation</build_depend>
  24 |   <build_export_depend>message_filters</build_export_depend>
  25 |   <build_export_depend>nav_msgs</build_export_depend>
  26 |   <build_export_depend>roscpp</build_export_depend>
  27 |   <build_export_depend>rospy</build_export_depend>
  28 |   <build_export_depend>sensor_msgs</build_export_depend>
  29 |   <build_export_depend>std_msgs</build_export_depend>
  30 |   <build_export_depend>visualization_msgs</build_export_depend>
  31 |   <exec_depend>message_filters</exec_depend>
  32 |   <exec_depend>nav_msgs</exec_depend>
  33 |   <exec_depend>roscpp</exec_depend>
  34 |   <exec_depend>rospy</exec_depend>
  35 |   <exec_depend>sensor_msgs</exec_depend>
  36 |   <exec_depend>std_msgs</exec_depend>
  37 |   <exec_depend>visualization_msgs</exec_depend>
  38 |   <exec_depend>message_runtime</exec_depend>
  39 | 
  40 | 
  41 |   <!-- The export tag contains other, unspecified, tags -->
  42 |   <export>
  43 |     <!-- Other tools can request additional information be placed here -->
  44 | 
  45 |   </export>
  46 | </package>

```

`src\simulator\params.yaml`:

```yaml
   1 | ego_scan_topic: "/scan"
   2 | ego_odom_topic: "/odom"
   3 | opp_odom_topic: "/opp_odom"
   4 | ego_drive_topic: "/drive"
   5 | lap_count_topic: "/lap_count"
   6 | lap_time_topic: "/lap_time"
   7 | collision_topic: "/collision"
   8 | race_info_topic: "/race_info"
   9 | gym_reset_topic: "/reset_gym_env"
  10 | 
  11 | # fixed vals DO NOT CHANGE
  12 | scan_fov: 4.7
  13 | scan_beams: 1080
  14 | scan_distance_to_base_link: 0.275
  15 | 
  16 | map_path: "/f1tenth_gym/maps/map.yaml"
  17 | map_img_ext: ".png"
  18 | executable_dir: "/f1tenth_gym/build/"
  19 | 
  20 | waypoints_path: "/catkin_ws/src/f1tenth_gym_ros/maps/opp_ref_path.csv" # do use the name "opp_ref_path.csv"
  21 | # waypoints_path: "/home/honda/sim_ws/src/f1tenth_gym_ros/maps/opp_ref_path.csv" # do use the name "opp_ref_path.csv"
  22 | 
  23 | # gym initial states
  24 | ego_initial_x: 0.0
  25 | ego_initial_y: 0.0
  26 | ego_initial_theta: 0.0
  27 | 
  28 | opp_initial_x: -2.0
  29 | opp_initial_y: -1.0
  30 | opp_initial_theta: 0.0
  31 | 
  32 | seed: 42

```

`src\simulator\scripts\agent_utils.py`:

```py
   1 | import numpy as np
   2 | from numba import njit
   3 | 
   4 | 
   5 | EPSILON = 0.00000000001
   6 | 
   7 | @njit(fastmath=False, cache=True)
   8 | def get_rotation_matrix(theta):
   9 |     c, s = np.cos(theta), np.sin(theta)
  10 |     return np.array([[c, -s], [s, c]])
  11 | 
  12 | 
  13 | # finds the nearest point on the given line segment connecting start and end
  14 |     # all arguments should be numpy arrays
  15 | def nearest_point_on_line_segment(point, start, end):
  16 |     '''
  17 |     Return the nearest point along the line segment connecting start and end.
  18 | 
  19 |     >>> nearest_point_on_line_segment(np.array([0.0,0.0]), np.array([1.0,1.0]), np.array([1.0,-1.0]))
  20 |     (array([ 1.,  0.]), 0.5)
  21 | 
  22 |     >>> nearest_point_on_line_segment(np.array([0.0,0.0]), np.array([1.0,1.0]), np.array([1.0,2.0]))
  23 |     (array([ 1.,  1.]), 0.0)
  24 | 
  25 |     >>> nearest_point_on_line_segment(np.array([0.0,0.0]), np.array([1.0,-2.0]), np.array([1.0,-1.0]))
  26 |     (array([ 1., -1.]), 1.0)
  27 |     '''
  28 |     diff = start - end
  29 |     l2 = np.dot(diff, diff)
  30 |     if l2 == 0.0:
  31 |         return start, 0.0
  32 |     t = np.clip(np.dot(point - start, end - start) / l2, 0.0, 1.0)
  33 |     projection = start + t * (end - start)
  34 |     return projection, t
  35 | 
  36 | 
  37 | @njit(fastmath=False, cache=True)
  38 | def nearest_point_on_trajectory_py2(point, trajectory):
  39 |     '''
  40 |     Return the nearest point along the given piecewise linear trajectory.
  41 | 
  42 |     Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
  43 |     not be an issue so long as trajectories are not insanely long.
  44 | 
  45 |         Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)
  46 | 
  47 |     point: size 2 numpy array
  48 |     trajectory: Nx2 matrix of (x,y) trajectory waypoints
  49 |         - these must be unique. If they are not unique, a divide by 0 error will destroy the world
  50 |     '''
  51 |     diffs = trajectory[1:,:] - trajectory[:-1,:]
  52 |     l2s   = diffs[:,0]**2 + diffs[:,1]**2
  53 |     # this is equivalent to the elementwise dot product
  54 |     # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
  55 |     dots = np.empty((trajectory.shape[0]-1, ))
  56 |     for i in range(dots.shape[0]):
  57 |         dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
  58 |     t = dots / l2s
  59 |     t[t<0.0] = 0.0
  60 |     t[t>1.0] = 1.0
  61 |     # t = np.clip(dots / l2s, 0.0, 1.0)
  62 |     projections = trajectory[:-1,:] + (t*diffs.T).T
  63 |     # dists = np.linalg.norm(point - projections, axis=1)
  64 |     dists = np.empty((projections.shape[0],))
  65 |     for i in range(dists.shape[0]):
  66 |         temp = point - projections[i]
  67 |         dists[i] = np.sqrt(np.sum(temp*temp))
  68 |     min_dist_segment = np.argmin(dists)
  69 |     return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment
  70 | 
  71 | @njit(fastmath=False, cache=True)
  72 | def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False):
  73 |     ''' starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.
  74 | 
  75 |     Assumes that the first segment passes within a single radius of the point
  76 | 
  77 |     http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
  78 |     '''
  79 |     start_i = int(t)
  80 |     start_t = t % 1.0
  81 |     first_t = None
  82 |     first_i = None
  83 |     first_p = None
  84 |     trajectory = np.ascontiguousarray(trajectory)
  85 |     for i in range(start_i, trajectory.shape[0]-1):
  86 |         start = trajectory[i,:]
  87 |         end = trajectory[i+1,:]+1e-6
  88 |         V = np.ascontiguousarray(end - start)
  89 | 
  90 |         a = np.dot(V,V)
  91 |         b = 2.0*np.dot(V, start - point)
  92 |         c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
  93 |         discriminant = b*b-4*a*c
  94 | 
  95 |         if discriminant < 0:
  96 |             continue
  97 |         #   print "NO INTERSECTION"
  98 |         # else:
  99 |         # if discriminant >= 0.0:
 100 |         discriminant = np.sqrt(discriminant)
 101 |         t1 = (-b - discriminant) / (2.0*a)
 102 |         t2 = (-b + discriminant) / (2.0*a)
 103 |         if i == start_i:
 104 |             if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
 105 |                 first_t = t1
 106 |                 first_i = i
 107 |                 first_p = start + t1 * V
 108 |                 break
 109 |             if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
 110 |                 first_t = t2
 111 |                 first_i = i
 112 |                 first_p = start + t2 * V
 113 |                 break
 114 |         elif t1 >= 0.0 and t1 <= 1.0:
 115 |             first_t = t1
 116 |             first_i = i
 117 |             first_p = start + t1 * V
 118 |             break
 119 |         elif t2 >= 0.0 and t2 <= 1.0:
 120 |             first_t = t2
 121 |             first_i = i
 122 |             first_p = start + t2 * V
 123 |             break
 124 |     # wrap around to the beginning of the trajectory if no intersection is found1
 125 |     if wrap and first_p is None:
 126 |         for i in range(-1, start_i):
 127 |             start = trajectory[i % trajectory.shape[0],:]
 128 |             end = trajectory[(i+1) % trajectory.shape[0],:]+1e-6
 129 |             V = end - start
 130 | 
 131 |             a = np.dot(V,V)
 132 |             b = 2.0*np.dot(V, start - point)
 133 |             c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
 134 |             discriminant = b*b-4*a*c
 135 | 
 136 |             if discriminant < 0:
 137 |                 continue
 138 |             discriminant = np.sqrt(discriminant)
 139 |             t1 = (-b - discriminant) / (2.0*a)
 140 |             t2 = (-b + discriminant) / (2.0*a)
 141 |             if t1 >= 0.0 and t1 <= 1.0:
 142 |                 first_t = t1
 143 |                 first_i = i
 144 |                 first_p = start + t1 * V
 145 |                 break
 146 |             elif t2 >= 0.0 and t2 <= 1.0:
 147 |                 first_t = t2
 148 |                 first_i = i
 149 |                 first_p = start + t2 * V
 150 |                 break
 151 | 
 152 |     return first_p, first_i, first_t
 153 | 
 154 |     # print min_dist_segment, dists[min_dist_segment], projections[min_dist_segment]
 155 | 
 156 | @njit(fastmath=False, cache=True)
 157 | def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
 158 |     # waypoint_car = np.dot(get_rotation_matrix(-pose_theta), (lookahead_point[0:2]-position))
 159 |     # waypoint_y = waypoint_car[1]
 160 |     waypoint_y = np.dot(np.array([np.sin(-pose_theta), np.cos(-pose_theta)]), lookahead_point[0:2]-position)
 161 |     speed = lookahead_point[2]
 162 |     if np.abs(waypoint_y) < 1e-6:
 163 |         return speed, 0.
 164 |     radius = 1/(2.0*waypoint_y/lookahead_distance**2)
 165 |     steering_angle = np.arctan(wheelbase/radius)
 166 |     return speed, steering_angle
 167 | 
 168 | 
 169 | # coords: Nx2 in polar (r,theta)
 170 | # in place modifies to Nx2 (x,y)
 171 | def polar_to_euclid(coords):
 172 |     xs = ranges * np.cos(angles)
 173 |     ys = ranges * np.sin(angles)
 174 |     return (xs, ys)
 175 | 
 176 | def angular_deflection_magnitude(points):
 177 |     # https://mail.python.org/pipermail/tutor/2007-July/055178.html
 178 |     # returns a numpy array of angular deflections between consequtive
 179 |     # line segments beginning and ending at the points provided
 180 |     # contains two less angles than points, since the angular deflection for the first and last components is ill defined
 181 | 
 182 |     lines = np.zeros((points.shape[0]-1, 3))
 183 |     thetas = np.zeros(points.shape[0]-2)
 184 |     for i in range(1,points.shape[0]):
 185 |         p0 = points[i-1,:]
 186 |         p1 = points[i,:]
 187 | 
 188 |         A = p0[1] - p1[1]
 189 |         B = p1[0] - p0[0]
 190 |         C = p0[0]*p1[1] - p1[0]*p0[1]
 191 |         lines[i-1] = (A,B,C)
 192 | 
 193 |     for i in range(1, lines.shape[0]):
 194 |         A1 = lines[i-1,0]
 195 |         B1 = lines[i-1,1]
 196 |         A2 = lines[i,0]
 197 |         B2 = lines[i,1]
 198 |         bottom = (A1**2+B1**2)*(A2**2+B2**2)
 199 |         if bottom > 0:
 200 |             inner = (A1*A2 + B1*B2) / np.sqrt(bottom)
 201 |             # thetas[i-1] = np.arccos(inner)
 202 |             if np.abs(np.abs(inner) - 1.0) < EPSILON:
 203 |                 thetas[i-1] = 0.0
 204 |             else:
 205 |                 thetas[i-1] = np.arccos(inner)
 206 |     return thetas
 207 | 
 208 | def piecewise_linear_local_waypoints_polar(points):
 209 |     thetas = angular_deflection_magnitude(points)
 210 |     # # compute the polar coordinate space local coordinate frame waypoints (r,theta)
 211 |     local_points_polar = np.zeros((points.shape[0]-1, 2))
 212 |     for i in range(1, points.shape[0]-1):
 213 |         # radius
 214 |         local_points_polar[i-1,0] = np.linalg.norm(points[i,:] - points[i-1,:])
 215 |         # angle
 216 |         local_points_polar[i,1] = thetas[i-1]
 217 |     local_points_polar[-1,0] = np.linalg.norm(points[-1,:] - points[-2,:])
 218 |     return local_points_polar
 219 | 
 220 |     # local_points_cartesian = np.zeros_like(local_points_polar)
 221 |     # local_points_cartesian[:,0] = local_points_polar[:,0] * np.cos(local_points_polar[:,1])
 222 |     # local_points_cartesian[:,1] = local_points_polar[:,0] * np.sin(local_points_polar[:,1])
 223 |     # print local_points_cartesian
 224 |     # print local_points_polar
 225 | 
 226 | class AckermannModel(object):
 227 |     """ A wrapper class for useful Ackermann steering geometry related functions
 228 |     """
 229 |     def __init__(self, wheelbase):
 230 |         self.L = wheelbase
 231 | 
 232 |     def path_radius(self, steering_angle):
 233 |         ''' The radius of the path driven if a constant steering angle is applied
 234 |         '''
 235 |         return self.L / np.tan(steering_angle)
 236 | 
 237 |     def yaw_rate(self, steering_angle, speed):
 238 |         ''' Rate of change of heading with a given steering angle and speed
 239 |         '''
 240 |         if steering_angle == 0.0:
 241 |             return 0.0
 242 |         return speed / self.path_radius(steering_angle)
 243 | 
 244 |     def dx(self, speed, dt, steering_angle):
 245 |         ''' Distance traveled in the local x direction given speed and steering_angle
 246 |         '''
 247 |         if steering_angle == 0.0:
 248 |             return speed * dt
 249 |         R = self.path_radius(steering_angle)
 250 |         d = dt*speed
 251 |         dx = R*np.sin(d/R)
 252 |         return dx
 253 | 
 254 |     def dy(self, speed, dt, steering_angle):
 255 |         ''' Distance traveled in the local y direction given speed and steering_angle
 256 |         '''
 257 |         if steering_angle == 0.0:
 258 |             return 0.0
 259 |         R = self.path_radius(steering_angle)
 260 |         d = dt*speed
 261 |         dy = R*(1.0 - np.cos(d/R))
 262 |         return dy
 263 | 
 264 |     def steering_angle(self, point):
 265 |         ''' Returns the steering angle required to pass through the given point
 266 |             (in local euclidean coordinates) assuming constant steering angle is applied
 267 |         '''
 268 |         theta = np.arctan2(point[1], point[0])
 269 |         return np.arctan(2.0*self.L*np.sin(theta)/np.linalg.norm(point))
 270 | 
 271 |     def steering_angle_polar(self, polar_point):
 272 |         ''' Returns the steering angle required to pass through the given point
 273 |             (in local polar coordinates) assuming constant steering angle is applied
 274 |         '''
 275 |         theta = polar_point[1]
 276 |         radius = polar_point[0]
 277 |         return np.arctan(2.0*self.L*np.sin(theta)/radius)
 278 | 
 279 | def max_angle(min_turning_radius, radius):
 280 |     tr2 = 2.0*min_turning_radius
 281 |     if radius < tr2:
 282 |         r2 = radius*radius
 283 |         y = r2 / (2.0*min_turning_radius)
 284 |         x = np.sqrt(r2 - y*y)
 285 |         max_angle = np.arctan(y/x)
 286 |     else:
 287 |         max_angle = np.pi / 2.0
 288 |     return max_angle

```

`src\simulator\scripts\agents.py`:

```py
   1 | import numpy as np
   2 | import csv
   3 | from agent_utils import (
   4 |     get_actuation,
   5 |     nearest_point_on_trajectory_py2,
   6 |     first_point_on_trajectory_intersecting_circle,
   7 | )
   8 | 
   9 | 
  10 | class Agent(object):
  11 |     def __init__(self, csv_path):
  12 |         # TODO: load waypoints from csv
  13 |         self.waypoints = None
  14 |         self.safe_speed = 0.5
  15 | 
  16 |     def plan(self, obs):
  17 |         pass
  18 | 
  19 | 
  20 | class StaticAgent(Agent):
  21 |     def __init__(self, csv_path, seed=0):
  22 |         super().__init__(csv_path)
  23 | 
  24 |         self._rng = np.random.default_rng(seed)
  25 | 
  26 |         with open(csv_path) as f:
  27 |             wpts = [tuple(line) for line in csv.reader(f)]
  28 |             self.waypoints = np.array(
  29 |                 [
  30 |                     (
  31 |                         float(pt[0]),
  32 |                         float(pt[1]),
  33 |                         float(pt[2]),
  34 |                         float(pt[3]),
  35 |                         float(pt[4]),
  36 |                         float(pt[5]),
  37 |                     )
  38 |                     for pt in wpts
  39 |                 ]
  40 |             )
  41 | 
  42 |     def get_random_pos(self, radius) -> np.ndarray:
  43 |         # Select a random waypoint
  44 |         waypoint = self._rng.choice(self.waypoints)
  45 |         x = waypoint[0]
  46 |         y = waypoint[1]
  47 |         # Select a random point within a circle of radius `radius` centered at the waypoint
  48 |         rand_r = self._rng.uniform(0, radius)
  49 |         rand_theta = self._rng.uniform(-np.pi, np.pi)
  50 |         rand_x = x + rand_r * np.cos(rand_theta)
  51 |         rand_y = y + rand_r * np.sin(rand_theta)
  52 |         rand_yaw = self._rng.uniform(-np.pi, np.pi)
  53 | 
  54 |         return np.array([[rand_x, rand_y, rand_yaw]])
  55 | 
  56 |     def plan(self, obs):
  57 |         return 0.0, 0.0
  58 | 
  59 | 
  60 | class PurePursuitAgent(Agent):
  61 |     def __init__(self, csv_path, wheelbase):
  62 |         super(PurePursuitAgent, self).__init__(csv_path)
  63 |         self.lookahead_distance = 1.0
  64 |         self.wheelbase = wheelbase
  65 |         self.max_reacquire = 10.0
  66 |         with open(csv_path) as f:
  67 |             wpts = [tuple(line) for line in csv.reader(f)]
  68 |             self.waypoints = np.array(
  69 |                 [
  70 |                     (
  71 |                         float(pt[0]),
  72 |                         float(pt[1]),
  73 |                         float(pt[2]),
  74 |                         float(pt[3]),
  75 |                         float(pt[4]),
  76 |                         float(pt[5]),
  77 |                     )
  78 |                     for pt in wpts
  79 |                 ]
  80 |             )
  81 | 
  82 |     def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
  83 |         wpts = waypoints[:, 0:2]
  84 |         nearest_point, nearest_dist, t, i = nearest_point_on_trajectory_py2(
  85 |             position, wpts
  86 |         )
  87 |         if nearest_dist < lookahead_distance:
  88 |             lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(
  89 |                 position, lookahead_distance, wpts, i + t, wrap=True
  90 |             )
  91 |             if i2 == None:
  92 |                 return None
  93 |             current_waypoint = np.empty(waypoints[i2, :].shape)
  94 |             # x, y
  95 |             current_waypoint[0:2] = waypoints[i2, 0:2]
  96 |             # theta
  97 |             current_waypoint[3] = waypoints[i2, 3]
  98 |             # speed
  99 |             current_waypoint[2] = waypoints[i2, 2]
 100 |             return current_waypoint
 101 |         elif nearest_dist < self.max_reacquire:
 102 |             return waypoints[i, :]
 103 |         else:
 104 |             return None
 105 | 
 106 |     def plan(self, obs):
 107 |         pose_x = obs["poses_x"][1]
 108 |         pose_y = obs["poses_y"][1]
 109 |         pose_theta = obs["poses_theta"][1]
 110 |         position = np.array([pose_x, pose_y])
 111 |         lookahead_point = self._get_current_waypoint(
 112 |             self.waypoints, self.lookahead_distance, position, pose_theta
 113 |         )
 114 |         if lookahead_point is None:
 115 |             return self.safe_speed, 0.0
 116 |         speed, steering_angle = get_actuation(
 117 |             pose_theta,
 118 |             lookahead_point,
 119 |             position,
 120 |             self.lookahead_distance,
 121 |             self.wheelbase,
 122 |         )
 123 |         return speed, steering_angle

```

`src\simulator\scripts\dummy_agent_node.py`:

```py
   1 | #!/usr/bin/env python3
   2 | import rospy
   3 | from ackermann_msgs.msg import AckermannDriveStamped
   4 | from sensor_msgs.msg import LaserScan
   5 | 
   6 | class Agent(object):
   7 |     def __init__(self):
   8 |         self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
   9 |         self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
  10 | 
  11 |     def scan_callback(self, scan_msg):
  12 |         drive = AckermannDriveStamped()
  13 |         drive.drive.speed = 0.0
  14 |         self.drive_pub.publish(drive)
  15 | 
  16 | if __name__ == '__main__':
  17 |     rospy.init_node('dummy_agent')
  18 |     dummy_agent = Agent()
  19 |     rospy.spin()

```

`src\simulator\scripts\gym_bridge.py`:

```py
   1 | #!/usr/bin/env python
   2 | import rospy
   3 | from sensor_msgs.msg import LaserScan
   4 | from nav_msgs.msg import Odometry
   5 | from geometry_msgs.msg import PoseStamped
   6 | from geometry_msgs.msg import TransformStamped
   7 | from geometry_msgs.msg import Transform
   8 | from geometry_msgs.msg import Quaternion
   9 | from ackermann_msgs.msg import AckermannDriveStamped
  10 | 
  11 | from std_msgs.msg import Bool, Int32, Float32, std_msgs
  12 | from visualization_msgs.msg import MarkerArray, Marker
  13 | 
  14 | from f1tenth_gym_ros.msg import RaceInfo
  15 | from tf2_ros import transform_broadcaster
  16 | from tf.transformations import quaternion_from_euler
  17 | import numpy as np
  18 | from agents import PurePursuitAgent, StaticAgent
  19 | 
  20 | from f110_gym.envs.base_classes import Integrator
  21 | 
  22 | import gym
  23 | 
  24 | 
  25 | class GymBridge(object):
  26 |     def __init__(self):
  27 |         # get params
  28 |         self.ego_scan_topic = rospy.get_param("ego_scan_topic")
  29 |         self.ego_odom_topic = rospy.get_param("ego_odom_topic")
  30 |         self.opp_odom_topic = rospy.get_param("opp_odom_topic")
  31 |         self.ego_drive_topic = rospy.get_param("ego_drive_topic")
  32 |         self.race_info_topic = rospy.get_param("race_info_topic")
  33 |         self.reset_gym_env_topic = rospy.get_param("gym_reset_topic")
  34 | 
  35 |         self.scan_distance_to_base_link = rospy.get_param("scan_distance_to_base_link")
  36 | 
  37 |         self.map_path = rospy.get_param("map_path")
  38 |         self.map_img_ext = rospy.get_param("map_img_ext")
  39 |         print(self.map_path, self.map_img_ext)
  40 | 
  41 |         scan_fov = rospy.get_param("scan_fov")
  42 |         scan_beams = rospy.get_param("scan_beams")
  43 |         self.angle_min = -scan_fov / 2.0
  44 |         self.angle_max = scan_fov / 2.0
  45 |         self.angle_inc = scan_fov / scan_beams
  46 | 
  47 |         self.csv_path = rospy.get_param("waypoints_path")
  48 | 
  49 |         self.wheelbase = 0.3302
  50 |         # mass = 3.47
  51 |         # l_r = 0.17145
  52 |         # I_z = 0.04712
  53 |         # mu = 0.523
  54 |         # h_cg = 0.074
  55 |         # cs_f = 4.718
  56 |         # cs_r = 5.4562
  57 |         # init gym backend
  58 |         self._lf = 0.15875
  59 |         self._lr = 0.17145
  60 |         self._length = 0.58
  61 |         self._width = 0.31
  62 |         num_ego_vehicle = 1
  63 |         self.num_opponent_vehicle = 1
  64 |         # get param as private param
  65 |         self.num_static_obstacles = rospy.get_param("~num_static_obstacles")
  66 | 
  67 |         # remove extension
  68 |         map_wo_ext = self.map_path.split(".")[0]
  69 |         self.racecar_env = gym.make(
  70 |             "f110_gym:f110-v0",
  71 |             map=map_wo_ext,
  72 |             map_ext=self.map_img_ext,
  73 |             num_agents=num_ego_vehicle
  74 |             + self.num_opponent_vehicle
  75 |             + self.num_static_obstacles,
  76 |             timestep=0.01,
  77 |             integrator=Integrator.RK4,
  78 |         )
  79 | 
  80 |         # init opponent agent
  81 |         self.opp_agent = PurePursuitAgent(self.csv_path, self.wheelbase)
  82 | 
  83 |         # init static obstacles list, num is num_static_obstacles
  84 |         self.static_obstacles = []
  85 |         self.seed_rank = rospy.get_param("seed")
  86 |         for i in range(self.num_static_obstacles):
  87 |             self.static_obstacles.append(
  88 |                 StaticAgent(csv_path=self.csv_path, seed=i + self.seed_rank)
  89 |             )
  90 | 
  91 |         self.reset_env(
  92 |             num_static_obs=self.num_static_obstacles,
  93 |             num_opponent_vehicle=self.num_opponent_vehicle,
  94 |         )
  95 | 
  96 |         # keep track of latest sim state
  97 |         self.ego_scan = list(self.obs["scans"][0])
  98 | 
  99 |         # keep track of collision
 100 |         self.ego_collision = False
 101 | 
 102 |         self.static_obs_collisions = [False for _ in range(self.num_static_obstacles)]
 103 | 
 104 |         # transform broadcaster
 105 |         self.br = transform_broadcaster.TransformBroadcaster()
 106 | 
 107 |         # pubs
 108 |         self.ego_scan_pub = rospy.Publisher(
 109 |             self.ego_scan_topic, LaserScan, queue_size=1
 110 |         )
 111 |         self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=1)
 112 |         self.opp_odom_pub = rospy.Publisher(self.opp_odom_topic, Odometry, queue_size=1)
 113 |         self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=1)
 114 |         self.static_obstacles_markers_pub = rospy.Publisher(
 115 |             "/static_obstacles", MarkerArray, queue_size=1
 116 |         )
 117 |         self.collision_check_pub = rospy.Publisher(
 118 |             "/collision_check", Marker, queue_size=1
 119 |         )
 120 | 
 121 |         # subs
 122 |         self.drive_sub = rospy.Subscriber(
 123 |             self.ego_drive_topic,
 124 |             AckermannDriveStamped,
 125 |             self.drive_callback,
 126 |             queue_size=1,
 127 |         )
 128 |         self.resetenv_sub = rospy.Subscriber(
 129 |             self.reset_gym_env_topic, Bool, self.reset_gym_env_callback, queue_size=1
 130 |         )  # signal to reset gym env
 131 | 
 132 |         # Timer
 133 |         self.timer = rospy.Timer(rospy.Duration(0.004), self.timer_callback)
 134 | 
 135 |     def reset_env(self, num_static_obs, num_opponent_vehicle):
 136 |         # # init opponent agent
 137 |         # self.opp_agent = PurePursuitAgent(self.csv_path, self.wheelbase)
 138 | 
 139 |         # load ego initial states
 140 |         ego_initial_x = rospy.get_param("ego_initial_x")
 141 |         ego_initial_y = rospy.get_param("ego_initial_y")
 142 |         ego_initial_theta = rospy.get_param("ego_initial_theta")
 143 | 
 144 |         ego_initial_state = np.array(
 145 |             [[ego_initial_x, ego_initial_y, ego_initial_theta]]
 146 |         )
 147 | 
 148 |         # load opp initial states
 149 |         opp_initial_x = rospy.get_param("opp_initial_x")
 150 |         opp_initial_y = rospy.get_param("opp_initial_y")
 151 |         opp_initial_theta = rospy.get_param("opp_initial_theta")
 152 | 
 153 |         opp_initial_state = np.array(
 154 |             [[opp_initial_x, opp_initial_y, opp_initial_theta]]
 155 |         )
 156 | 
 157 |         self.static_obstacles = []
 158 |         self.seed_rank = rospy.get_param("seed")
 159 |         for i in range(self.num_static_obstacles):
 160 |             self.static_obstacles.append(
 161 |                 StaticAgent(csv_path=self.csv_path, seed=i + self.seed_rank)
 162 |             )
 163 | 
 164 |         random_static_obs_state = []
 165 |         radius = 0.1
 166 |         for i in range(num_static_obs):
 167 |             random_static_obs_state.append(
 168 |                 self.static_obstacles[i].get_random_pos(radius=radius)
 169 |             )
 170 | 
 171 |         # reset gym environment and initialize
 172 |         initial_state = np.concatenate((ego_initial_state, opp_initial_state), axis=0)
 173 |         for obs_state in random_static_obs_state:
 174 |             initial_state = np.concatenate((initial_state, obs_state), axis=0)
 175 |         self.obs, reward, self.done, info = self.racecar_env.reset(initial_state)
 176 | 
 177 |         self.ego_pose = [ego_initial_x, ego_initial_y, ego_initial_theta]
 178 |         self.ego_speed = [0.0, 0.0, 0.0]
 179 |         self.ego_steer = 0.0
 180 | 
 181 |         self.opp_pose = [ego_initial_x, ego_initial_y, ego_initial_theta]
 182 |         self.opp_speed = [0.0, 0.0, 0.0]
 183 |         self.opp_steer = 0.0
 184 | 
 185 |         self.static_obstacles_poses = []
 186 |         for pos in random_static_obs_state:
 187 |             self.static_obstacles_poses.append([pos[0][0], pos[0][1], pos[0][2]])
 188 | 
 189 |     def update_sim_state(self):
 190 |         self.ego_scan = list(self.obs["scans"][0])
 191 | 
 192 |         self.ego_pose[0] = self.obs["poses_x"][0]
 193 |         self.ego_pose[1] = self.obs["poses_y"][0]
 194 |         self.ego_pose[2] = self.obs["poses_theta"][0]
 195 |         self.ego_speed[0] = self.obs["linear_vels_x"][0]
 196 |         self.ego_speed[1] = self.obs["linear_vels_y"][0]
 197 |         self.ego_speed[2] = self.obs["ang_vels_z"][0]
 198 | 
 199 |         self.opp_pose[0] = self.obs["poses_x"][1]
 200 |         self.opp_pose[1] = self.obs["poses_y"][1]
 201 |         self.opp_pose[2] = self.obs["poses_theta"][1]
 202 |         self.opp_speed[0] = self.obs["linear_vels_x"][1]
 203 |         self.opp_speed[1] = self.obs["linear_vels_y"][1]
 204 |         self.opp_speed[2] = self.obs["ang_vels_z"][1]
 205 | 
 206 |     def drive_callback(self, drive_msg):
 207 |         # print('in drive callback')
 208 |         # TODO: trigger opp agent plan, step env, update pose and steer and vel
 209 |         ego_speed = drive_msg.drive.speed
 210 |         self.ego_steer = drive_msg.drive.steering_angle
 211 |         ego_control_vec = np.array([[self.ego_steer, ego_speed]])
 212 | 
 213 |         # activate opp_car path-tracking
 214 |         opp_speed, self.opp_steer = self.opp_agent.plan(self.obs)
 215 |         opp_control_vec = np.array([[self.opp_steer, opp_speed]])
 216 | 
 217 |         action_vec = np.concatenate((ego_control_vec, opp_control_vec), axis=0)
 218 |         for static_obs in self.static_obstacles_poses:
 219 |             action_vec = np.concatenate((action_vec, np.array([[0.0, 0.0]])), axis=0)
 220 |         self.obs, step_reward, self.done, info = self.racecar_env.step(action_vec)
 221 | 
 222 |         if self.obs["collisions"][0]:
 223 |             self.ego_collision = True
 224 |         else:
 225 |             self.ego_collision = False
 226 | 
 227 |         for i in range(self.num_static_obstacles):
 228 |             self.static_obs_collisions[i] = self.obs["collisions"][i + 2]
 229 | 
 230 |         self.update_sim_state()
 231 | 
 232 |     def reset_gym_env_callback(self, reset_gym_env_msg):
 233 |         if reset_gym_env_msg.data == True:
 234 |             print("reset gym environment")
 235 |             self.reset_env(
 236 |                 num_opponent_vehicle=self.num_opponent_vehicle,
 237 |                 num_static_obs=self.num_static_obstacles,
 238 |             )
 239 | 
 240 |             self.ego_collision = False
 241 |             self.static_obs_collisions = [
 242 |                 False for _ in range(self.num_static_obstacles)
 243 |             ]
 244 | 
 245 |     def timer_callback(self, timer):
 246 |         ts = rospy.Time.now()
 247 | 
 248 |         # pub scan
 249 |         scan = LaserScan()
 250 |         scan.header.stamp = ts
 251 |         scan.header.frame_id = "ego_racecar/laser"
 252 |         scan.angle_min = self.angle_min
 253 |         scan.angle_max = self.angle_max
 254 |         scan.angle_increment = self.angle_inc
 255 |         scan.range_min = 0.0
 256 |         scan.range_max = 30.0
 257 |         scan.ranges = self.ego_scan
 258 |         self.ego_scan_pub.publish(scan)
 259 | 
 260 |         # pub tf
 261 |         self.publish_odom(ts)
 262 |         self.publish_transforms(ts)
 263 |         self.publish_laser_transforms(ts)
 264 |         self.publish_wheel_transforms(ts)
 265 | 
 266 |         # publish static obstacles marker
 267 |         self.publish_static_obstacle_markers(ts)
 268 | 
 269 |         # pub race info
 270 |         self.publish_race_info(ts)
 271 | 
 272 |         self.publish_collision_check(ts)
 273 | 
 274 |     def publish_race_info(self, ts):
 275 |         info = RaceInfo()
 276 |         info.header.stamp = ts
 277 |         info.header.frame_id = ""
 278 |         info.ego_collision = bool(self.ego_collision)
 279 |         info.ego_elapsed_time = float(self.obs["lap_times"][0])
 280 |         info.ego_lap_count = int(self.obs["lap_counts"][0])
 281 |         info.num_obstacles = int(self.num_static_obstacles)
 282 |         for i in range(self.num_static_obstacles):
 283 |             info.static_obstacle_collisions.append(bool(self.static_obs_collisions[i]))
 284 |         self.info_pub.publish(info)
 285 | 
 286 |     def publish_odom(self, ts):
 287 |         ego_odom = Odometry()
 288 |         ego_odom.header.stamp = ts
 289 |         ego_odom.header.frame_id = "/map"
 290 |         ego_odom.child_frame_id = "ego_racecar/base_link"
 291 |         ego_odom.pose.pose.position.x = self.ego_pose[0]
 292 |         ego_odom.pose.pose.position.y = self.ego_pose[1]
 293 |         ego_quat = quaternion_from_euler(0.0, 0.0, self.ego_pose[2])
 294 |         ego_odom.pose.pose.orientation.x = ego_quat[0]
 295 |         ego_odom.pose.pose.orientation.y = ego_quat[1]
 296 |         ego_odom.pose.pose.orientation.z = ego_quat[2]
 297 |         ego_odom.pose.pose.orientation.w = ego_quat[3]
 298 |         ego_odom.twist.twist.linear.x = self.ego_speed[0]
 299 |         ego_odom.twist.twist.linear.y = self.ego_speed[1]
 300 |         ego_odom.twist.twist.angular.z = self.ego_speed[2]
 301 |         self.ego_odom_pub.publish(ego_odom)
 302 | 
 303 |         opp_odom = Odometry()
 304 |         opp_odom.header.stamp = ts
 305 |         opp_odom.header.frame_id = "/map"
 306 |         opp_odom.child_frame_id = "opp_racecar/base_link"
 307 |         opp_odom.pose.pose.position.x = self.opp_pose[0]
 308 |         opp_odom.pose.pose.position.y = self.opp_pose[1]
 309 |         opp_quat = quaternion_from_euler(0.0, 0.0, self.opp_pose[2])
 310 |         opp_odom.pose.pose.orientation.x = opp_quat[0]
 311 |         opp_odom.pose.pose.orientation.y = opp_quat[1]
 312 |         opp_odom.pose.pose.orientation.z = opp_quat[2]
 313 |         opp_odom.pose.pose.orientation.w = opp_quat[3]
 314 |         opp_odom.twist.twist.linear.x = self.opp_speed[0]
 315 |         opp_odom.twist.twist.linear.y = self.opp_speed[1]
 316 |         opp_odom.twist.twist.angular.z = self.opp_speed[2]
 317 |         self.opp_odom_pub.publish(opp_odom)
 318 | 
 319 |     def publish_transforms(self, ts):
 320 |         ego_t = Transform()
 321 |         # center to baselink
 322 |         ego_t.translation.x = self.ego_pose[0] - self._lr * np.cos(self.ego_pose[2])
 323 |         ego_t.translation.y = self.ego_pose[1] - self._lr * np.sin(self.ego_pose[2])
 324 |         ego_t.translation.z = 0.0
 325 |         ego_quat = quaternion_from_euler(0.0, 0.0, self.ego_pose[2])
 326 |         ego_t.rotation.x = ego_quat[0]
 327 |         ego_t.rotation.y = ego_quat[1]
 328 |         ego_t.rotation.z = ego_quat[2]
 329 |         ego_t.rotation.w = ego_quat[3]
 330 | 
 331 |         ego_ts = TransformStamped()
 332 |         ego_ts.transform = ego_t
 333 |         ego_ts.header.stamp = ts
 334 |         ego_ts.header.frame_id = "/map"
 335 |         ego_ts.child_frame_id = "ego_racecar/base_link"
 336 | 
 337 |         opp_t = Transform()
 338 |         opp_t.translation.x = self.opp_pose[0]
 339 |         opp_t.translation.y = self.opp_pose[1]
 340 |         opp_t.translation.z = 0.0
 341 |         opp_quat = quaternion_from_euler(0.0, 0.0, self.opp_pose[2])
 342 |         opp_t.rotation.x = opp_quat[0]
 343 |         opp_t.rotation.y = opp_quat[1]
 344 |         opp_t.rotation.z = opp_quat[2]
 345 |         opp_t.rotation.w = opp_quat[3]
 346 | 
 347 |         opp_ts = TransformStamped()
 348 |         opp_ts.transform = opp_t
 349 |         opp_ts.header.stamp = ts
 350 |         opp_ts.header.frame_id = "/map"
 351 |         opp_ts.child_frame_id = "opp_racecar/base_link"
 352 | 
 353 |         self.br.sendTransform(ego_ts)
 354 |         self.br.sendTransform(opp_ts)
 355 | 
 356 |         # publish static obstacles
 357 |         for i, pos in enumerate(self.static_obstacles_poses):
 358 |             obs_t = Transform()
 359 |             obs_t.translation.x = pos[0] - self._lr * np.cos(pos[2])
 360 |             obs_t.translation.y = pos[1] - self._lr * np.sin(pos[2])
 361 |             obs_t.translation.z = 0.0
 362 |             obs_quat = quaternion_from_euler(0.0, 0.0, pos[2])
 363 |             obs_t.rotation.x = obs_quat[0]
 364 |             obs_t.rotation.y = obs_quat[1]
 365 |             obs_t.rotation.z = obs_quat[2]
 366 |             obs_t.rotation.w = obs_quat[3]
 367 | 
 368 |             obs_ts = TransformStamped()
 369 |             obs_ts.transform = obs_t
 370 |             obs_ts.header.stamp = ts
 371 |             obs_ts.header.frame_id = "/map"
 372 |             obs_ts.child_frame_id = "static_obstacle_" + str(i) + "/base_link"
 373 | 
 374 |             self.br.sendTransform(obs_ts)
 375 | 
 376 |     def publish_wheel_transforms(self, ts):
 377 |         ego_wheel_ts = TransformStamped()
 378 |         ego_wheel_quat = quaternion_from_euler(0.0, 0.0, self.ego_steer)
 379 |         ego_wheel_ts.transform.rotation.x = ego_wheel_quat[0]
 380 |         ego_wheel_ts.transform.rotation.y = ego_wheel_quat[1]
 381 |         ego_wheel_ts.transform.rotation.z = ego_wheel_quat[2]
 382 |         ego_wheel_ts.transform.rotation.w = ego_wheel_quat[3]
 383 |         ego_wheel_ts.header.stamp = ts
 384 |         ego_wheel_ts.header.frame_id = "ego_racecar/front_left_hinge"
 385 |         ego_wheel_ts.child_frame_id = "ego_racecar/front_left_wheel"
 386 |         self.br.sendTransform(ego_wheel_ts)
 387 |         ego_wheel_ts.header.frame_id = "ego_racecar/front_right_hinge"
 388 |         ego_wheel_ts.child_frame_id = "ego_racecar/front_right_wheel"
 389 |         self.br.sendTransform(ego_wheel_ts)
 390 | 
 391 |         opp_wheel_ts = TransformStamped()
 392 |         opp_wheel_quat = quaternion_from_euler(0.0, 0.0, self.opp_steer)
 393 |         opp_wheel_ts.transform.rotation.x = opp_wheel_quat[0]
 394 |         opp_wheel_ts.transform.rotation.y = opp_wheel_quat[1]
 395 |         opp_wheel_ts.transform.rotation.z = opp_wheel_quat[2]
 396 |         opp_wheel_ts.transform.rotation.w = opp_wheel_quat[3]
 397 |         opp_wheel_ts.header.stamp = ts
 398 |         opp_wheel_ts.header.frame_id = "opp_racecar/front_left_hinge"
 399 |         opp_wheel_ts.child_frame_id = "opp_racecar/front_left_wheel"
 400 |         self.br.sendTransform(opp_wheel_ts)
 401 |         opp_wheel_ts.header.frame_id = "opp_racecar/front_right_hinge"
 402 |         opp_wheel_ts.child_frame_id = "opp_racecar/front_right_wheel"
 403 |         self.br.sendTransform(opp_wheel_ts)
 404 | 
 405 |     def publish_laser_transforms(self, ts):
 406 |         ego_scan_ts = TransformStamped()
 407 |         ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
 408 |         ego_scan_ts.transform.rotation.w = 1.0
 409 |         ego_scan_ts.header.stamp = ts
 410 |         # TODO: check frame names
 411 |         ego_scan_ts.header.frame_id = "ego_racecar/base_link"
 412 |         ego_scan_ts.child_frame_id = "ego_racecar/laser"
 413 |         self.br.sendTransform(ego_scan_ts)
 414 | 
 415 |         opp_scan_ts = TransformStamped()
 416 |         opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
 417 |         opp_scan_ts.transform.rotation.w = 1.0
 418 |         opp_scan_ts.header.stamp = ts
 419 |         # TODO: check frame names
 420 |         opp_scan_ts.header.frame_id = "opp_racecar/base_link"
 421 |         opp_scan_ts.child_frame_id = "opp_racecar/laser"
 422 |         self.br.sendTransform(opp_scan_ts)
 423 | 
 424 |     def publish_static_obstacle_markers(self, ts):
 425 |         marker_array = MarkerArray()
 426 |         for i, pos in enumerate(self.static_obstacles_poses):
 427 |             # baselink to center of obstacle
 428 |             pos_center_x = 0.5 * (self._lr + self._lr)
 429 |             pos_center_y = 0.0
 430 | 
 431 |             marker = Marker()
 432 |             marker.header.frame_id = "static_obstacle_" + str(i) + "/base_link"
 433 |             # marker.header.frame_id = "map"
 434 |             marker.header.stamp = ts
 435 |             marker.ns = "static_obstacles"
 436 |             marker.id = i
 437 |             marker.type = Marker.CUBE
 438 |             marker.action = Marker.ADD
 439 |             marker.pose.position.x = pos_center_x
 440 |             marker.pose.position.y = pos_center_y
 441 |             marker.pose.position.z = 0.6
 442 |             quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
 443 |             marker.pose.orientation.x = quaternion[0]
 444 |             marker.pose.orientation.y = quaternion[1]
 445 |             marker.pose.orientation.z = quaternion[2]
 446 |             marker.pose.orientation.w = quaternion[3]
 447 |             marker.scale.x = self._length
 448 |             marker.scale.y = self._width
 449 |             marker.scale.z = 2.0
 450 |             marker.color.a = 1.0
 451 |             marker.color.r = 0.5
 452 |             marker.color.g = 0.5
 453 |             marker.color.b = 0.5
 454 | 
 455 |             marker_array.markers.append(marker)
 456 |         self.static_obstacles_markers_pub.publish(marker_array)
 457 | 
 458 |     def publish_collision_check(self, ts):
 459 |         marker = Marker()
 460 |         marker.header.frame_id = "map"
 461 |         marker.header.stamp = ts
 462 |         marker.ns = "collision_check"
 463 |         marker.id = 0
 464 |         marker.type = Marker.SPHERE
 465 |         marker.action = Marker.ADD
 466 |         marker.pose.position.x = self.ego_pose[0]
 467 |         marker.pose.position.y = self.ego_pose[1]
 468 |         marker.pose.position.z = 0.3
 469 |         marker.pose.orientation.w = 1.0
 470 |         marker.scale.x = 0.2
 471 |         marker.scale.y = 0.2
 472 |         marker.scale.z = 0.2
 473 |         marker.color.a = 0.7
 474 |         if self.ego_collision:
 475 |             # red
 476 |             marker.color.r = 1.0
 477 |             marker.color.g = 0.0
 478 |             marker.color.b = 0.0
 479 |         else:
 480 |             # green
 481 |             marker.color.r = 0.0
 482 |             marker.color.g = 1.0
 483 |             marker.color.b = 0.0
 484 |         self.collision_check_pub.publish(marker)
 485 | 
 486 | 
 487 | if __name__ == "__main__":
 488 |     rospy.init_node("gym_bridge")
 489 |     gym_bridge = GymBridge()
 490 |     rospy.spin()

```

`src\simulator\start.sh`:

```sh
   1 | #!/bin/bash
   2 | 
   3 | NUM_STATIC_OBS=$1
   4 | 
   5 | if [ -z "$NUM_STATIC_OBS" ]
   6 | then
   7 |     NUM_STATIC_OBS=5
   8 | fi
   9 | 
  10 | source /catkin_ws/devel/setup.bash
  11 | 
  12 | roslaunch f1tenth_gym_ros gym_bridge.launch num_static_obstacles:=${NUM_STATIC_OBS}

```