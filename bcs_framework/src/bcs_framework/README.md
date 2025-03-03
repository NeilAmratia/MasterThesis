# Body Comfort System Framework

## Copyright and Ownership
© 2025 Institute for Industrial Automation System  
University of Stuttgart, Germany  
All rights reserved.

## Authors
- Maintained by: Institut für Automatisierungstechnik und Softwaresysteme
- Contact: [st184805@stud.uni-stuttgart.de]

## Overview
This framework allows you to create base features and apply deltas (modifications) to them dynamically. It provides a flexible architecture for feature development and modification without changing the original implementation.

## Prerequisites
- ROS2 Humble
- CMake 3.8+
- C++17 compiler
- colcon
- Visual Studio Code (recommended)

## Quick Start

## Installation
1. Create ROS2 workspace:
--bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository-url>

2. Install dependencies:
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

## Usage
1. Generate Feature Model:
ros2 run bcs_framework feature_model_generator

2. Create Framework Implementation
ros2 run bcs_framework create_framework

## Project Structure
bcs_framework/
├── base_features/           # Base feature implementations
│   ├── DoorSystem.hpp
│   └── README.md
├── deltas/                 # Delta modifications
│   ├── CLS.toml
│   ├── Automatic_PW.toml
│   └── README.md
├── feature_configs/        # Feature configurations
│   ├── DoorSystem.toml
│   └── README.md
└── framework_core/         # Core framework files
    ├── delta_processor.cpp
    ├── feature_model_generator.cpp
    └── framework_manager.cpp

## Directory Structure
- `base_features/`: Contains all base feature implementations
- `deltas/`: Contains all delta modifications for base features
- `feature_configs/`: Contains configuration files for each feature
- `framework_core/`: Contains core framework files (not to be modified)
- `build/`: Contains build outputs
- `input_config`: Contains the default input_configuration file 
- `generated_feature_model`: Conatins the generated feature model

## Workflow
1. Base Feature Development
- Create base implementation in base_features
- Add delta markers for modification points
- Document feature interfaces

2. Delta Creation
- Define deltas in TOML files under deltas
- Specify modifications for each feature
- Map features to deltas in configuration

3. Feature Configuration
- Configure feature relationships
- Set dependencies and constraints
- Define feature model structure

4. Framework Generation
- Generate feature model
- Process delta modifications
- Create final implementation


## Adding New Features
1. Create a new base feature in `base_features/`
2. Add delta markers in `base_features/`
3. Create deltas in `deltas/`
4. Add configuration in `feature_configs/`
5. Rebuild framework

## Testing
colcon test --packages-select bcs_framework

## Visual Studio Code Integration
1. Install ROS extension
2. Configure workspace
3. Use integrated terminal

## Troubleshooting
1. Build Issues:
   # Clean build
   rm -rf build/ install/
   colcon build --packages-select bcs_framework

2. Runtime Issues
   # Source setup
   source install/setup.bash
   # Check ROS2 environment
   printenv | grep -i ROS

## Documentation
See individual README files in each directory for detailed documentation.
