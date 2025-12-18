# GitHub Copilot Instructions for ed_perception

## Repository Overview

This repository contains **ed_perception**, a ROS (Robot Operating System) package for classifying entities based on their attached RGBD (RGB + Depth) measurements. It is part of the TU/e Robotics software stack for robot perception and object recognition.

## Technology Stack

- **ROS Noetic**: Robot Operating System framework
- **C++**: Primary language for perception plugins and tools
- **Python**: Used for ROS nodes and utility scripts
- **CMake/catkin**: Build system
- **OpenCV**: Computer vision library
- **RGBD**: RGB-D image processing
- **geolib2**: Geometry library from TU/e

## Repository Structure

```
ed_perception/
├── ed_perception/          # Main package
│   ├── plugins/           # Perception plugins (C++)
│   ├── src/               # Source files for tools
│   ├── scripts/           # Python scripts and ROS nodes
│   ├── CMakeLists.txt     # Build configuration
│   └── package.xml        # ROS package manifest
└── ed_perception_msgs/    # ROS message definitions
```

## Build Instructions

This is a catkin package. To build:

```bash
cd <catkin_workspace>
catkin build ed_perception
```

## Testing

Testing is performed via catkin test infrastructure:

```bash
catkin build ed_perception --catkin-make-args run_tests
```

Linting is done with catkin_lint_cmake (enabled when `CATKIN_ENABLE_TESTING` is set).

## Coding Standards

### C++ Code

- **Compiler warnings**: Code must compile with `-Wall -Werror=all -Wextra -Werror=extra`
- **Style**: Follow ROS C++ style guide conventions
- **Error handling**: Properly handle ROS console logging via rosconsole_bridge
- **Dependencies**: Minimize external dependencies; use existing ROS/TU/e libraries when possible

### Python Code

- **Version**: Python 2.7 compatible (ROS Noetic standard)
- **Shebang**: Use `#!/usr/bin/env python` for scripts
- **ROS nodes**: Follow ROS Python node conventions
- **Installation**: Python scripts must be listed in `catkin_install_python()` in CMakeLists.txt

### General Guidelines

- Keep changes minimal and focused
- Maintain backward compatibility with existing ROS APIs
- Update documentation when changing public interfaces
- Test changes with actual ROS runtime environment when possible

## Key Concepts

### Perception Plugins

Perception plugins are implemented as shared libraries that integrate with the ED (Environment Description) framework. They process RGBD sensor data to classify and recognize objects.

### RGBD Images

The package works with RGBD images that contain:
- Color image (RGB)
- Depth image
- Metadata (timestamp, 6D pose)

### Annotation and Training

The repository includes tools for:
- **annotation-gui**: Manual annotation of RGBD images
- **store_segments**: Extracting annotated segments for training
- **train-perception**: Training perception models (deprecated)
- **test-perception**: Testing perception models (deprecated)

## Dependencies

### Build Dependencies

- ed
- ed_perception_msgs
- ed_sensor_integration
- geolib2
- image_recognition_msgs
- rgbd
- rosconsole_bridge
- roscpp
- tue_config
- tue_filesystem
- OpenCV

### Runtime Dependencies

- ed_object_models
- std_srvs
- robocup_knowledge (for knowledge-based object types)

## CI/CD

- GitHub Actions workflow uses TU/e CI infrastructure
- Workflow runs on push, pull_request, and workflow_dispatch
- Runs catkin_lint_cmake for code quality checks
- Uses custom TU/e robotics CI actions

## Common Tasks

### Adding a New Perception Plugin

1. Create plugin source files in `ed_perception/plugins/`
2. Implement the plugin interface from the ED framework
3. Update CMakeLists.txt to build the plugin library
4. Add plugin configuration to `plugins.xml`
5. Test with actual ROS environment

### Adding a New Tool

1. Create executable source in `ed_perception/src/`
2. Link against necessary libraries in CMakeLists.txt
3. Add installation target in CMakeLists.txt
4. Document usage in README.md if user-facing

### Adding a Python Script

1. Create script in `ed_perception/scripts/`
2. Add shebang `#!/usr/bin/env python`
3. Add to `catkin_install_python()` in CMakeLists.txt
4. Ensure ROS node initialization follows conventions

## Notes for Copilot

- This is a specialized robotics perception package; changes should maintain ROS compatibility
- Do not modify working perception algorithms unless fixing a bug
- Respect the catkin build system conventions
- Test changes require ROS runtime environment (may not be available in CI)
- Documentation changes should reflect actual implementation
- Follow existing code patterns for consistency with the TU/e Robotics codebase
