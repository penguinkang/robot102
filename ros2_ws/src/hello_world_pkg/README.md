# ROS2 Hello World Package

A simple ROS2 package demonstrating basic publisher-subscriber communication using Python.

## Overview

This package contains:
- **Publisher Node**: Publishes "Hello World: {counter}" messages to `/hello_topic` at 1 Hz
- **Subscriber Node**: Subscribes to `/hello_topic` and prints received messages

## Prerequisites

### Install ROS2 on macOS

ROS2 installation on macOS can be done in several ways:

#### Option 1: Using Homebrew (Recommended if available)
```bash
# Check if ROS2 is available via Homebrew
brew search ros

# If available, install (e.g., for Humble or Iron)
brew install ros2
```

#### Option 2: Using Docker (Easiest)
```bash
# Pull ROS2 Docker image (Humble Hawksbill - LTS)
docker pull osrf/ros:humble-desktop

# Run container with workspace mounted
cd /Users/bkang/work/git/robot102/ros2_ws
docker run -it --rm \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:humble-desktop \
  /bin/bash
```

#### Option 3: Build from Source
Follow the official ROS2 documentation for building from source:
https://docs.ros.org/en/humble/Installation/macOS-Development-Setup.html

## Building the Package

Once ROS2 is installed:

```bash
# Navigate to workspace
cd /Users/bkang/work/git/robot102/ros2_ws

# Source ROS2 (adjust path based on your installation)
source /opt/ros/humble/setup.bash  # Linux
# or
source /opt/homebrew/opt/ros2/setup.bash  # macOS Homebrew (if applicable)
# or if in Docker, it's already sourced

# Build the package
colcon build --packages-select hello_world_pkg

# Source the workspace
source install/setup.bash
```

## Running the Nodes

### Terminal 1: Run Publisher
```bash
cd /Users/bkang/work/git/robot102/ros2_ws
source install/setup.bash
ros2 run hello_world_pkg publisher
```

**Expected output:**
```
[INFO] [hello_world_publisher]: Hello World Publisher has been started
[INFO] [hello_world_publisher]: Publishing: "Hello World: 0"
[INFO] [hello_world_publisher]: Publishing: "Hello World: 1"
[INFO] [hello_world_publisher]: Publishing: "Hello World: 2"
...
```

### Terminal 2: Run Subscriber
```bash
cd /Users/bkang/work/git/robot102/ros2_ws
source install/setup.bash
ros2 run hello_world_pkg subscriber
```

**Expected output:**
```
[INFO] [hello_world_subscriber]: Hello World Subscriber has been started
[INFO] [hello_world_subscriber]: I heard: "Hello World: 0"
[INFO] [hello_world_subscriber]: I heard: "Hello World: 1"
[INFO] [hello_world_subscriber]: I heard: "Hello World: 2"
...
```

## Testing ROS2 Communication

With both nodes running, you can inspect the communication:

```bash
# List all active topics
ros2 topic list

# Show topic information
ros2 topic info /hello_topic

# Echo messages from the topic
ros2 topic echo /hello_topic

# Show topic publish rate
ros2 topic hz /hello_topic

# List all running nodes
ros2 node list

# Show node information
ros2 node info /hello_world_publisher
ros2 node info /hello_world_subscriber
```

## Package Structure

```
hello_world_pkg/
├── hello_world_pkg/           # Python package
│   ├── __init__.py
│   ├── publisher_node.py      # Publisher implementation
│   └── subscriber_node.py     # Subscriber implementation
├── resource/                   # Package resources
│   └── hello_world_pkg
├── package.xml                 # Package manifest
├── setup.py                    # Python package setup
├── setup.cfg                   # Installation configuration
└── README.md                   # This file
```

## Troubleshooting

### ROS2 not found
- Make sure ROS2 is installed
- Source the ROS2 setup file: `source /opt/ros/<distro>/setup.bash`
- If using Docker, ensure you're inside the container

### Package not found after building
- Make sure you sourced the workspace: `source install/setup.bash`
- Rebuild with: `colcon build --packages-select hello_world_pkg`

### No messages received
- Ensure both nodes are running in separate terminals
- Check if topic exists: `ros2 topic list`
- Verify both nodes are on the same ROS_DOMAIN_ID (default is 0)

## Learning Resources

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Python Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS2 Concepts](https://docs.ros.org/en/humble/Concepts.html)

## License

Apache License 2.0
