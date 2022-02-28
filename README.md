# PMOD-ROS2

## Requirement

- NVIDIA-Driver `>=418.81.07`
- Docker `>=19.03`
- NVIDIA-Docker2

## Preparation

1. ```bash
    git clone https://github.com/shikishima-TasakiLab/pmod-ros1.git
    ```

1. Place a trained Torch Script model in `./model`.

## Docker Image

- pull

    ```bash
    docker pull shikishimatasakilab/pmod-ros2:amd64-torch1.7
    ```

- build

    ```bash
    ./docker/build.foxy.amd64.sh
    ```

## Start a Docker Container

1. Start a Docker container with the following command.
    ```bash
    ./docker/run.sh
    ```

1. Build source code.
    ```bash
    colcon build
    source /workspace/install/setup.bash
    ```

## Launch

1. Launch the ROS nodes with the following command.
    ```bash
    ros2 launch pmod_ros 5class.py
    ```

## PMOD

### ROS Launch

```python
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('pmod_ros'), 'config')

    return LaunchDescription([
        Node(
            namespace='pmod',
            package='pmod_ros',
            executable='pmod',
            parameters=[
                {'hz': 5.0},
                {'sub_queue_size': 10},
                {'pub_queue_size': 2},
                {'use_optical_frame': False},
                {'pmod_config_path': os.path.join(config_dir, 'pmod-5class.yaml')},
                {'use_sync': False},
                {'pub_seg_id': True},
                {'pub_seg_color': False},
                {'pub_depth': True},
                {'pub_points_ground': False},
                {'pub_points_no_ground': False},
                {'pub_points_static': False},
                {'pub_points_dynamic': True},
            ],
        ),
    ])
```

## dynamic2noground

### ROS Launch

```python
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            namespace='pmod',
            package='pmod_ros',
            executable='dynamic2noground',
            parameters=[
                {'init_maps': [
                    "/workspace/src/pmod_ros/maps/noground.pcd",
                ]},
                {'init_frame_id': 'map'},
                {'radius': 5.0},
            ],
            remappings=[
                ('points_no_ground', '/points_no_ground')
            ]
        ),
    ])
```
