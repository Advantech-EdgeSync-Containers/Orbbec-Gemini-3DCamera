# **Orbbec ROS 2 Demonstration Container**

> A prebuilt Docker container based on Ubuntu 22.04, providing an all-in-one ROS 2 environment for Orbbec depth camera demonstration.

#### Table of Contents

- [Suggested Container Name on Container Catalog](#suggested-container-name-on-container-catalog)

- [Container Functional Overview](#container-functional-overview)

- [Container Key Features](#container-key-features)

- [Supported Host Device List](#supported-host-device-list)

- [Prerequisite Software on the Host OS (Required)](#prerequisite-software-on-the-host-os-required)

- [Optional: GPU Acceleration Support](#optional-gpu-acceleration-support)

- [Software Components Inside the Container Image](#software-components-inside-the-container-image)

- [Quick Start Guide](#quick-start-guide)

- [Demonstration Images](#demonstration-images)

## Suggested Container Name on Container Catalog

`orbbec-containers/orbbec-ros2-3dcamera:v1.0.0-dev`  
(Recommended format: `vendor/image_name:tag`)

## Container Functional Overview

This container provides a complete demonstration environment for Orbbec depth cameras using ROS 2. It includes:

- Orbbec camera drivers

- Functional sample packages

- Pre-configured RViz2 visualization

- Udev rules and system setup scripts for camera detection

Designed for quick testing, development, and evaluation of Orbbec cameras on any Linux host with Docker installed.

## Container Key Features

- Pre-installed ROS 2 (Humble)

- Built-in OrbbecSDK ROS 2 wrapper

- Supports RViz2 visualization

- Includes RGB/Depth/IR/D2C/PointCloud demos

- Launch-ready functional AMR-style example

- Works out of the box with supported USB-connected Orbbec devices

## Supported Host Device List

| Platform | Compatibility |
| -------- | ------------- |
| x86_64   | Tested        |

## Prerequisite Software on the Host OS (Required)

Before running the container, ensure that your host system has the following essential software installed:

- **Docker Engine**  
  Version 20.10 or newer is recommended for best compatibility.  
  Installation guide: https://docs.docker.com/engine/install/

- **Docker Compose**  
  Required for managing multi-container setups.  
  Installation guide: https://docs.docker.com/compose/install/

---

## Optional: GPU Acceleration Support

If your system has an NVIDIA GPU and you wish to enhance the performance of visual tools such as **RViz2** during demo execution, install the following optional components:

- **NVIDIA GPU Driver**  
  The official NVIDIA GPU driver must be installed on the host to enable GPU hardware acceleration.  
  Ensure the driver version matches your GPU model and is compatible with the NVIDIA Container Toolkit.  
  Installation guide: [CUDA Installation Guide for Linux](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html)

- **NVIDIA Container Toolkit**  
  Enables GPU access inside Docker containers. This toolkit allows Docker to interface with the NVIDIA driver and utilize GPU resources.  
  Installation guide: [NVIDIA Container Toolkit Install Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

>  **Note**: Installing GPU support is not required for running the container, but it can **significantly improve the RViz2 visualization experience** when rendering complex scenes.**This project is preconfigured to enable GPU support by default**. If your system does not have an NVIDIA GPU or if GPU acceleration is not needed, please remove or comment out the related GPU configuration lines in the docker-compose.yml file to avoid runtime issues.

## Software Components Inside the Container Image

- **Base Image**: `ubuntu:22.04`

- **ROS 2**: Humble Hawksbill (also supports Foxy/Jazzy if rebuilt)

- **OrbbecSDK_ROS2**: ROS 2 wrapper for Orbbec camera SDK

- **amr_multi_cam**: Sample functional package for multi-camera use case

- **RViz2**: Pre-installed for visualization

- **colcon**, **argcomplete**, **image_transport_plugins**, and others

## Quick Start Guide

1. Clone the repository to your local machine

```shell
mkdir ~/orbbec_ros2_ws/
cd ~/orbbec_ros2_ws
git clone https://github.com/Advantech-EdgeSync-Containers/Orbbec-Gemini-3DCamera.git
```

2. Run the build script

```shell
cd ~/orbbec_ros2_ws/Orbbec-Gemini-3DCamera/orbbec-camera-demo
chmod +x build.sh
./build.sh
```

3. Build the ROS 2 workspace 

```shell
# Optional: The workspace is already prebuilt inside the container.
# Run this only if you've made changes to the source code.
colcon build
```

4. Launch the camera node

```shell
source ./install/setup.bash
ros2 launch amr_multi_cam single_camera_bringup.launch.py
```

You should see **RViz2** launch automatically. In addition, you should see the published **image streams**, **point clouds**, and **TF data**.

> **Note1:**  
> The launch file is configured by default for the **Gemini 330** series.  If you're using a different Orbbec camera model (e.g., Femto), please **update the launch file accordingly** to match your device configuration.

> **Note2:**  
> If RViz2 fails to launch, it may be due to an incorrect `DISPLAY` environment setting.  Run `who` to check the active display number (e.g., `(:1)`), then set it using `export DISPLAY=:1` and try again.  

## Demonstration Images

**RGB**, **Depth**, **Stereo IR**, and **Depth to Color Align** Image Display

![](./orbbec-camera-demo/images/ImageStreams.gif)

**Point Cloud Voxelization**

Voxel filtering reduces point cloud density, improving processing speed and system performance.

![](./orbbec-camera-demo/images/VoxelPointCloud.gif)

**DepthPointCloud**

Converts depth information into 3D point cloud data.

![](./orbbec-camera-demo/images/DepthPointCloud.gif)

**Colored Point Cloud**

Generates colored point clouds by aligning RGB images with depth maps.

![](./orbbec-camera-demo/images/ColorPointCloud.gif)

**Local Costmap**

The local costmap reflects real-time perception of the robot's immediate surroundings.

![](./orbbec-camera-demo/images/Costmap.gif)

**Camera Intrinsics Acquisition**

The `camera_info` topic outputs intrinsic parameters for each sensor.

![](./orbbec-camera-demo/images/camera_info_list.png)

**Camera Extrinsics Acquisition**

Camera extrinsic parameters are published on the `tf/tf_static` topics, which can be viewed using TF tools or visualized in RViz2.

![](./orbbec-camera-demo/images/TF.png)
