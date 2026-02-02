# RealSense Camera Verification & Point Cloud Pipeline (ROS 2 Humble)

This repository contains a standalone ROS 2 package (`camera_checking`) designed to verify the functionality of Intel RealSense cameras, Python image processing, and 3D point cloud generation without relying on complex AI models.

It serves as a hardware and middleware diagnostic tool to ensure your OS, USB drivers, and ROS transformations are working correctly before deploying complex robotics applications.

## 📌 System Architecture

This system isolates the data pipeline used in advanced manipulation tasks (like robotic pick-and-place) by removing external dependencies (MoveIt, Gemini AI, YOLO).

### The Data Flow:
1. **Driver**: `realsense2_camera` captures raw Depth and Color data.
2. **Mock Logic**: `mock_filter.py` receives the data, draws a white square in the center of the depth image (simulating a segmentation mask), and republishes it with Best Effort QoS and synchronized timestamps.
3. **Processing**: `depth_image_proc` (C++) takes the masked depth image and converts it into 3D points (`/points`).
4. **Visualization**: RViz2 displays the resulting Point Cloud.

## ⚙️ Prerequisites

- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS Distro**: ROS 2 Humble
- **Hardware**: Intel RealSense D400 Series Camera (Must be connected via USB 3.0)

## 🛠️ Installation

### Clone with Submodules

```bash
# Clone repository with realsense-ros submodule
mkdir -p ~/camera_test_ws/src && cd ~/camera_test_ws/src
git clone --recurse-submodules https://github.com/ASIFXS/camera_test.git

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-humble-depth-image-proc \
    ros-humble-cv-bridge

# Build the workspace
cd ~/camera_test_ws
colcon build
source install/setup.bash
```

**Note**: The `realsense-ros` submodule (commit: bafc21080c5c8e259dadbb309797949aee0dd950, v4.56.4) will be automatically downloaded and built from source.

## 🚀 Usage

```bash
# Launch the complete system
cd ~/camera_test_ws
source install/setup.bash
ros2 launch camera_checking check_system.launch.py
```

**What should happen:**
- The RealSense camera driver will start
- The mock_filter_node will start processing images
- RViz2 will open automatically

## 👁️ How to View the Point Cloud in RViz (CRITICAL)

By default, ROS 2 RealSense drivers publish data using **Best Effort** reliability (like UDP) for speed. RViz defaults to **Reliable** (like TCP). If these settings do not match, you will see nothing.

### Follow these steps to visualize the data:

1. **Global Options**: Ensure "Fixed Frame" is set to `camera_link`.
2. **Add Display**: Click the Add button in the bottom left.
3. **Select Topic**: Go to the **By Topic** tab and select `/points` → PointCloud2.
4. **Fix QoS Settings (The Important Part)**:
   - Expand the PointCloud2 item in the left panel.
   - Find the **Topic** section and expand **QoS**.
   - Change **Reliability** from `Reliable` to `Best Effort`.
   - Change **Durability** from `Transient Local` to `Volatile`.

### Expected Result

You should see a square patch of 3D points floating in black space. This square confirms that:
- ✅ The Camera is working.
- ✅ The Python script is correctly filtering the image.
- ✅ The timestamps are synchronized.
- ✅ The conversion to 3D space is successful.

## 🔧 Troubleshooting

### 1. "No RealSense devices were found!"

- **Check USB**: Ensure the camera is plugged into a USB 3.0 port (Blue/Red port).
- **Virtual Machine**: If using a VM (VirtualBox/VMware), ensure the USB Controller is set to USB 3.0/3.1 in the VM settings, and the device is passed through.

### 2. "Permission denied" / "Cannot open /dev/video"

Your user does not have permission to access the hardware. Run:

```bash
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
```

You must **restart your computer** (or log out and back in) for this to take effect.

### 3. "AttributeError: _ARRAY_API not found"

This happens if you have numpy version 2.0+ installed, which is incompatible with ROS Humble's cv_bridge.

**Fix**: Downgrade NumPy:

```bash
pip3 install "numpy<2.0"
```

### 4. "Incompatible QoS" Warnings

If you see warnings in the terminal about QoS policies, ensure you have followed the RViz configuration steps above. The launch file and Python script are hard-coded to use **Best Effort** to maximize compatibility with the hardware.

## 📁 Package Structure

```
camera_test_ws/
└── src/
    ├── camera_checking/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── check_system.launch.py
    │   └── scripts/
    │       └── mock_filter.py
    └── realsense-ros/              # Git submodule
        ├── realsense2_camera/
        ├── realsense2_camera_msgs/
        └── realsense2_description/
```

## 📝 License

Apache 2.0
