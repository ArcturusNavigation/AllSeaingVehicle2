# AllSeaingVehicle2

## Getting Started

### Prerequisites

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
    - If using the Jetson, use [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
- ZED SDK 4.0 (only install on Jetson devices)
    - When flashing the Jetson Xavier, make sure to flash with Jetpack 5.1.1! As of now (08/29/2023), the ZED SDK does not support the latest Jetpack 5.1.2.

To download the ZED SDK, find and download the correct installer from https://www.stereolabs.com/developers/release/ or use curl

```
curl -o installer.run https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.0/ZED_SDK_Tegra_L4T35.3_v4.0.6.zstd.run
chmod +x ./installer.run
```

Next, run the installer

```
sudo apt install zstd # Run this if on a linux/jetson OS
./installer.run
```

On a Jetson device, you also need to create an alias to the path for opencv

```
sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv 
```

### Cloning the repository

```
git clone https://github.com/ArcturusNavigation/AllSeaingVehicle2.git
```

Update the submodules:

```
git submodule update --init --recursive
```

## Building repository

First, we need to install all the rospackage dependencies. Navigate to the top-level of the repository then run

```
rosdep install --from-paths src --ignore-src -r -y 
```

Next, we'll build the main packages of the repository and source the build

```
colcon build --symlink-install
source install/setup.bash
```

## Testing
