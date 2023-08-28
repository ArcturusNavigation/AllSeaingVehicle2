# AllSeaingVehicle2

## Getting Started

### Prerequisites

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Docker and [Distrobox](https://github.com/89luca89/distrobox) (if Ubuntu 22.04 is not available, e.g. on Jetson devices)
- ZED SDK (only install on Jetson devices)

If Ubuntu 22.04 is needed, install Distrobox and run 

```
distrobox create -i ubuntu:22.04
```

to create an Ubuntu 22.04 environment. You can now enter the Ubuntu 22.04 environment by running `distrobox enter <name-of-env>` and exit by running `exit`.

The Velodyne LiDAR drivers also require the following packages:

```
sudo apt install libpcap-dev ros-humble-diagnostic-updater
```

Finally, to download the ZED SDK, find and download the correct installer from https://www.stereolabs.com/developers/release/ or use curl

```
curl -L https://download.stereolabs.com/zedsdk/3.8/l4t32.6/jetsons -o installer.run # Replace link with https://download.stereolabs.com/zedsdk/3.8/cu117/ubuntu20 if using docker
chmod +x installer.run 
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
