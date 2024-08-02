# CUBE EYE camera ROS2 driver (S100D / S110D / S111D / I200D device)
This driver package is for the following devices on ROS2
- #### S100D / S110D / S111D / I200D

Please refer to our website for detailed product specifications. [http://www.cube-eye.co.kr](http://www.cube-eye.co.kr)



## Installation Instructions

The following instructions are written for ROS2 Humble, on Ubuntu 22.04 - Previously tested on ROS2 Eloquent, Ubuntu 18.04.

### Step 1 : Install the ROS2 distribution
- #### Install ROS2 Humble distribution. https://docs.ros.org/en/rolling/Releases.html

### Step 2 : Install the driver
- #### Create a workspace
```bash
$ source /opt/ros/humble/setup.bash
$ mkdir –p ~/dev_ws/src
$ cd ~/dev_ws/src
```
And copy the driver source to the path(dev_ws/src)

- #### Build
```bash
$ cd ~/dev_ws
$ colcon build --symlink-install
```

## Usage Instructions

Connect the camera power and execute the following commands

```bash
$ source install/setup.bash
```

```bash
$ ros2 run cubeeye_camera cubeeye_camera_node
```

Or

```bash
$ ros2 launch cubeeye_camera camera_launch.py
```
#### Services
- #### Scan CUBE EYE camera
```bash
$ ros2 service call /cubeeye_camera_node/scan cubeeye_camera/srv/Scan
```

- #### Connect camera source (camera index)
Connect the first camera (index: 0) in scanned sources.
```bash
$ ros2 service call /cubeeye_camera_node/connect cubeeye_camera/srv/Connect "{index: 0}"
```

- #### Run camera (frame type)
Run Amplitude and Depth (type: 6):
```bash
$ ros2 service call /cubeeye_camera_node/run cubeeye_camera/srv/Run "{type: 6}"
```
Run PointCloud (type: 32):
```bash
$ ros2 service call /cubeeye_camera_node/run cubeeye_camera/srv/Run "{type: 32}"
```

- #### Stop camera
```bash
$ ros2 service call /cubeeye_camera_node/stop cubeeye_camera/srv/Stop
```

- #### Disconnect camera
```bash
$ ros2 service call /cubeeye_camera_node/disconnect cubeeye_camera/srv/Disconnect
```

#### Topics
Topics are published when a camera is connected. Once a camera starts with a frame type, the frame is populated.
- /cubeeye/camera/depth : Depth Image 
- /cubeeye/camera/amplitude : Amplitude Image
- /cubeeye/camera/rgb : RGB Image
- /cubeeye/camera/points : PointCloud Image

#### Operating Test
```bash
$ rqt
```
/cubeeye/camera/amplitude, /cubeeye/camera/depth
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545764-ca58ce00-55f8-11eb-86a9-3bc091a1aeb9.png"/></p>

```bash
$ ros2 run rviz2 rviz2
```
Fixed Frame : pcl  
PointCloud2 : /cubeeye/camera/points

<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545815-de043480-55f8-11eb-8293-baa2edba664f.png"/></p>

#### Using Dynamic Reconfigure Params
```bash
$ ros2 run rqt_reconfigure rqt_reconfigure
```
![dynamic_reconfigure](https://user-images.githubusercontent.com/90016619/131959624-b3ff03e2-2eb7-4cc6-bc5d-df39dc74ed89.png)
