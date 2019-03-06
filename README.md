# Toy Car Simulation

![](images/screenshot.png)

Workspace for simulation a toy Ackermann vehicle

## Setup

### 1. Clone this repo

Note that this repo it a workspace. Clone it into your home directory.

```
$ cd
$ git clone <this repo>
```

### 2. `ros_control`

Install `ros-control` packages:

```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control
```

### 3. Build

```
$ cd ~/toy_car_sim
$ catkin_make
```

### 4. Floor Model

Go into the models directory and run the following script:

```
$ cd ~/toy_car_sim/models
$ sh copy_model.sh
```

## Running 

```
roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch
```

### Topics

  - **Camera:** `/car/camera1/image_raw`
  - Twist command: `/prizm/twist_controller/twist_cmd`



# Lane Following

## Dynamic Reconfigure

Here are some hints to transfer existing lane following code. 

Add the following lines to `cfg/LaneFollow.cfg` below `mask_dialate`:

```python
gen.add('speed',     double_t,   0, 'speed',      0,  0, 4.0)
gen.add('turn',     double_t,   0, 'turn',      0,  0, 4.0)
```

Use the config_ params just before publishing the message in `src/lane_follow.cpp`:

```c++
geometry_msgs::Twist twist;
twist.linear.x = vel.first * config_.speed;
twist.angular.z = vel.second * config_.turn;

pub_.publish(twist);
```

## Topics

## `lane_follow.launch`

```xml
<!-- launch the node -->
<node name="lane_follow" pkg="lane_follow" type="lane_follow" respawn="true" respawn_delay="10" output="screen">
    <!-- Use a specific camera path -->
    <param name="camera_topic" type="string" value="/car/camera1/image_raw" /> 
</node>
```

