# CarND - Capstone Project

Team Members: Alone (fahadzubair@gmail.com)

## NO TRAFFIC LIGHT DETECTION (AS YET)

I got late in my homework, haven't been able to teamup with someone and hence no traffic light detection (**AS YET**)

## Waypoint Calculation

*Overall Strategy*:

When current pose of the car is received, the alogrithm tries to find where the car is located by calculating the distance from the pose to all waypoints in the world map. Once the point has been found, next time the algorithm only looks at up to 100 points to the last known location to see, which point is closest.

After finding the closest point, the code publishes 200 points ahead of that point on **/final_waypoints**

Ros Node: **waypoint_updater** (only implemented partially, does not look for traffic signals)

|Subscribes To|Message|Reason|
|--|--|--|
|/base_waypoints|Lane|All waypoints are saved to figure out waypoints ahead of car|
|/current_pose|PoseStamped|To figure out where the car is located using distance to the closest point|


|Publishes|Message|Reason|
|--|--|--|
|/final_waypoints|Lane|100 Waypoints that are in front of the car|

**Shortcomings**

1) Does not cater to for traffic / obstacle waypoints
2) Does not modify velocity between points based on traffic figured
3) There is no gaurantee that after the first time car is located it will always be within 100 points of the last detection. Maybe a particle filter should be implemented here

## Driving Code

*Overall Strategy*:

Figures out the current linear and angular velocity of the car by subscribing to /current_velocity topic. Error is calculated between the proposed velocities (linear/angular) being sent on the /twist_cmd topic and the current linear/angular velocity of the car. Two simple PID controllers are used for throttle and steering. Values returned by the PID controller are sent to /vehicle/throttle_cmd and /vehicle/steering_cmd topics. Since traffic lights have not been detected, brake is set to 0.

Ros Node: **dbw_node** 

|PID Values||
|--|--|
|Throttle_P|1.0|
|Throttle_I|0.0|
|Throttle_D|0.0|
|Steering_P|2.0|
|Steering_I|0.0|
|Steering_D|0.0|

**Shortcomings**

1) Would have liked to implement MPC instead of a simple PID controller
2) Max / Min Steering has been implemented using -1 to 1 range. Where as ~max_steer_angle is set to 8. So need to fix that
3) Integral and Derivatives should be properly tuned (or calculated using twiddle)
4) Braking should be calculated. Plan on using a range of -1 to 1 for throttle and then use negative values to brake.


## Traffic Light Detection

***TODO:** Yet to implement


# Original Repo Notes

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
