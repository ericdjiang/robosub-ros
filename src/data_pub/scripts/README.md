# IMU Documentation

Publishes an `sensor_msgs/IMU` message to the `IMU_DEST_TOPIC_QUAT` topic with
- orientation (using quaternions)
- angular velocity
- linear acceleration 


Also publishes a `sensor_msgs/MagneticField` message with magnetometer measurements to the `IMU_DEST_TOPIC_MAG` topic.

Locates the serial name of the IMU and reads its input as a string: 

```
$VNQMR,-0.017057,-0.000767,+0.056534,+0.998255,+1.0670,-0.2568,+3.0696,-
00.019,+00.320,-09.802,-0.002801,-0.001186,-0.001582*65
```

This is parsed into its individual components to be published as parts of the `IMU` and `MagneticField` messages.

## Running the Code

Start the dockerfile, mounting the code to the container:

```
docker run -td --privileged --net=host --mount type=bind,source=/home/robot/robosub-ros/src,target=/home/duke/dev/robosub-ros/catkin_ws/src  dukerobotics/robosub-ros
```

Once the container is running, SSH into the `duke` machine:

  * On the local machine
```
ssh -XY -p 2200 duke@localhost
```

  * On the remote machine
```
ssh -XY -p 2200 duke@192.168.1.1
```

Password should be obtained from another member.

## Testing/Editing the Code
Start with starting roscore:
```
roscore&
```
where the `&` is used to give user control while the `roscore` runs in the background.  If `roscore` is already running, it will give you an error saying this.  This is fine as long as `roscore` is running.

Then navigate to the directory:
```
~/dev/robosub-ros/catkin_ws
```

Here, run this command to build the workspace:
```
catkin build
```
This will create some files (if the docker container was not already running), one of which will be the directory `devel/`.

Then you will source the files needed to run scripts:

```
source /opt/ros/kinetic/setup.bash
```
where `kinetic` should be the version of `ros` that you are using (in this case, it should be `kinetic`)

```
source devel/setup.bash
```
