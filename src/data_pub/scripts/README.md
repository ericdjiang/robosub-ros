# IMU Documentation

Publishes an `sensor_msgs/IMU` message to the `IMU_DEST_TOPIC_QUAT` topic that contains information about
- orientation (using quaternions)
- angular velocity
- linear acceleration 


and publishes a `sensor_msgs/MagneticField` message with magnetometer measurements to the `IMU_DEST_TOPIC_MAG` topic.

Locates the serial name of the IMU and reads its input as a string: 

```
$VNQMR,-0.017057,-0.000767,+0.056534,+0.998255,+1.0670,-0.2568,+3.0696,
-00.019,+00.320,-09.802,-0.002801,-0.001186,-0.001582*65
```

This is parsed into its individual components to be published as parts of the `IMU` and `MagneticField` messages.

## Physical Devices

In the transition from the old computer (`NUC`) to the new computer (`Jetson`), certain physical devices need to be checked before running code.

First you want to make sure that both the IMU and (ADAPTER) are plugged into the robot *before* booting it up.

Then you need to have the `pyserial` python library installed on the computer.
```
pip install pyserial
```

Make sure that the correct version of the library is installed.  The robot should be compatible with `python2` and `python3`, but make sure with others which version should be installed.

If you are installing for `python2.7`:
```
python2.7 -m pip install <PACKAGE NAME>
```

If you are installing for `python3`:
```
python3 -m pip install <PACKAGE NAME>
```

When `pyserial` is installed as well as the devices both plugged in correctly, the following command should list 2 external USB devices:
```
python -m serial.tools.list_ports -v
```

After this, running code should work if not otherwise.

## Starting the Container

Start the dockerfile, mounting the code to the container.  The code may be still in the command history:

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

At this directory, run this command to build the workspace:
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

The IMU code is in the directory:
```
~/dev/robosub-ros/catkin_ws/src/data_pub/scripts/
```

To run the code, the following command format should work in any directory once everything above is set up:
```
rosrun <PACKAGE NAME> <PROGRAM NAME>
```

In this case we want to run the `data_pub` package's file `IMU.py`:
```
rosrun data_pub IMU.py
```

You should see the following message format printed in the terminal very quickly:
```
$VNQMR,±0.XXXXXX,±0.XXXXXX,±0.XXXXXX,±0.XXXXXX,±X.XXXX,±X.XXXX,±X.XXXX,±XX.XXX,±XX.XXX,±XX.XXX,±0.XXXXXX,±0.XXXXXX,±0.XXXXXX*YY
```

To listen to this topic, run the following code in a *separate terminal* while the first terminal is still running the `IMU.py` code:
```
rostopic echo IMU_DEST_TOPIC_QUAT
```
