# IMU Documentation

Publishes an IMU message with quaternion orientation, angular velocity, and linear acceleration to the IMU_DEST_TOPIC_QUAT topic.

Also publishes a MagneticField message with magnetometer measurements to the IMU_DEST_TOPIC_MAG topic.

Locates the serial name of the IMU and reads its input as a string: 

```
$VNQMR,-0.017057,-0.000767,+0.056534,+0.998255,+1.0670,-0.2568,+3.0696,-
00.019,+00.320,-09.802,-0.002801,-0.001186,-0.001582*65
```

which is parsed into its individual components to be published as parts of the IMU and MagneticField messages.
