# Multi-Lidar Calibrator Helper Tool
Multi lidar calibrator helper tool allows for finding transforms between 2 given lidars. For the original documentation of multi lidar calibrator refer [here](README_calibrator.md).

There are 5 lidars on the bus and combining all of them in order to sense the surrounding of the bus is crucial. Thus, all lidar transforms must be accurately calculated with the help of this tool. The tool works for a pair of sensors at a time. So the general convention is to take the parent lidar as reference to other 4 children. This means 4 separate calibrations must be performed: 
* parent-front_right, 
* parent-front_left, 
* parent-back_right, 
* parent-back_left.


## Instructions

* Make sure all sensors are up by running `roslaunch adastec_sensing.launch`.
* Open up a terminal (Ctrl+Alt+T) and run ```lidar_calibrator```. This will open up RViz and run lidar calibration helper tool.

* By default, the calibration is set to be done between parent and front_right lidars. To change the child sensor, give its frame id as an argument: ```lidar_calibrator front_left```

<img src="images/multi_lidar_calibrator.png" alt="drawing" width="1000"/>


* Try to locate the bus where the two lidars that are currently being calibrated have enough space to scan. This ensures there is decent amount of features to be extracted by the algorithm and thus a more accurate transformation can be calculated.

* Observe the surroundings and try to find good reference points where the points from both sensors lay on top of eachother. Corners of structures, straight lines, poles are some of the references to look for. Other than that, pan the camera on RViz to reach ground level and observe both sensors register points on the ground at the same level.

* Play around with X, Y, Z values on rqt_reconfigure to crop the area so that places where only one sensor can see are eliminated. This makes focusing easier on the area where points from both sensors are present. Each time a crop dimension is changed, the algorithm will try to recalculate transformations. If any of the sensors appear to be in an unstable state (i.e. stuttering along one axis or dramatically tilted etc.), re-adjust crop area and give a 2D Pose Estimate to fit it to place.

* Once the points from both sensors look to be right on top of eachother and nicely levelled, kill the node with Ctrl+C to see the final transformation matrix. Copy the highlighted line as shown below and paste it inside bus's respective launch file i.e. "flowride.launcher/Main.Config/Calibrations/Extrinsics/MICHIGAN/MICHIGAN_Sensors.launch".

<img src="images/multi_lidar_output.png" alt="drawing" width="1000"/>


* Note that the terminal output won't include which child lidar frame was used in calibration as it only includes name="parent_to_front_" Don't forget to change it to the actual frame that was just calibrated i.e. *parent_to_front_left*.

* Once all lidars have been calibrated, verify the calibrations are done correctly by observing the output of all lidars concatenated. To do so run: `roslaunch tf.launch` (located flowride.launcher/Main.Config/Calibrations/Extrinsics/), then `roslaunch lidar_preprocessing.launch` to publish concatenated lidar points. Open RViz and add the topic of "concatenated".