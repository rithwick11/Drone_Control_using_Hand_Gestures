# Drone_Control_using_Hand_Gestures

Hi everyone!

This repository contains the files hand_detect.py and dronefly.py the scripts used to run the detections and integrate with dronekit and sitl to control the drone.
Made certain modifications by which the drone can be controlled in the following 8 ways using the following hand gestures: 
1. Forward - Touch Thumb to Index Finger
2. Backward - Touch Thumb to Middle Finger
3. Right - Touch Thumb to Ring Finger
4. Left - Touch Thumb to Pinky Finger
5. Up - Touch Thumb to MetaCarpel of Index Finger
6. Down - Touch Thumb to MetaCarpel of Middle Finger
7. Stop - Touch Thumb to MetaCarpel of Ring Finger
8. RTL (Return To Launch) - Touch Thumb to MetaCarpel of Pinky Finger

Instead of building a model to train the gestures, right now I have used the framework Mediapipe.
The scripts use the frameworks: DroneKit, SITL, Mediapipe.
