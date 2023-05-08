# Hi! Welcome to our examples!!

## Here is the Control Award from the 2022-2023 Season! We got our first coordinate-based autonomous that season!
https://docs.google.com/document/d/12C6fO5XzDp6SqYQiDkT0dG8IgRCLnMp-NpVTef2KQ-o/edit?usp=sharing

## Motion Profile
Our robot uses an algorithm that is able to take in a max velocity and acceleration to calculate 
where the robot should be between its starting position and its end position (a coordinate on the field) over time. 
This calculates ‘steps’ for the robot to follow, and the error between the robot’s position and this target. 
The error is then passed to the PID Controller to calculate motor output. Our motion profile has multiple states 
for the turn to the target, drive in a straight line to that target (which is necessary as the junctions on the 
field require more precise movement), and stop. The robot is able to correct its course due to its localization.

## Localization
This year one of our goals was to cycle (score more than one cone) in autonomous. We use two dead wheel
(small wheels disconnected to motors or servos) attached to encoders on our tank drive. As the robot moves, 
the wheels spin and the encoders record the information, thus, we are able to track the robot’s position 
on the field using the Road Runner library’s Localization class

## Roadrunner Control Explained - CTRL ALT FTC
https://www.ctrlaltftc.com/practical-examples/roadrunner-control-explained
This is what we based our motion profile and PID controller on. It has lots of good explanations and is
a lot simpler and easier to understand than roadrunner

## Learn Road Runner
https://learnroadrunner.com/
This is where we got our localization class from. It has a very good introductory explanation for roadrunner
and how localization works/can be used for. Very good place to start!

## Tank Drive + Mecanum drive classes
Instead of putting all the robot functions in different programs, we can create a robot object that 'owns'
all of the robot functions and motors. I have copies of these programs to use/look at, and the TankDrive one
(what we used this season) is annotated!

## Features | FTC Dashboard
https://acmerobotics.github.io/ftc-dashboard/features.html
The dashboard is a super cool resource to make quick changes instead of having to rebuild the entire project
(which takes around 2-5 minutes depending on how long since last build). The link to your dashbaord is the
link given in 'Progamming' on the driver station '/dash'. Make sure you are connected to the robot wifi!
To make programs and variables show up in the dashboard, add '@Config' next ro '@Autonomous' or '@Teleop'.
Variables will show up if they are public and static.

## Easy Wireless Control Hub Connection in Android Studio : r/FTC
https://www.reddit.com/r/FTC/comments/jt8hei/easy_wireless_control_hub_connection_in_android/
1) Close Rev Hardware Client (RHC)
2) Connect pc to robot wifi
3) Open RHC
4) If RHC sees the control hub → Android studio should build!
5) If not → disconnect and try again
6) If not again → power cycle (turn robot on and off)
7) If not again → restart everything, reconnect with wire