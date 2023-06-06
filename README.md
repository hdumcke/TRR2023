# Toulouse Robot Race

## Introduction

In June 2023 I participated for the first time in a robot race, the [TRR](http://www.toulouse-robot-race.org)

## The Event

The Toulouse Robot Race is a race for small autonomous  robots to  follow a [track](http://www.toulouse-robot-race.org/wp-content/uploads/2023/02/piste.png) in the shortest time possible. There are different categories for different type of robots: robots with wheels, robots with two legs and robots with more than two legs. You'll find a picture of the track [here](media/IMG-0875.jpg)

The event was well organized, a big thank you to the organizers, the partners and the sponsors.

Participation was open to interested parties regardless of age or affiliation and it was a pleasure seeing a wide variety of participants of all age and all different robots, starting with a little boy of 4 years and his dad to a group of 10 - 15 years old using Scratch to program their robots up to 70+ building their own robots.

## The Robot

I decided to [participate](media/IMG-0873.jpg) with [Mini Pupper v2](https://www.kickstarter.com/projects/336477435/mini-pupper-2-open-source-ros2-robot-kit-for-dreamers), an open source project I have been involved with for over a year now. With a maximum speed of 20 cm per second it is certainly not a very fast robot but it is a full featured 12 DoF quadruped well equipped with camera, Lidar, IMU and other sensors. It has what is required to navigate the track.

My special thanks goes to [MangDang](https://www.mangdang.net) who provided me with the latest pre-production version for this race.

## The Theory

With a robot capable to move forward and backwards, left and right and turn in any direction all that was required to implement an algorithm to keep the robot on the track and stop after crossing the red finishing line. Piece of cake, what can possibly go wrong?

## The Implementation

I choose to use the gait controller from Stanford that I [enhanced](https://github.com/hdumcke/StanfordQuadruped/tree/mini_pupper_ros2) to work with the ROS2 middle ware. I started to write a ROS node with OpenCV to detect the line with the intention to use this information to generate cmd_vel messages to keep the robot over the middle of the line. I became concerned about reflection of sun light that could distort my line follower and decided to use the Lidar to implement a wall follower.

With the border walls of the track being 20 cm hight and the quadruped shaking while running which results in moving the base plane of the Lidar in an angle to the ground I had to ensure that the Lidar is mounted as low as possible. Once mini pupper v2 will ship it will come with a Lidar mount that mounts the Lidar higher and moved further back.

To implement the wall follower I segmented the laser scan into front, left, behind and right segments and for each segment I used Hough transform to discover lines in the point cloud and then picked the longest line to be my wall. This is certainly more complicated as required by TRR but when testing at home my walls in the corridor also have doors.

I also implemented a node with OpenCV to discover red objects, assuming that the only red I will see is the finishing line.

I also needed a start button and the rules of the race required that there is an emergency stop button. Mini Pupper v2 provides four programmable touch buttons that would have been ideal for my use case but unfortunately the development board that I had did not have this functionality fully operational. I decided to disable the speaker for which I had no use, this freed a GPIO port that I used to connect to a physical button. No other usable GPIO port was left, so the button has to be used to start and stop the robot.

I did all the development of these nodes on a PC and used CyclonDDS to communicate with mini pupper. Only two days before I had to leave for the race I moved these nodes to the robot itself, the rules are that the robot must be autonomous.

Mini pupper is equipped with an ESP32 micro processor and a Raspberry CM4 as main processor. The current software uses the ESP32 to drive the servos, collect data from the IMU and the battery status but the gait controller itself runs on the Raspberry. Adding all the additional nodes to detect the finish line, follow the wall and monitoring the start/stop button added load to the Raspberry Pi to a point that the gait controller slowed down so much that it became unusable.

With little time left I decided to give up on OpenCV and use a timer hoping that it will time out after the robot has crossed the finishing line. I also give up on the (useless) idea to detect any wall front or left to only focus on one wall on the side. I used two PID controller, one to regulate liner.y to keep the robot a constant distance from the wall and the other to regulate angular.z to keep the robot parallel to the wall. The test environment I had was a wall in a corridor of 2 meter length but interrupted by doors. But the fine tuning of the PID controllers looked reasonable well to me.

Last day before I had to leave for the race. Autonomous probably means no network connection either. Let's test with WiFi disabled. Panic, the robot does not move and it even does not show 'No IP address' on the LCD screen, meaning that the ROS node responsible for controlling the gait has not started. Luckily I was still able to connect the robot to a monitor,  I had no plan to take a monitor to the event. It turned out that CyclonDDS needs an IP address to joint a multicast address. The workaround was to disable CyclonDDS and then the ROS services came up and operated as expected.


## The Reality

We had a day prior to the race to test using the race track. First observation when looking at the track was that the walls in a curve are not straight lines, who would have thought? I obviously have not thought everything though, here goes my dream of making the full circuit. But at least the straight line should be no problem, unfortunately it turned out that the robot turned and I was not able to configure the PID parameters to compensate. My wall follower turned into a wall hugger. At 20 cm per sec the robot should take 30 sec to complete the 6 meter straight line. Drifting to the side, touching the wall many times it took the robot 40 sec to cross the [finishing line](media/IMG-79101.3gp).

My time was 10m times the time it took for the winner of the race and that was a 8DoF quadruped build out of Lego bricks. Kudos to the winner.

## Conclusion

It was a fun event and it was great meeting like minded people but overall I was very disappointed by my own performance. Now it is time to analyze what went wrong, find a better strategy to navigate the robot and try again. But most importantly I have to grow some legs :-)
