# RoboticsGroupProject

This is a robot implemented under the LEJOS EV3 platform written in Java. The main function can be found under the RobotController.java. All classes are implemented in the same file due to the single file upload limitation of the LEJOS platform. The sequence of tasks it can perform is as follows:

1.Localization
2.A* search to plan a path from localized point to the goal position
2.Path Navigation according to planned path
3.Enter into the U-shaped box
4.Move until it touches the wall and make a beep sound
5.Move back out from the U-shaped box to the previous goal point
6.A* search again to plan a path from that goal point back to original starting position
7.Path Navigation according to planned path

<a href="https://www.youtube.com/watch?v=RvMfIgPz6fQ&feature=youtu.be" title="Link Title"><img src="{image-url}" alt="Here is a demo of the robot" /></a>

[![Demo CountPages alpha](https://gifs.com/?source=https://www.youtube.com/watch?v=RvMfIgPz6fQ&feature=youtu.be](https://www.youtube.com/watch?v=RvMfIgPz6fQ&feature=youtu.be)
