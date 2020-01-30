# Robotics Localization, Planning and Navigation

This is a robotics group project about implementing a robotics system under the LEJOS EV3 platform written in Java. The main function can be found under the RobotController.java. All classes are implemented in the same file due to the single file upload limitation of the LEJOS platform. The sequence of tasks it can perform is as follows:

1.Localization to find its current location on the map<br/> ![Farmers Market Finder Demo](bin/gif.gif) <br/>
<br/>
2.Conduct A* search to plan a path from localized point to the goal position while avoiding obstacles on the map<br/>
<br/>
3.Path Navigation according to planned path <br/> ![Farmers Market Finder Demo](bin/robot_navigation.gif) <br/>
<br/>
4.Enter into the U-shaped box <br/> ![Farmers Market Finder Demo](bin/enter.gif) <br/>
<br/>
5.Move until it touches the wall and make a beep sound<br/>
<br/>
6.Move back out from the U-shaped box to the previous goal point <br/>
<br/>
7.Conduct A* search again to plan a path from that goal point back to original starting position while avoiding obstacles on the map<br/>
<br/>
8.Path Navigation according to planned path <br/> ![Farmers Market Finder Demo](bin/leave.gif) <br/>
<br/>

<a href="https://www.youtube.com/watch?v=RvMfIgPz6fQ&feature=youtu.be" title="Link Title"><img src="{image-url}" alt="Here is the link to the video of the entire demo of the robot" /></a>

