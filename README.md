# Perception Using Laser Range Finder

This contains basic implementaion of RANSAC algorithm to determine the walls "visible" to the robot from the data obtained from the laser range finder.
The algorithm consists of following steps
  
1. From the set of points the laser range finder gives, pick two at random and draw a line. 
2. Find out the distance of each of the other points from this line, and bin them as inliers and outliers based on if the distance is lesser or greater than a threshold distance. Repeat this for k iterations. After k iterations, pick the line that has the most number of inliers.
3. Drop those points, and repeat the algorithm to the remaining set of points until you have lower than a threshold number of points.

Detected lines are publish on rviz for visualisation


# Navigation -Bug2 algorithm
This contains navigation of robot from start to goal by avoiding collisions using Bug2 algorithm. 
