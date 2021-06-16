MLND Capstone Project Description - Robot Motion Planning

In this project, a robot mouse is tasked with plotting a path from a corner of the maze to its center in two runs.

In the first run, the robot mouse tries to map out the maze to not only find the center, but also figure out the best paths to the center. In subsequent runs, the robot mouse attempts to reach the center in the fastest time possible, using what it has previously learned. 
In the first run, it must enter the goal room at some point during its exploration, but is free to continue exploring the maze after finding the goal. After entering the goal room, the robot may choose to end its exploration at any time. The robot is then moved back to the starting position and orientation for its second run. Its objective now is to go from the start position to the goal room in the fastest time possible. The robot’s score for the maze is equal to the number of time steps required to execute the second run, plus one thirtieth the number of time steps required to execute the first run. A maximum of one thousand time steps is allowed to complete both runs for a single maze.


Files Explanation:

|- robot.py - This script establishes the robot class. This is the only script that I can modify and work on.
|- maze.py - This script contains functions for constructing the maze and for checking for walls upon robot movement or sensing.
|- tester.py - This script will be run to test the robot’s ability to navigate mazes.
|- showmaze.py - This script can be used to create a visual demonstration of what a maze looks like.
|- test_maze_##.txt - These files provide sample mazes upon which to test your robot.


