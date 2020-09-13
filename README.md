# chess-bot

This is a repo for a chess-playing robot that I am building to brush up on Python, C/C++, OpenCV, and ROS, and to make a fun/educational YouTube video out of. The desired functionality of the robot is as follows:

- A GUI on a laptop will allow a human player to interact with the robot, both by allowing the player to indicate when human moves have been made, and by making funny comments based on the board state
- When a human move is indicated, a camera plugged into the laptop will take an image of the chessboard and detect all the squares where pieces are. It will compare this new board state to the board state prior to the human move, and determine if a legal move is made. If an illegal move is made, a robot arm will knock over all the chess pieces.
- If a legal move is made, the chess robot core will use a Python interface with Stockfish (an open-source chess engine) to determine what move to make in response.
- Once a computer move is decided, a robot arm with 4 degrees of freedom (base, shoulder, elbow, and gripper) will move the relevant chess piece to the correct square. The robot will then start waiting again for a human move.

So far, the following functionality has been achieved:
- A 3-DoF robot arm has been designed, built, and tested. Currently, the joints are a bit shaky as they are direct connections to the driving servomotors instead of geared connections, so I will likely change this in an upcoming iteration.
- A rudimentary gripper has been designed, but has not yet been tested.
- Inverse kinematics C code has been written for the arm's Arduino Nano to convert Cartesian points to joint angles on the robot arm. The Arduino Nano is also able to receive commands corresponding to Cartesian points, and move its end effector in a straight line between these points by computing intermediate positions to move to. 
- A class for interacting with Stockfish with Python has been written.
