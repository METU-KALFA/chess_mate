# chess_mate
Chess playing robot arm project.

## Dependencies
- libfranka
- franka_ros
- stockfish
- python-chess
- OpenCV
- realsense-ros
- MoveIt
- rosserial
- roscontrol-pkg
- ultralytics

## Weights for piece and board detection

## How to run everything separately

## How to run everything using a launch file

## Pipeline

### Game Controller
- Keeps track of the game state
- Requests needed board information from board_sensor
- Picks the best move for the robot using the Stockfish engine
- Handles special chess rules such as castling, piece promotion, and en passant.
- Detects the player move after the button is pressed. If move is not valid due to piece detection errors, program will prompt you to 		  enter a move from the terminal
- Sends the pick and place positions to move_controller


### Computer Vision
*fill this part and add images*

### Motion Controller
*fill this part*

## Future Improvements
- Improve piece detection
- Pixel to world coordinates implemented but not working correctly. For now we keep the board fixed.
- After fixing step 2, will need to test piece promotion to make sure the model can detect pieces outside of the board and their coordinates and make the correct set of moves to promote a piece
- Test if robot can make an en passant move, and if it can recognize if a player makes an en passant move
- Test if the robot can make a castling move

## Some errors we run into and how to fix them
