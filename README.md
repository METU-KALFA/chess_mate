# chess_mate
Chess playing robot arm project.

## Dependencies

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


### Motion Controller


## Future Improvements


## Some errors we run into and how to fix them
