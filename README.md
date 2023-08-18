# chess_mate
<div id="header" align="center">
  <img src="https://i.imgur.com/yw1zi1H.jpeg" width="180"/>
  
  <img src="https://www.metu.edu.tr/system/files/logo_orj/9/9.4.png" width="350"/>
 </div>
A chess-playing robot arm adept at analyzing and engaging with diverse board positions using object detection and traditional computer vision methods.

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

## Installation
`add`
`stuff`
`here`

## Project Outline
![UML](uml.png)


### Game Controller
- Keeps track of the game state
- Requests needed board information from board_sensor
- Picks the best move for the robot using the Stockfish engine
- Handles special chess rules such as castling, piece promotion, and en passant.
- Detects the player move after the button is pressed. If move is not valid due to piece detection errors, program will prompt you to 		  enter a move from the terminal
- Sends the pick and place positions to move_controller


### Computer Vision
#### First Step: Chessboard Detection
- The image is resized as 640x640 to be prepared for object detection.
- Chessboard is detected by machine learning model prepared using YOLOv8 object detection algorithm.
- The image is cropped by the taken data from the object detection model.
 <div id="header" align="center">
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/original%20image.jpg" height="200"/>
  
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/resize.jpg" height="200"/>
  
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/cropped%20image.jpg" height="200"/>
 </div>
 
#### Second Step: Preparing Image
- Adaptive threshold is applied to the image.
- All contours are found and the biggest contour is selected.
- Outer corner points of the image is found by the selected contour.
 <div id="header" align="center">
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/threshold.jpg" height="200"/>
   
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/outer%20corners.jpg" height="200"/>
  
 </div>
 
#### Third Step: Perspective Transform
- According to found corner points, perspective transform matrix M is found by cv2.perpectiveTransform() and perspective transform is applied to the image by cv2.warpPerspective().
- In the bird-view image, all four corner points of every square are found and transformed a list.
 <div id="header" align="center">
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/bird%20view.jpg" height="200"/>

  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/copy.jpg" height="200"/>
 </div>
 
#### Fourth Step: Inverse Perspective Transform
- By using the inverse matrix of the perspective transform matrix, four corner points of every square are transformed back to the original perspective.
- By mathematical operations, center points of every square are found.
<div id="header" align="center">
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/four%20points.jpg" height="200"/>
 </div>
 
#### Fifth Step: Back to the Original Image
- Thanks to knowing the top left corner pixel coordinate of the boundary box of detected chessboard, the cropped image with center points is tranformed back to the resized version of the original image.
- By scaling the resized image, the original image with center points is obtained.
<div id="header" align="center">
  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/resized%20middle%20points.jpg" height="200"/>

  <img src="https://github.com/METU-KALFA/chess_mate/blob/main/images/original%20middle%20points.jpg" height="200"/>

 </div>

#### Weights for piece and board detection
Board detection: https://drive.google.com/drive/folders/1Q814KcSBCfySLgavn4tyzaaHBehxSmb8?usp=drive_link
Pieces detection: https://drive.google.com/drive/folders/1nWCjnJWY0MIILTfV6DDCxnoSCoZ2IwSv?usp=drive_link


### Motion Controller
*fill this part*
*maybe add gazebo gif*


## Future Improvements
- Improve piece detection
- Pixel to world coordinates implemented but not working correctly. For now we keep the board fixed.
- After fixing step 2, will need to test piece promotion to make sure the model can detect pieces outside of the board and their coordinates and make the correct set of moves to promote a piece
- Test if robot can make an en passant move, and if it can recognize if a player makes an en passant move
- Test if the robot can make a castling move

## Author(s)

