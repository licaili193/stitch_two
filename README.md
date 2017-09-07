# stitch_two

## Description

Stitch two videos side by side and inverse them.

## Prerequisites

ROS kinect
OpenCV 2.4.x

## Installation

Clone the repository into the src directory under the ROS workspace and compile the ROS workspace.

## Usage

* roslaunch stitch_two stitch.launch video_1:=VEDIO_PATH_1 video_2:=VEDIO_PATH_2

### Example
* roslaunch stitch_two stitch.launch video_1:=$HOME/field_trees.avi video
_2:=$HOME/dynamic_test.mp4

### Launch file arguments
* vedio_1 - path of the first video
* vedio_2 - path of the second video
* vedio_output - path of the output video - default: $HOME/output.avi
* fourcc - the fourcc code used for video writing - default: DIVX

## Program behavior

### Video load
The program will read the two videos into memory first. For each video, a class is designed with two member objects of thread. The first thread will read the frames into a queue buffer; the second thread will process each frame and invert it pixel by pixel. Conditional variables are used to protect from race conditions.

### Video resize
The output video will have the same height as the smaller one of the two videos. Frames of the other video will be resized accordingly to obtain the same height as the output video.

### Synchronizeing
The output video will have the same frame rate as the smaller one of the two videos. Frames of the other video will be synchronized according to the fps ratio.

### Video length
The output video will have the same length as the shorter one of the two videos. The other video will be cut off.

## Future improvement

* If the application is designed to not having any editing or previewing features, it is not necessary to load the whole videos. Video rendering may happens the same time as video loading in other threads.
