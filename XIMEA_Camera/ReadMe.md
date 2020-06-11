### This is instruction for camera calibration  
#### 1. Prepare for camera calibration. Go to the folder "/home/nvidia/Desktop/stereovision/camera-calibration/calibrationCode/"; source code are all under this folder.  
##### 1.1 Compile the right camera reading code: open "CMakeLists.txt" in this folder, and add two more line as following:  
add_executable(read_images read_images.cpp)  
target_link_libraries(read_images ${OpenCV_LIBS} "-lpopt")  
Then go the folder "bin" from the command window and type in "cmake ..; make" to recompile the source code  
#### 1.2 Change the image exposure time: open "read_images.cpp" in this folder (calibrationCode), and you will find the exposue_time definition as in line12. The exposure time can be change by change the values like "exposure_time(88000)" -> "exposure_time(60000)" are changing exposure time from 88000us to 60000us.  
Then go the folder "bin" from the command window and type in "cmake ..; make" to recompile the source code  
#### 2. capture the image: go the "bin" folder from the command window and type in  
./read_images -w 1440 -h 1080 -d "../calib_imgs/" -e bmp  
This command will start the camera for capturing image. As shown in the following image: 
<p align="center"> <img width = "1000" src = "https://github.com/wutongyu98/CTR_ObstacleAvoidance/blob/master/camera_calibration_Instruction/capture%20Image.png" ></img>
</p>  

**Remarking1:** make sure you arrange the three windows, including left camera window, right camera window and command window can be seen at the same time. You need to check the camera windows to adjust the calibration target position and make sure the target is in both cameras' view all the time. 

**Remarking2:** There is around 5 seconds before every image capture, and the program will count down to let you know how much time is remaining. You need to change the target postion, distance away from the camera and orientation of the target board relative to the camera during this time count down. Then freeze your motion when the count down is approaching 0, so the camera can capture the image non-blurred image; change the target position when the new count down start. The image capture program will collect 50 pairs of images.  

**Remarking3:** Tips for good target postion and image quality: (1) target should fully show in two cameras' view; (2) target should fill at least 50% window of the camera's view window. (3) turn around the target at different orientation (4) exposure time should be appropirate to clearly visualize the calibration target, but not over exposure.  

#### 3. calibration left and right camera.  

##### 3.1 the camera should save all the captured images in the folder "calib_imgs". We need to make a new folder named "1" under the folder "calib_imgs" and move all the collected images to folder "1". 

##### 3.2 calibrate the left camera:  
Go to folder "bin" from the command window, and type in  
./calibrate -w 24 -h 17 -n 50 -s 0.015 -d "../calib_imgs/1/" -i "left" -o "cam_left.yml" -e "bmp"  
I assume you are using targe "18x25 | Checker Size: 15mm", and the width and height are both substracted by 1, so you have 24 and 17; the number 50 here is the image number; this program will give you the following results as you see in the following image  
<p align="center"> <img width = "1000" src = "https://github.com/wutongyu98/CTR_ObstacleAvoidance/blob/master/camera_calibration_Instruction/left_camea_Calibration.png" ></img>
</p>   

**Remarking1:** Check the errors for each images, if you find some error is abnormally large, for example, error for the 17th image is 3.22312, while all others are around 0.5. You need to remove this image mannually; so for this case, you need to remove left18.bmp and right18.bmp at the same time; be careful not left17.bmp and right17.bmp. since the counting for calibration is starting from 0 while image capture are starting from 1;  

**Remarking2:** Since the program only read image sequency like left1.jpg, left2.jpg, left3.jpg ....; if you are have a gap in the sequence like left1.jpg, left3.jpg  left4.jpg ....; or left2.jpg, left3.jpg  left4.jpg ....; you will get an error.  

**Remarking3:** Therefore, after you remove left18.bmp and right18.bmp, you need to fill this missing name from the end of the image list, for this case, you need to rename left50.jpg to left18.jpg and right50.jpg to right18.jpg. If you need to remove multiple images having large error, you need always fill in the missing gaps in the name sequence. Then change the -n 50 in the command line to -n 49, if you only remove 1 pair of images (I have updated the code on my side for the new cameras to automatically fill the image name gap, but the code on your xavier was not updated).  

**Remarking4:** rerun the command after you remove and rename the images.  
./calibrate -w 24 -h 17 -n 49 -s 0.015 -d "../calib_imgs/1/" -i "left" -o "cam_left.yml" -e "bmp"   

##### 3.3 calibrate the right camera:  
./calibrate -w 24 -h 17 -n 49 -s 0.015 -d "../calib_imgs/1/" -i "right" -o "cam_right.yml" -e "bmp"  
then repeat to remove large error images, and rename the image name to make sure the sequence is not gapping.  then re-run the calibration command for both cameras.  
##### 3.3 calibrate the stereo camera:  
./calibrate_stereo -n 49 -u cam_left.yml -v cam_right.yml -L ../calib_imgs/1/ -R ../calib_imgs/1/ -l left -r right -o cam_stereo.yml  
Remarking: you get the final calibration file "cam_stereo.yml"  
#### 4 verify the calibration result  
##### 4.1 start the two camera and capture a pair of images and name them as "left1.bmp" and "right1.bmp";  

##### 4.2 copy this pair of images to the folder "/home/nvidia/Desktop/stereovision/camera-calibration/calibrationCode/calib_imgs/"; and copy the generated "camera_stereo.yml" to the folder "/home/nvidia/Desktop/ros_stereo/test/IMS_MThread_PCL_Filter_Xavier"  

##### 4.3 open a new command window and type in "cd "/home/nvidia/Desktop/ros_stereo/test/IMS_MThread_PCL_Filter_Xavier/bin" to let you into the stereo vision test code. then type in "./main" in the command window. You will see this  
<p align="center"> <img width = "1000" src = "https://github.com/wutongyu98/CTR_ObstacleAvoidance/blob/master/camera_calibration_Instruction/stereovision.png" ></img>
</p>  

If there is no 3D point cloud display, or very poor. Try to swap the image name of "left1.bmp" and "right1.bmp". If still very poor, that means the calibration result is not good, that might be caused by the calibration target are not well posed during the calibration process, and you have to recapture the image and redo the calibration. Until you get an good 3D point cloud map.  
