# object_detection

This folder give several ways to detect object with their colour. 
I used these algorithms for a Pick&Place for PR2.

Launch the Kinect : 
roslaunch openni_launch openni.launch publish_tf:=false camera:="head_mount_kinect" depth_frame_id:="head_mount_kinect_ir_optical_frame"

Launch the color Algorithm :
rosnode object_detection ColorTrackingEssaiKinect

See topics send :
rostopic echo /pyride_status

I used OPENCV and PointCloud.

![alt text](OPENCV.png " Image with OpenCV Detection")
![alt text](OPENCV.png " Image with OpenCV Detection")

