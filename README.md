# object_tracking_particle_filter | (https://www.youtube.com/watch?v=cNlmO6aZ-Ug)
Install Picoflexx Firmaware
---------------------------
Go to : https://pmdtec.com/picofamily/software/
Install the software as per your system.
<You will need password for this, which is written on the box of camera>

In the Readme file for PMD Picoflexx Camera:
For Linux : Please extract the Linux package (will result in a “libroyale-3.15.0.X-LINUX-x86-64Bit” or
“libroyale-3.15.0.X-LINUX-x86-32Bit” folder). Then transfer the complete folder to your com-
puter.
Make sure that you have proper permissions to the USB device. The installation package
contains a proper rules file which can be used. It is located in the /driver/udev directory,
please refer to the README file in that directory for more details.

So we'll do as instructed,
Go to ../libroyale-3.15.0.50-LINUX-x86-64Bit/driver/udev and do as instructed in README:
From the ../libroyale-3.15.0.50-LINUX-x86-64Bit/driver/udev folder run the command: sudo cp 10-royale-ubuntu.rules /etc/udev/rules.d

Check if camera is working:
Go to ../libroyale-3.15.0.50-LINUX-x86-64Bit/bin and run the following command in terminal : ./royaleviewer.sh
Note: Please note that libroyale version 3.8 perfectly works for Ubuntu 14. I faced some problems with libroyale version 3.11
(which is the latest), hence I would reccommend to use libroyale version 3.8 if you are using Ubuntu 14.

As of now, the folder is on Desktop and works perfectltly!

# Install Picoflexx Working with ROS
------------------------------------
source : readme file insie /sampleROS prakcage of ROS for PMD Picoflexx camera.
Which you'll find after unpacking the zipped file for your system. 

Before, using Picoflexx with ROS we need ROS!

Install ROS
------------
1. Install and configure ROS (e.g. Kinetic). (http://wiki.ros.org/kinetic/Installation/Ubuntu)
   - upper right button in Ubuntu --> `System Settings...` --> `Software & Updates` --> make sure the options
     (`restricted`, `universe` and `multiverse`) are selected.
   - add source:
	`$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
   - set the key:
	`$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116`
   - make sure the system software is in the latest version:
	`$ sudo apt-get update`
   - install ROS:
	`$ sudo apt-get install ros-kinetic-desktop-full`
   - initialize rosdep:
	`$ sudo rosdep init`
	`$ rosdep update`
   - initialize environment
	`$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc`
	`$ source ~/.bashrc`
   - install the commonly used plugins, e.g.:
	`$ sudo apt-get install python-rosinstall`
2. Creating a workspace for catkin. (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
   - create a catkin workspace:
	`$ mkdir -p ~/catkin_ws/src`
   - build the workspace:
	`$ cd ~/catkin_ws/`
	`$ catkin_make`
   - add the directory into ROS_PACKAGE_PATH environment variable:
	`$ echo $ROS_PACKAGE_PATH`
	/home/youruser/catkin_ws/src:/opt/ros/kinetic/share

Congratulation! you have ROS working now.

Copy the ROS package for camera inside ROS worksapce
----------------------------------------------------
 Copy `sampleROS` from `<path_of_extracted_royale_sdk>/libroyale-<version_number>-LINUX-64Bit/samples/cpp/` to `~/catkin_ws/src/sampleROS`.
 
 
 3. Add the executable flag to the camera_driver.cfg file
    `$ cd ~/catkin_ws/src/sampleROS/cfg`
    `$ chmod +x camera_driver.cfg`
4. Build the example
	`$ cd ~/catkin_ws/`
	`$ catkin_make`
5. Connect camera device
6. Source the setup.bash file
	`$ source devel/setup.bash`
7. Run ROS
	`$ roslaunch royale_in_ros camera_driver.launch`
8. Open a new Terminal to start rviz (a visualization for ROS)
	`$ rosrun rviz rviz`
   (in rviz)
   To add point_cloud topic:
	a. Set the `Fixed frame` to `royale_camera_link`;
	b. Add `PointCloud2` from `By display type`, set `Topic` to `royale_camera_driver/point_cloud` and
       set `Channel Name` to `z`;
	   Or add `PointCloud2` from `By topic` and set `Channel Name` to `z`.
   To add image topics:
	a. Add `Image` from `By display type` and set `Image Topic`;
	   Or add topics from `By topic`;
	b. The both image topics support for sub-topics `Camera` and `DepthCloud` too. Before adding these
       two sub-topics, the `Fixed frame` has to been set earlier.
9. Open a new Terminal to start Dynamic Reconfigure
	`$ rosrun rqt_reconfigure rqt_reconfigure`
   (in Dynamic Reconfigure)
	Choose `royale_camera`

Now you have Picoflexx working with ROS!

The next step is to use bring PCL into ROS and make it work.

Get PCL working in ROS
----------------------
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

Check PCL
---------
Create a folder 'test_pcl'
Create file and copy the code from : http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough
Create build folder inside that folder :
mkdir build & cd build
Compile the source code
cmake .. & make

PCL is working on your computer now!

Getting it working with ros
---------------------------
Dependencies :
sudo apt-get install ros-indigo-moveit-planners-ompl 
sudo apt-get install ros-indigo-ompl

PCL-ROS
-------
sudo apt-get install ros-indigo-pcl-ros

Good Place to learn PCL : https://github.com/hsean/Capstone-44-Object-Segmentation/wiki

Finally we will run the tracker 
------------------------------

Go to catkin_ws folder:
cd ~/catkin_ws

Plugin the camers 
( Re-plug it if it does not work!                                                              )
( Its better to use USB 3 and not USB 2                                                        )     
( Sometimes, if there is a screen attached, it might create an issue, so try it without screen )

Source the worksapce :
source devel/setup.bash

Run the camera:
roslaunch royale_in_ros camera_driver.launch

Run the Rviz Rviz:
rosrun rviz rviz
Add the Point Cloud 2

Add a PointCloud2 display type and Topic as /royale_camera_driver/point_cloud
(When the Rviz window opens, see lower left side, there will be Add tab, from the list select PointCloud2
Then in the topic tab select the one which is described)

Change the frame to royale_camera_optical_frame
(When the RViz windows open, see right upper corner, where there will be window with frame, change the wondow to
one which is mentioned above)

(See Part 3 for more.)
Runt the tracker, in a new terminal
rosrun tracker tracking_object <location of the .pcd file>
rosrun tracker tracking_object data/teddy.pcd 

Few things about above command:
-------------------------------
Add another point cloud tab in the same rviz and select the topic as 'result_cloud'
The last argment is the saved point cloud which should somewhere saved on your computer.

You should see everything working!
