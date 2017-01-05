# rosbag_from_images

ROS package to generate a rosbag from a collection of images. Images are ordered alphabetically. The timestamp for each image is assigned according to the specified frequency or is obtained from the filename. 

Tested in ROS Kinetic.

## Usage:

    rosrun rosbag_from_images rosbag_from_images PATH_TO_IMAGES IMAGE_EXTENSION FREQUENCY PATH_TO_OUPUT_BAG [TOPIC]
  
 - `PATH_TO_IMAGES`: Path to the folder with the images
 - `IMAGE_EXTENSION`: .jpg, .png, etc.
 - `FREQUENCY`: Frames per second. If FREQUENCY <= 0, the timestamp is obtained from the filename (must only contain an iso formatted time stamp!).
 - `PATH_TO_OUTPUT_BAG`: Path to save the bag (including the filename e.g. directory/filename.bag)
 -  TOPIC : the topic to publish the images on (optional, default: /camera/image_raw).

