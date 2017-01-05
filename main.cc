#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>

namespace fs = ::boost::filesystem;

// return the filenames of all files that have the specified extension
// in the specified directory and all subdirectories
std::vector<std::string> get_all(const std::string& path, const std::string& ext)
{
    std::vector<std::string> filenames;
    const fs::path root(path);
    if(fs::exists(root) && fs::is_directory(root)) {

       fs::directory_iterator it(root);
       fs::directory_iterator endit;

       while(it != endit) {
          if(fs::is_regular_file(*it) && it->path().extension() == ext) filenames.push_back(it->path().string());
          ++it;
       }
    }
    return filenames;
}

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_from_images");

    if(argc!=5)
    {
        cerr << "Usage: rosrun rosbag_from_images rosbag_from_images <path to image directory> <image extension .ext> <frequency> <path to output bag>" << endl;
        return 0;
    }

    ros::start();

    // Vector of paths to image
    vector<string> filenames = get_all(std::string(argv[1]), std::string(argv[2]));

    cout << "Images: " << filenames.size() << endl;

    // Frequency
    double freq = atof(argv[3]);

    // Output bag
    rosbag::Bag bag_out(argv[4],rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T=1.0f/freq;
    ros::Duration d(T);

    for(size_t i=0;i<filenames.size();i++)
    {
        if(!ros::ok())
            break;

        cv::Mat im = cv::imread(filenames[i],CV_LOAD_IMAGE_COLOR);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::RGB8;
        cvImage.header.stamp = t;
        bag_out.write("/camera/image_raw",ros::Time(t),cvImage.toImageMsg());
        t+=d;
        cout << i << " / " << filenames.size() << endl;
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
