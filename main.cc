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
// in the specified directory (not the subdirectories!).
std::vector<fs::path> get_all(const fs::path& root, const std::string& ext)
{
    std::vector<fs::path> filenames;
    if(fs::exists(root) && fs::is_directory(root)) {
       for (fs::directory_iterator it(root), endit;  it != endit; ++it) {
          if(fs::is_regular_file(*it) && it->path().extension() == ext) {
            filenames.push_back(it->path());
          }
       }
    }
    return filenames;
}

std::string fixFileExtension(const std::string& ext) {
  if (*ext.begin() == '.') {
    return ext;
  } else {
    return '.' + ext;
  }
}

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_from_images");

    if(!(argc==5 || argc == 6))
    {
        cerr << "Usage: rosrun rosbag_from_images rosbag_from_images <path to image directory> <image extension .ext> <frequency> <path to output bag> [topic]" << endl;
        return 0;
    }

    const std::string topic = argc == 6 ? std::string(argv[5]) : "/camera/image_raw";

    ros::start();

    // Vector of paths to image
    const vector<fs::path> filenames = get_all(fs::path(std::string(argv[1])), fixFileExtension(std::string(argv[2])));

    cout << "Images: " << filenames.size() << endl;

    // Frequency
    const double freq = atof(argv[3]);
    const bool use_freq = freq > 0;

    // Output bag
    rosbag::Bag bag_out(argv[4],rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    for(size_t i = 0; i < filenames.size() && ros::ok(); i++)
    {
        const cv::Mat im = cv::imread(filenames[i].string(),CV_LOAD_IMAGE_COLOR);

        if (!use_freq) {
          const std::string date_str = filenames[i].stem().string();
          t = ros::Time::fromBoost(boost::posix_time::from_iso_string(date_str));
        }

        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::BGR8;
        cvImage.header.stamp = t;
        bag_out.write(topic, t, cvImage.toImageMsg());

        if (use_freq) {
          t += ros::Duration(1.0f/freq);
        }
        cout << i << " / " << filenames.size() << endl;
    }

    bag_out.close();

    ros::shutdown();
    return 0;
}
