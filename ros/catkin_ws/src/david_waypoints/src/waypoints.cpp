#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "image_transport/subscriber.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <aruco.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;

class WaypointCalculator {

private:
  ros::NodeHandle _nh;
  // TODO: Waypoint position publishers.

  aruco::MarkerDetector _detector;
  aruco::CameraParameters _cam_params;
  aruco::MarkerMap _mmap;
  aruco::MarkerMapPoseTracker _mmtrack;
  // image_transport::Subscriber sub;
  float _marker_size;

  tf2_ros::TransformBroadcaster _br;

public:
  WaypointCalculator(char *topic_1, char *topic_2, char *calibration_file, char *mmap_name,
                     float size) {
    cout << "Init Waypoint Calculator " << topic_1 << endl;
    cout << "Init Waypoint Calculator " << topic_2 << endl;

    image_transport::ImageTransport it(_nh);
    new image_transport::Subscriber(it.subscribe(
        topic_1, 1, &WaypointCalculator::camera_image_callback, this));
    new image_transport::Subscriber(it.subscribe(
        topic_2, 1, &WaypointCalculator::camera_image_callback, this));
    _detector.setDictionary("ARUCO_MIP_36h12");
    _cam_params.readFromXMLFile(calibration_file);
    _mmap.readFromFile(mmap_name);
    _mmtrack.setParams(_cam_params, _mmap);
    _marker_size = size;
    cout << "Testicles" << endl;
  }

  void camera_image_callback(const sensor_msgs::ImageConstPtr &ros_image) {

    cv::Mat img = cv_bridge::toCvShare(ros_image, "bgr8")->image;
    auto markers = _detector.detect(img, _cam_params, _marker_size);

    std::vector<double> Rvec;
    std::vector<double> Tvec;

    // Estimate the pose
    _mmtrack.estimatePose(markers);

    if (_mmtrack.isValid()) {
      Rvec = _mmtrack.getRvec();
      Tvec = _mmtrack.getTvec();

      // Only print marker detected if we actually have a transform
      if (Rvec.size() != 0) {
        ROS_INFO("Marker Detected");
        cout << _mmtrack.getRvec() << endl;
        cout << _mmtrack.getTvec() << endl;


        // Setup transform
        tf2::Quaternion q_rot;
        q_rot.setRPY(Rvec[0], Rvec[1], Rvec[2]);
        geometry_msgs::Quaternion quat_msg = tf2::toMsg(q_rot);

        geometry_msgs::TransformStamped mytf;
        mytf.header.stamp = ros::Time::now();
        mytf.header.frame_id = "map";
        mytf.child_frame_id = "marker_map";
        // Set Translations
        mytf.transform.translation.x = Tvec[0];
        mytf.transform.translation.y = Tvec[1];
        mytf.transform.translation.z = Tvec[2];
        // Set Rotation
        mytf.transform.rotation.x = q_rot.x();
        mytf.transform.rotation.y = q_rot.y();
        mytf.transform.rotation.z = q_rot.z();
        mytf.transform.rotation.w = q_rot.w();

        _br.sendTransform(mytf);
      } else {
        ROS_INFO("No Marker Found");
      }
    }
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "waypoints");

  if (argc != 6) {
    cout << "Usage waypoints <image_topic_1> <image_topic_2> <camera_params.yml> "
            "<marker_map.yml> <marker_size>"
         << endl;
    abort();
  }

  WaypointCalculator calc(argv[1], argv[2], argv[3], argv[4], stof(argv[4]));
  ros::spin();

  ros::shutdown();
}
