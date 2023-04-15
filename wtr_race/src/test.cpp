#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  // Initialize the ROS node
  ros::init(argc, argv, "marker_array_example");
  ros::NodeHandle n;

  // Create a publisher for the marker array
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  // Create a MarkerArray message
  visualization_msgs::MarkerArray marker_array;

  // Create a Marker message for a cube
  visualization_msgs::Marker marker_cube[2];
  

  // Add the cube marker to the array
    for(int i=0;i<2;i++)
    {
        marker_cube[i].header.frame_id = "world";
        marker_cube[i].header.stamp = ros::Time::now();
        marker_cube[i].ns = "marker_array_example";
        marker_cube[i].id = i;
        marker_cube[i].type = visualization_msgs::Marker::CUBE;
        marker_cube[i].action = visualization_msgs::Marker::ADD;

        marker_cube[i].pose.orientation.x = 0.0;
        marker_cube[i].pose.orientation.y = 0.0;
        marker_cube[i].pose.orientation.z = 0.0;
        marker_cube[i].pose.orientation.w = 1.0;
        marker_cube[i].scale.x = 1.0;
        marker_cube[i].scale.y = 1.0;
        marker_cube[i].scale.z = 1.0;
        marker_cube[i].color.r = 0.0;
        marker_cube[i].color.g = 1.0;
        marker_cube[i].color.b = 0.0;
        marker_cube[i].color.a = 1.0;
        marker_cube[i].pose.position.x = i;
        marker_cube[i].pose.position.y = i;
        marker_cube[i].pose.position.z = 0.0;
        marker_cube[i].lifetime=ros::Duration(0.1);
        marker_array.markers.push_back(marker_cube[i]);
    }
  

  while (ros::ok())
  {
    marker_pub.publish(marker_array);
  }

  return 0;
}
// #include <ros/ros.h>
// #include <visualization_msgs/Marker.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "marker_publisher");
//   ros::NodeHandle n;
//   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

//   // Set the frame ID and timestamp
//   std::string frame_id = "world";
//   ros::Time timestamp = ros::Time::now();

//   // Set the marker type and action
//   uint32_t shape = visualization_msgs::Marker::CUBE;
//   visualization_msgs::Marker marker;
//   marker.type = shape;
//   marker.action = visualization_msgs::Marker::ADD;

//   // Set the pose of the marker
//   marker.pose.position.x = 0;
//   marker.pose.position.y = 0;
//   marker.pose.position.z = 0;
//   marker.pose.orientation.x = 0.0;
//   marker.pose.orientation.y = 0.0;
//   marker.pose.orientation.z = 0.0;
//   marker.pose.orientation.w = 1.0;

//   // Set the scale of the marker
//   marker.scale.x = 1.0;
//   marker.scale.y = 1.0;
//   marker.scale.z = 1.0;

//   // Set the color of the marker
//   marker.color.r = 0.0;
//   marker.color.g = 1.0;
//   marker.color.b = 0.0;
//   marker.color.a = 1.0;

//   // Set the remaining fields of the marker
//   marker.header.frame_id = frame_id;
//   marker.header.stamp = timestamp;
//   marker.lifetime = ros::Duration();

//   // Publish the marker
//   while (ros::ok())
//   {
//     marker_pub.publish(marker);
//     ros::spinOnce();
//   }

//   return 0;
// }

