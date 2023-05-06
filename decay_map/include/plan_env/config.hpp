#ifndef _DECAY_MAP_CONFIG_
#define _DECAY_MAP_CONFIG_
#include "vector"
#include "Eigen/Dense"
#include "string"
#include "ros/ros.h"
using namespace std;
class DecayMapConfig {
 public:
  vector<double> resolutions;
  vector<int> inflate_steps;

  /* aster properties */
  string frame_id;
  bool use_global_cloud{false}, print_update_time{false};
  string odom_topic, local_cloud_topic;
  /* probability update */
  /* visualization and computation time display */
  double visualization_truncate_height, virtual_ceil_height, ground_height;
  double local_update_distance;

  int hit_point_buffer_size;
  int hit_num_thresh;
  double decay_time;
  double odom_timeout;
  Eigen::Vector3d map_min_boundary, map_max_boundary;  // map range in pos
  Eigen::Vector3d map_origin;
  Eigen::Vector3d map_size;

  template<class T>
  bool LoadParam(string param_name, T &param_value, T default_value) {
	if (nh_.getParam(param_name, param_value)) {
	  printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
	  cout << param_value << endl;
	  return true;
	} else {
	  printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
	  param_value = default_value;
	  cout << param_value << endl;
	  return false;
	}
  }

  template<class T>
  bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
	if (nh_.getParam(param_name, param_value)) {
	  printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
	  for (int i = 0; i < param_value.size(); i++) {
		cout << param_value[i] << " ";
	  }
	  cout << endl;
	  return true;
	} else {
	  printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
	  param_value = default_value;
	  for (int i = 0; i < param_value.size(); i++) {
		cout << param_value[i] << " ";
	  }
	  cout << endl;
	  return false;
	}
  }

  ros::NodeHandle nh_;

  DecayMapConfig() {};

  DecayMapConfig(const ros::NodeHandle &nh_priv) {
	nh_ = nh_priv;
	// For decaymap
	vector<int> stts{1, 1};
	vector<double> ress{0.1, 0.2};
	LoadParam("decay_map/resolutions", resolutions, ress);
	LoadParam("decay_map/inflation_steps", inflate_steps, stts);

	double x_size, y_size, z_size;
	LoadParam("decay_map/map_size_x", x_size, -1.0);
	LoadParam("decay_map/map_size_y", y_size, -1.0);
	LoadParam("decay_map/map_size_z", z_size, -1.0);
	map_size = Eigen::Vector3d(x_size, y_size, z_size);

    // For virtual wall
	vector<double> zeroV;

	LoadParam("decay_map/local_update_distance", local_update_distance, -1.0);
	LoadParam("decay_map/odom_timeout", odom_timeout, 0.5);

	LoadParam("decay_map/print_update_time", print_update_time, false);
	LoadParam("decay_map/visualization_truncate_height", visualization_truncate_height, -0.1);
	LoadParam("decay_map/virtual_ceil_height", virtual_ceil_height, -0.1);

	LoadParam("decay_map/frame_id", frame_id, string("world"));
	LoadParam("decay_map/ground_height", ground_height, 1.0);
	LoadParam("decay_map/hit_point_buffer_size", hit_point_buffer_size, 10);
	LoadParam("decay_map/hit_num_thresh", hit_num_thresh, 4);
	LoadParam("decay_map/decay_time", decay_time, 3.0);

	LoadParam("decay_map/cloud", local_cloud_topic, string("cloud"));
	LoadParam("decay_map/odom", odom_topic, string("odom"));

  }

};
#endif