#ifndef _VOX_MAP_H
#define _VOX_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <iostream>
#include <random>
#include "queue"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "utils/scope_timer.hpp"
#include "raycast.hpp"
#include "livox_lidar/utils/common_type_name.hpp"
#include "livox_lidar/utils/visualization_utils.hpp"

#include "mutex"

#include "config.hpp"

#define logit(x) (log((x) / (1 - (x))))

using namespace std;
using namespace fmt;
using namespace Eigen;

/* *
 *  For FOV
 *
 * */


struct DecayMapNode {
    float prob{0.0}; // current hit probability
    // 0: free; 1: occupied; -1: permanent occupied
    int is_occupied{0};
    vector<double> point_time; // N point input time
    double update_time;
    int time_update_id{0};
    uint point_hit_num{0};
    int round{0};
};

struct DecayMapInfNode {
    double update_time;
    // 0: free; 1: occupied; -1: permanent occupied
    int is_occupied{0};
    int round{0};
};

class VoxMap {
public:
    VoxMap() {}

    ~VoxMap() {
    }

    double resolution, resolution_inv;
    int inflation_step;
    double decay_time;
    /* map properties */
    Eigen::Vector3i map_min_boundary_id, map_max_boundary_id;
    Eigen::Vector3i map_voxel_num;                        // map range in index
    int max_address;
    std::vector<DecayMapNode> occupancy_buffer;
    std::vector<DecayMapInfNode> occupancy_buffer_inflate;

    Eigen::Vector3d map_min_boundary, map_max_boundary;  // map range in pos
    Eigen::Vector3d map_origin;
    Eigen::Vector3d map_size;

    void initMap(DecayMapConfig &cfg_, double _resolution, int inf_step, bool only_inflate = false) {
        resolution = _resolution;
        resolution_inv = 1.0 / resolution;
        inflation_step = inf_step;
        map_origin = cfg_.map_origin;
        map_size = cfg_.map_size;
        decay_time = cfg_.decay_time;
        map_max_boundary = cfg_.map_max_boundary;
        map_min_boundary = cfg_.map_min_boundary;

        for (int i = 0; i < 3; ++i)
            map_voxel_num(i) = ceil(cfg_.map_size(i) / resolution);
        max_address = map_voxel_num(0) * map_voxel_num(1) * map_voxel_num(2);
        posToIndex(map_min_boundary, map_min_boundary_id);
        posToIndex(map_max_boundary, map_max_boundary_id);

        int buffer_size = map_voxel_num(0) * map_voxel_num(1) * map_voxel_num(2);
        occupancy_buffer_inflate.resize(buffer_size);

        // Add vi
       int ceil_id = (cfg_.virtual_ceil_height - map_origin.z()) * resolution_inv;
       for (int x = map_min_boundary_id.x(); x < map_max_boundary_id.x(); x++)
           for (int y = map_min_boundary_id.y(); y < map_max_boundary_id.y(); y++) {
               occupancy_buffer_inflate[toAddress(x, y, ceil_id)].is_occupied = -1;
           }

        if (!only_inflate) {
            // init occupancy buffer
            occupancy_buffer.resize(buffer_size);
            for (int i = 0; i < occupancy_buffer.size(); i++) {
                occupancy_buffer[i].point_time.resize(cfg_.hit_point_buffer_size, 0);
            }
        }

    }

    void updateInflation(const Eigen::Vector3d pos, const double eff_time) {
        // 更新 26 邻居
        Eigen::Vector3i id;
        posToIndex(pos, id);
        Eigen::Vector3i shift;
        for (int x = -inflation_step; x <= inflation_step; ++x)
            for (int y = -inflation_step; y <= inflation_step; ++y)
                for (int z = -inflation_step; z <= inflation_step; ++z) {
                    shift = Eigen::Vector3i(x, y, z) + id;
                    int address = toAddress(shift);
                    if (address >= max_address || address < 0) {
                        continue;
                    }
                    if (occupancy_buffer_inflate[address].is_occupied == -1) {
                        continue;
                    }
                    if (decay_time == 0) {
                        occupancy_buffer_inflate[address].is_occupied = -1;
                        continue;
                    }
                    occupancy_buffer_inflate[address].update_time = eff_time;
                    occupancy_buffer_inflate[address].is_occupied = 1;
                }
    }

    void boundIndex(Eigen::Vector3i &id) {
        Eigen::Vector3i id1;
        id1(0) = max(min(id(0), map_voxel_num(0) - 1), 0);
        id1(1) = max(min(id(1), map_voxel_num(1) - 1), 0);
        id1(2) = max(min(id(2), map_voxel_num(2) - 1), 0);
        id = id1;
    }

    bool isOccupied(const Eigen::Vector3d &pos) {
        return isOccupied(toAddress(pos));
    }

    bool isOccupied(Eigen::Vector3i &id) {
        return isOccupied(toAddress(id));
    }

    bool isOccupied(int address) {
        if (address >= max_address || address < 0) {
            // the ponint out of map is considered to be occupied.
            return true;
        }
        if (occupancy_buffer[address].is_occupied == 0) {
            return false;
        }
        if (occupancy_buffer[address].is_occupied == -1) {
            return true;
        }
        if (decay_time > 0) {
            double cur_t = ros::Time::now().toSec();
            if (occupancy_buffer[address].update_time + decay_time > cur_t) {
                return true;
            } else {
                occupancy_buffer[address].is_occupied = 0;
                return false;
            }
        } else // decay time = 0 and point is occupied.
        {
            return true;
        }
    }

    bool isOccupiedInflate(int address) {
        if (address >= max_address || address < 0) {
            // the ponint out of map is considered to be occupied.
            return true;
        }
        if (occupancy_buffer_inflate[address].is_occupied == 0) {
            return false;
        }
        if (occupancy_buffer_inflate[address].is_occupied == -1) {
            return true;
        }
        if (decay_time > 0) {
            double cur_t = ros::Time::now().toSec();
            if (occupancy_buffer_inflate[address].update_time + decay_time > cur_t) {
                return true;
            } else {
                occupancy_buffer_inflate[address].is_occupied = 0;
                return false;
            }
        } else // decay time = 0 and point is occupied.
        {
            return true;
        }
    }

    bool isOccupiedInflate(const Vector3d &pt) {
        int address = toAddress(pt);
        if (address >= max_address || address < 0) {
            // the ponint out of map is considered to be occupied.
            return true;
        }
        if (occupancy_buffer_inflate[address].is_occupied == 0) {
            return false;
        }
        if (occupancy_buffer_inflate[address].is_occupied == -1) {
            return true;
        }
        if (decay_time > 0) {
            double cur_t = ros::Time::now().toSec();
            if (occupancy_buffer_inflate[address].update_time + decay_time > cur_t) {
                return true;
            } else {
                occupancy_buffer_inflate[address].is_occupied = 0;
                return false;
            }
        } else // decay time = 0 and point is occupied.
        {
            return true;
        }
    }

    bool isOccupiedInflate(const Vector3i &pt) {
        int address = toAddress(pt);
        if (address >= max_address || address < 0) {
            // the ponint out of map is considered to be occupied.
            return true;
        }
        if (occupancy_buffer_inflate[address].is_occupied == 0) {
            return false;
        }
        if (occupancy_buffer_inflate[address].is_occupied == -1) {
            return true;
        }
        if (decay_time > 0) {
            double cur_t = ros::Time::now().toSec();
            if (occupancy_buffer_inflate[address].update_time + decay_time > cur_t) {
                return true;
            } else {
                occupancy_buffer_inflate[address].is_occupied = 0;
                return false;
            }
        } else // decay time = 0 and point is occupied.
        {
            return true;
        }
    }

    int toAddress(const Eigen::Vector3i &id) {
        return id(0) * map_voxel_num(1) * map_voxel_num(2) + id(1) * map_voxel_num(2) + id(2);
    }

    int toAddress(const Eigen::Vector3d &pos) {
        Vector3i id;
        posToIndex(pos, id);
        return id(0) * map_voxel_num(1) * map_voxel_num(2) + id(1) * map_voxel_num(2) + id(2);
    }

    int toAddress(const int x, const int y, const int z) {
        return x * map_voxel_num(1) * map_voxel_num(2) + y * map_voxel_num(2) + z;
    }

    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) {
        for (int i = 0; i < 3; ++i) {
            id(i) = floor((pos(i) - map_origin(i)) * resolution_inv + 0.5);
        }

    }

    void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos) {
        for (int i = 0; i < 3; ++i) {
            pos(i) = (id(i)) * resolution +
                     map_origin(i);
        }
    }

    bool insideMap(Vector3i &id) {
        if (id.x() > map_max_boundary_id.x() || id.x() < map_min_boundary_id.x()) { return false; }
        if (id.y() > map_max_boundary_id.y() || id.y() < map_min_boundary_id.y()) { return false; }
        if (id.z() > map_max_boundary_id.z() || id.z() < map_min_boundary_id.z()) { return false; }
        return true;

    }

    void boxSearch(Eigen::Vector3d &box_max,
                   Eigen::Vector3d &box_min,
                   vector<Eigen::Vector3d> &pc, const int skip_point = 0) {
//        TimeConsuming t("boxSearch");
        pc.clear();
        Eigen::Vector3d size = (box_max - box_min).cwiseAbs();
        int approximate_size = (size.x() * resolution_inv) *
                               (size.y() * resolution_inv) *
                               (size.z() * resolution_inv) * 0.2;
        pc.reserve(approximate_size);
        Eigen::Vector3i local_box_min, local_box_max;
        posToIndex(box_min, local_box_min);
        posToIndex(box_max, local_box_max);
        Eigen::Vector3i tmp_p;
        for (tmp_p.x() = local_box_min.x(); tmp_p.x() <= local_box_max.x(); tmp_p.x() += skip_point + 1) {
            for (tmp_p.y() = local_box_min.y(); tmp_p.y() <= local_box_max.y(); tmp_p.y() += skip_point + 1) {
                for (tmp_p.z() = local_box_min.z(); tmp_p.z() <= local_box_max.z(); tmp_p.z() += skip_point + 1) {
                    if (isOccupied(tmp_p)) {
                        Eigen::Vector3d pt_w;
                        indexToPos(tmp_p, pt_w);
                        pc.push_back(pt_w);
                    }
                }
            }
        }
    }

    void boxSearchInflate(Eigen::Vector3d &box_max,
                          Eigen::Vector3d &box_min,
                          vector<Eigen::Vector3d> &pc, const int skip_point = 0) {
//        TimeConsuming t("boxSearch");
        pc.clear();
        Eigen::Vector3d size = (box_max - box_min).cwiseAbs();
        int approximate_size = (size.x() * resolution_inv) *
                               (size.y() * resolution_inv) *
                               (size.z() * resolution_inv) * 0.2;
        pc.reserve(approximate_size);
        Eigen::Vector3i local_box_min, local_box_max;
        posToIndex(box_min, local_box_min);
        posToIndex(box_max, local_box_max);
        Eigen::Vector3i tmp_p;
        for (tmp_p.x() = local_box_min.x(); tmp_p.x() <= local_box_max.x(); tmp_p.x() += skip_point + 1) {
            for (tmp_p.y() = local_box_min.y(); tmp_p.y() <= local_box_max.y(); tmp_p.y() += skip_point + 1) {
                for (tmp_p.z() = local_box_min.z(); tmp_p.z() <= local_box_max.z(); tmp_p.z() += skip_point + 1) {
                    if (isOccupiedInflate(tmp_p)) {
                        Eigen::Vector3d pt_w;
                        indexToPos(tmp_p, pt_w);
                        pc.push_back(pt_w);
                    }
                }
            }
        }
    }
};

// intermediate mapping data for fusion
struct DecayMapData {
    // main map data, occupancy of each voxel and Euclidean distance

    pcl::PointCloud<pcl::PointXYZ> latest_pc_;
    ros::Time pc_rcv_time_;
    VoxMap obstacle_map, guide_map;

    // camera position and pose data

    QuadState odom_;

    // flags of map stateA
    bool occ_need_update_{false};
    bool new_cloud_rcv_{false};

    // range of updating grid
    Eigen::Vector3i local_bound_min_, local_bound_max_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class DecayMap {
private:
    DecayMapConfig cfg_;
    DecayMapData md_;
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_, odom_sub_, click_sub_;
    ros::Publisher map_pub_, map_inf_pub_, map_ori_pub_;
    ros::Timer occ_timer_, vis_timer_;

public:
    DecayMap() {};

    DecayMap(ros::NodeHandle nh, DecayMapConfig &cfg) {
        print(fg(color::lime_green), " -- [DecayMap] Init process.\n");
        // cfg_.map_size has been init in config.h
        cfg_ = cfg;
        nh_ = nh;

        if (cfg_.virtual_ceil_height - cfg_.ground_height > cfg_.map_size.z()) {
            cfg_.virtual_ceil_height = cfg_.ground_height + cfg_.map_size.z();
        }
        cfg_.map_origin = Eigen::Vector3d(-cfg_.map_size.x() / 2.0, -cfg_.map_size.y() / 2.0, cfg_.ground_height);

        cfg_.map_min_boundary = cfg_.map_origin;
        cfg_.map_max_boundary = cfg_.map_origin + cfg_.map_size;

        print(fg(color::lime_green),
              "\tmap_size = ({}, {}, {})\n",
              cfg_.map_size.x(),
              cfg_.map_size.y(),
              cfg_.map_size.z());
        print(fg(color::lime_green),
              "\tmap_origin = ({}, {}, {})\n",
              cfg_.map_origin.x(),
              cfg_.map_origin.y(),
              cfg_.map_origin.z());
        print(fg(color::lime_green),
              "\tmap_min_boundary = ({}, {}, {})\n",
              cfg_.map_min_boundary.x(),
              cfg_.map_min_boundary.y(),
              cfg_.map_min_boundary.z());
        print(fg(color::lime_green),
              "\tmap_max_boundary = ({}, {}, {})\n",
              cfg_.map_max_boundary.x(),
              cfg_.map_max_boundary.y(),
              cfg_.map_max_boundary.z());


        md_.obstacle_map.initMap(cfg_, cfg_.resolutions[0], cfg_.inflate_steps[0]);
        md_.guide_map.initMap(cfg_, cfg_.resolutions[1], cfg_.inflate_steps[1], true);

        print(fg(color::lime_green), " -- [DecayMap] Init Done.\n");
        // Update mapping date structure size
        odom_sub_ = nh_.subscribe(cfg_.odom_topic, 100, &DecayMap::odomCallback, this);
        cloud_sub_ = nh_.subscribe(cfg_.local_cloud_topic, 1000, &DecayMap::pointCloudCallback, this);
        vis_timer_ = nh_.createTimer(ros::Duration(0.11), &DecayMap::visCallback, this);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("decay_map/occupancy", 10);
        map_inf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("decay_map/occupancy_inflate", 10);
        map_ori_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("decay_map/occupancy_origin", 10);


    }

    ~DecayMap() {
    }

    Vector3i getPoolSize() {
        return md_.obstacle_map.map_voxel_num;
    }

    double getResolution(int type = 0) {
        return cfg_.resolutions[type];
    }

    Vector3d getOrigin(int type = 0) {
        if (type == 0) {
            return md_.obstacle_map.map_origin;
        }
        if (type == 1) {
            return md_.guide_map.map_origin;
        }
    }

    typedef std::shared_ptr<DecayMap> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Vector3d alignCoord(const Vector3d &coord, bool inflate = false){
      Vector3d aCoord;
      Vector3i idx;
      if(inflate){
          md_.guide_map.posToIndex(coord, idx);
          md_.guide_map.indexToPos(idx, aCoord);
      }
      else{
          md_.obstacle_map.posToIndex(coord, idx);
          md_.obstacle_map.indexToPos(idx, aCoord);
      }
      return aCoord;
    }

    bool getCurrentOdom(QuadState &out) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if ((ros::Time::now() - md_.odom_.callback_time).toSec() > cfg_.odom_timeout) {
            print(fg(color::indian_red), " -- [DecayMap] No odom.\n");
            return false;
        }
        out = md_.odom_;
        return true;
    };

    bool isOccupied(const Vec3 &pt, bool inflate = true) {
        if (!isInMap(pt)) {
            return true;
        }
        if(!inflate){
            return md_.obstacle_map.isOccupied(pt);
        }
        return md_.obstacle_map.isOccupiedInflate(pt);
    }

    bool isOccupiedInflate(const Vec3 &pt) {
        if (!isInMap(pt)) {
            return true;
        }
        return md_.guide_map.isOccupiedInflate(pt);
    }

    bool isLineFree(const Vec3 &start_pt,
                    const Vec3 &end_pt,
                    const bool in_fov_search = false,
                    const double max_dis = -1) {
        RayCaster raycaster;
        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
        Eigen::Vector3d ray_pt;
        raycaster.setInput(end_pt / md_.obstacle_map.resolution, start_pt / md_.obstacle_map.resolution);

        while (raycaster.step(ray_pt)) {
            Eigen::Vector3d tmp = (ray_pt + half) * md_.obstacle_map.resolution;
            if (max_dis > 0 && (tmp - start_pt).norm() > max_dis) {
                return false;
            }
            if (isOccupied(tmp, in_fov_search)) {
                return false;
            }
        }
        return true;
    }

    void boxSearch(Eigen::Vector3d &box_max,
                   Eigen::Vector3d &box_min,
                   vector<Eigen::Vector3d> &pc, const int skip_point = 0) {
        md_.obstacle_map.boxSearch(box_max, box_min, pc, skip_point);
    }

    void boxSearchInflate(Eigen::Vector3d &box_max,
                          Eigen::Vector3d &box_min,
                          vector<Eigen::Vector3d> &pc, const int skip_point = 0) {
        md_.obstacle_map.boxSearchInflate(box_max, box_min, pc, skip_point);
    }

    void getLocalBoundingBox(vector<Eigen::Vector3d> &pc,
                             const double margin,
                             Eigen::Vector3d &box_max,
                             Eigen::Vector3d &box_min) {
//	TimeConsuming t("getLocalBoundingBox");
        double max_x{-9999}, max_y{-9999}, max_z{-9999};
        double min_x{9999}, min_y{9999}, min_z{9999};

        for (auto it: pc) {
            if (it.x() > max_x) max_x = it.x();
            if (it.y() > max_y) max_y = it.y();
            if (it.z() > max_z) max_z = it.z();

            if (it.x() < min_x) min_x = it.x();
            if (it.y() < min_y) min_y = it.y();
            if (it.z() < min_z) min_z = it.z();
        }
        max_x = min(cfg_.map_max_boundary.x(), max_x + margin);
        max_y = min(cfg_.map_max_boundary.y(), max_y + margin);
        max_z = min(cfg_.map_max_boundary.z(), max_z + margin);

        min_x = max(cfg_.map_min_boundary.x(), min_x - margin);
        min_y = max(cfg_.map_min_boundary.y(), min_y - margin);
        min_z = max(cfg_.map_min_boundary.z(), min_z - margin);

        box_max = Eigen::Vector3d(max_x, max_y, max_z);
        box_min = Eigen::Vector3d(min_x, min_y, min_z);

    }

private:

    // 通过三点获得一个平面
    static void FromPointsToPlane(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3,
                                  Eigen::Vector4d &hPoly) {
        // Each row of hPoly is defined by h0, h1, h2, h3 as
        // h0*x + h1*y + h2*z + h3 <= 0
        hPoly(0) = ((p2.y() - p1.y()) * (p3.z() - p1.z()) - (p2.z() - p1.z()) * (p3.y() - p1.y()));
        hPoly(1) = ((p2.z() - p1.z()) * (p3.x() - p1.x()) - (p2.x() - p1.x()) * (p3.z() - p1.z()));
        hPoly(2) = ((p2.x() - p1.x()) * (p3.y() - p1.y()) - (p2.y() - p1.y()) * (p3.x() - p1.x()));
        hPoly(3) = (0 - (hPoly(0) * p1.x() + hPoly(1) * p1.y() + hPoly(2) * p1.z()));
    }

    void publishGuideMap() {
        if (map_inf_pub_.getNumSubscribers() <= 0)
            return;
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        Eigen::Vector3i min_cut, max_cut;
        Vec3 local_update(cfg_.local_update_distance, cfg_.local_update_distance, cfg_.local_update_distance);
        odom_mutex_.lock();
        md_.guide_map.posToIndex(md_.odom_.position - local_update, min_cut);
        md_.guide_map.posToIndex(md_.odom_.position + local_update, max_cut);
        odom_mutex_.unlock();
        md_.guide_map.boundIndex(min_cut);
        md_.guide_map.boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y)
                for (int z = min_cut(2); z <= max_cut(2); ++z) {
                    if (!md_.guide_map.isOccupiedInflate(md_.guide_map.toAddress(x, y, z)))
                        continue;

                    Eigen::Vector3d pos;
                    md_.guide_map.indexToPos(Eigen::Vector3i(x, y, z), pos);
                    if (pos(2) > cfg_.visualization_truncate_height)
                        continue;

                    pt.x = pos(0);
                    pt.y = pos(1);
                    pt.z = pos(2);
                    cloud.push_back(pt);
                }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = cfg_.frame_id;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";
        map_inf_pub_.publish(cloud_msg);
    }

    void publishObstacleMap() {
        if (map_pub_.getNumSubscribers() <= 0)
            return;
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        Eigen::Vector3i min_cut, max_cut;

        Vec3 local_update(cfg_.local_update_distance, cfg_.local_update_distance, cfg_.local_update_distance);
        odom_mutex_.lock();
        md_.obstacle_map.posToIndex(md_.odom_.position - local_update, min_cut);
        md_.obstacle_map.posToIndex(md_.odom_.position + local_update, max_cut);
        odom_mutex_.unlock();
        md_.obstacle_map.boundIndex(min_cut);
        md_.obstacle_map.boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y)
                for (int z = min_cut(2); z <= max_cut(2); ++z) {
                    if(!md_.obstacle_map.isOccupiedInflate(md_.obstacle_map.toAddress(x, y, z))){
                        continue;
                    }
                    Eigen::Vector3d pos;
                    md_.obstacle_map.indexToPos(Eigen::Vector3i(x, y, z), pos);
                    if (pos(2) > cfg_.visualization_truncate_height)
                        continue;

                    pt.x = pos(0);
                    pt.y = pos(1);
                    pt.z = pos(2);
                    cloud.push_back(pt);
                }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = cfg_.frame_id;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";
        map_pub_.publish(cloud_msg);
    }

    void publishOriginMap() {
        if (map_ori_pub_.getNumSubscribers() <= 0)
            return;
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        Eigen::Vector3i min_cut, max_cut;

        Vec3 local_update(cfg_.local_update_distance, cfg_.local_update_distance, cfg_.local_update_distance);
        odom_mutex_.lock();
        md_.obstacle_map.posToIndex(md_.odom_.position - local_update, min_cut);
        md_.obstacle_map.posToIndex(md_.odom_.position + local_update, max_cut);
        odom_mutex_.unlock();
        md_.obstacle_map.boundIndex(min_cut);
        md_.obstacle_map.boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y)
                for (int z = min_cut(2); z <= max_cut(2); ++z) {
                    if(!md_.obstacle_map.isOccupied(md_.obstacle_map.toAddress(x, y, z))){
                        continue;
                    }
                    Eigen::Vector3d pos;
                    md_.obstacle_map.indexToPos(Eigen::Vector3i(x, y, z), pos);
                    if (pos(2) > cfg_.visualization_truncate_height)
                        continue;

                    pt.x = pos(0);
                    pt.y = pos(1);
                    pt.z = pos(2);
                    cloud.push_back(pt);
                }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = cfg_.frame_id;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";
        map_ori_pub_.publish(cloud_msg);
    }

    void visCallback(const ros::TimerEvent &e/*event*/) {
        publishObstacleMap();
        publishOriginMap();
        publishGuideMap();
    }

    bool isInMap(const Eigen::Vector3d &pos) {
        if (pos(0) < cfg_.map_min_boundary(0) + 1e-4 || pos(1) < cfg_.map_min_boundary(1) + 1e-4 ||
            pos(2) < cfg_.map_min_boundary(2) + 1e-4) {
            // cout << "less than min range!" << endl;
            return false;
        }
        if (pos(0) > cfg_.map_max_boundary(0) - 1e-4 || pos(1) > cfg_.map_max_boundary(1) - 1e-4 ||
            pos(2) > cfg_.map_max_boundary(2) - 1e-4) {
            return false;
        }
        return true;
    }

    void updateLocalMap() {
        // NOTE: parallel map update
        double rcv_t = md_.pc_rcv_time_.toSec();
        double time_now = ros::Time::now().toSec();
        TimeConsuming tt4("tbb::parallel_for_each updateLocalMap");
        tt4.set_enbale(cfg_.print_update_time);
        odom_mutex_.lock();
        Vec3 cur_position = md_.odom_.position;
        odom_mutex_.unlock();
//	tbb::parallel_for_each(md_.latest_pc_.begin(), md_.latest_pc_.end(), [&](auto &pt) {
        std::for_each(md_.latest_pc_.begin(), md_.latest_pc_.end(), [&](auto &pt) {
            Vec3 temp_p(pt.x, pt.y, pt.z);
            if ((temp_p - cur_position).norm() < cfg_.local_update_distance &&
                isInMap(temp_p)) {
                // First update obstacle map, non inflation part
                Eigen::Vector3i temp_id;
                md_.obstacle_map.posToIndex(temp_p, temp_id);
                int address = md_.obstacle_map.toAddress(temp_id);
                if (address < md_.obstacle_map.max_address && address >= 0) {
                    if (cfg_.decay_time == 0) {
                        md_.obstacle_map.occupancy_buffer[address].is_occupied = -1;
                        md_.obstacle_map.updateInflation(temp_p, 0);
                        md_.guide_map.updateInflation(temp_p, 0);
                    } else if (md_.obstacle_map.occupancy_buffer[address].is_occupied != -1) {
                        int &update_id = md_.obstacle_map.occupancy_buffer[address].time_update_id;
                        vector<double> &times = md_.obstacle_map.occupancy_buffer[address].point_time;
                        update_id++;
                        if (update_id == (cfg_.hit_point_buffer_size)) {
                            update_id = 0;
                        }
                        times[update_id] = rcv_t;
                        md_.obstacle_map.occupancy_buffer[address].point_hit_num++;
                        int cnt = 0;
                        double min_effictive_time = 0;
                        // 从当前id往前搜索，找到前n个有效时间
                        for (int i = update_id;; i--) {
                            if (i < 0) {
                                i += cfg_.hit_point_buffer_size;
                            }
                            if (time_now - times[i] < cfg_.decay_time) {
                                cnt++;
                                if (cnt >= cfg_.hit_num_thresh) {
                                    min_effictive_time = times[i];
                                    // 找够了 break
                                    break;
                                }
                            }
                            if (i == update_id + 1 || (update_id == cfg_.hit_point_buffer_size - 1 && i == 0)) {
                                // 最后一个点
                                break;
                            }
                        }
                        if (min_effictive_time > 0) {
                            md_.obstacle_map.occupancy_buffer[address].update_time = min_effictive_time;
                            md_.obstacle_map.occupancy_buffer[address].is_occupied = 1;
                            md_.obstacle_map.updateInflation(temp_p, min_effictive_time);
                            md_.guide_map.updateInflation(temp_p, min_effictive_time);
                        }
                    }
                }

            }
        });

    }

    mutex odom_mutex_;

    void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
        odom_mutex_.lock();
        md_.odom_.position = Vec3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        md_.odom_.q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

        Eigen::Vector3d eulerAngle = md_.odom_.q.matrix().eulerAngles(0, 1, 2);
        md_.odom_.yaw = eulerAngle(2);
        md_.odom_.callback_time = msg->header.stamp;
        md_.odom_.rcv = true;
        odom_mutex_.unlock();
    };

    mutex cloud_mutex_;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
        cloud_mutex_.lock();
        pcl::fromROSMsg(*msg, md_.latest_pc_);

        int plsize = md_.latest_pc_.size();
        if (cfg_.print_update_time) {
            print(" -- [DecayMap] Update {} pts.\n", plsize);
        }
        if (plsize > 0) {
            md_.new_cloud_rcv_ = true;
            md_.pc_rcv_time_ = msg->header.stamp;
            updateLocalMap();
        }
        cloud_mutex_.unlock();

    }

};

#endif