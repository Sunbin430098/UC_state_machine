//
// Created by yunfan on 2022/6/25.
//

#ifndef UTILS_VISUALIZATION_UTILS_HPP
#define UTILS_VISUALIZATION_UTILS_HPP

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

class Color : public std_msgs::ColorRGBA {
public:
    Color() : std_msgs::ColorRGBA() {}

    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}

    Color(double red, double green, double blue, double alpha) : Color() {
        r = red;
        g = green;
        b = blue;
        a = alpha;
    }

    static const Color White() { return Color(1.0, 1.0, 1.0); }

    static const Color Black() { return Color(0.0, 0.0, 0.0); }

    static const Color Gray() { return Color(0.5, 0.5, 0.5); }

    static const Color Red() { return Color(1.0, 0.0, 0.0); }

    static const Color Green() { return Color(0.0, 0.96, 0.0); }

    static const Color Blue() { return Color(0.0, 0.0, 1.0); }

    static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }

    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }

    static const Color Orange() { return Color(1.0, 0.5, 0.0); }

    static const Color Purple() { return Color(0.5, 0.0, 1.0); }

    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }

    static const Color Teal() { return Color(0.0, 1.0, 1.0); }

    static const Color Pink() { return Color(1.0, 0.0, 0.5); }

    static const Color SE3_color() { return Color( 255.0/255.0, 193.0/255.0, 37.0/255.0); }

    static const Color R3_color() { return Color( 30.0/255.0, 144.0/255.0, 255.0/255.0); }
};

namespace VisualUtils {

    static void DeleteMkrArr(ros::Publisher &pub_) {
        visualization_msgs::Marker del;
        visualization_msgs::MarkerArray arr;
        del.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(del);
        pub_.publish(arr);
    }

    static void DeleteMkr(ros::Publisher &pub_) {
        visualization_msgs::Marker del;
        del.action = visualization_msgs::Marker::DELETEALL;
        pub_.publish(del);
    }

    // @msg: visualization_msgs::MarkerArray
    static void VisualizePath(ros::Publisher &pub_,vector<Vec3> &path,
                              string ns = "path",
                              Color color = Color::Pink(),
                              double pt_size = 0.1,
                              double line_size = 0.05) {
        visualization_msgs::Marker line_list;
        visualization_msgs::MarkerArray mkr_ary;
        if (path.size() <= 0) {
            print(fg(color::gold), " -- [Viz] Try to publish empty path, return.\n");
            return;
        }
        Vec3 cur_pt = path[0], last_pt;
        static int point_id = 0;
        static int line_cnt = 0;
        for (size_t i = 0; i < path.size(); i++) {
            last_pt = cur_pt;
            cur_pt = path[i];

            /* Publish point */
            visualization_msgs::Marker point;
            point.header.frame_id = "world";
            point.header.stamp = ros::Time::now();
            point.ns = ns.c_str();
            point.id = point_id++;
            point.action = visualization_msgs::Marker::ADD;
            point.pose.orientation.w = 1.0;
            point.type = visualization_msgs::Marker::SPHERE;
            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            point.scale.x = pt_size;
            point.scale.y = pt_size;
            point.scale.z = pt_size;
            // Line list is blue
            point.color = color;
            point.color.a = 1.0;
            // Create the vertices for the points and lines
            geometry_msgs::Point p;
            p.x = cur_pt.x();
            p.y = cur_pt.y();
            p.z = cur_pt.z();
            point.pose.position = p;
            mkr_ary.markers.push_back(point);
            /* publish lines */
            if (i > 0) {
                geometry_msgs::Point p;
                // publish lines
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "world";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = ns + "_line";
                line_list.id = line_cnt++;
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.pose.orientation.w = 1.0;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                line_list.scale.x = line_size;
                // Line list is blue
                line_list.color = color;
                // Create the vertices for the points and lines

                p.x = last_pt.x();
                p.y = last_pt.y();
                p.z = last_pt.z();
                // The line list needs two points for each line
                line_list.points.push_back(p);
                p.x = cur_pt.x();
                p.y = cur_pt.y();
                p.z = cur_pt.z();
                // The line list needs
                line_list.points.push_back(p);

                mkr_ary.markers.push_back(line_list);
            }
        }
        pub_.publish(mkr_ary);
    }


    // @msg: visualization_msgs::MarkerArray
    static inline void VisualizeTrajectory(ros::Publisher pub_, vector<Vec3> &traj,
                                           string ns = "traj",
                                           Color color = Color::SteelBlue(),
                                           double eval_dt = 0.03,
                                           double size = 0.2
    ) {
        Vec3 last_pos, cur_pos, end_point;
        visualization_msgs::Marker line_list;
        visualization_msgs::MarkerArray mrkarr;
        mrkarr.markers.clear();
        static int idx = 0;
        last_pos = traj[0];
        for (auto cur_pos: traj) {
            visualization_msgs::Marker line_list;
            {
                static int cnt = 0;
                // publish lines
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "world";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = ns;
                line_list.id = cnt++;
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.pose.orientation.w = 1.0;
                line_list.type = visualization_msgs::Marker::ARROW;
                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                line_list.scale.x = size;
                line_list.scale.y = 0.0000001;
                line_list.scale.z = 0.0000001;
                // Line list is blue
                line_list.color = color;
                line_list.color.a = 1;
                // Create the vertices for the points and lines

                geometry_msgs::Point p;
                p.x = last_pos.x();
                p.y = last_pos.y();
                p.z = last_pos.z();
                // The line list needs two points for each line
                line_list.points.push_back(p);
                p.x = cur_pos.x();
                p.y = cur_pos.y();
                p.z = cur_pos.z();
                // The line list needs
                line_list.points.push_back(p);

                mrkarr.markers.push_back(line_list);
            }
            last_pos = cur_pos;
        }
        pub_.publish(mrkarr);
    }

    // @msg: visualization_msgs::MarkerArray
    static void VisualizeCube(ros::Publisher &pub_, const Vec3 &top_left,
                              const Vec3 &but_right,
                              Color color = Color::Purple(),
                              double alpha = 0.3,
                              string ns = "cube") {
        Vec3 center = (top_left + but_right) / 2;
        Vec3 size = (top_left - but_right).cwiseAbs();
        visualization_msgs::Marker mkr;
        static int cube_id = 0;
        mkr.header.frame_id = "world";
        mkr.header.stamp = ros::Time::now();
        mkr.ns = ns.c_str();
        mkr.id = cube_id++;
        mkr.action = visualization_msgs::Marker::ADD;
        mkr.pose.orientation.w = 1.0;
        mkr.type = visualization_msgs::Marker::CUBE;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        mkr.scale.x = size.x();
        mkr.scale.y = size.y();
        mkr.scale.z = size.z();
        // Line list is blue
        mkr.color = color;
        mkr.color.a = alpha;
        // Create the vertices for the points and lines
        geometry_msgs::Point p;
        p.x = center.x();
        p.y = center.y();
        p.z = center.z();
        mkr.pose.position = p;

        visualization_msgs::MarkerArray mkr_arr;
        mkr_arr.markers.push_back(mkr);
        pub_.publish(mkr_arr);

    }

    // @msg: visualization_msgs::MarkerArray
    static void VisualizeGoal(ros::Publisher &goal_pub_, const Vec3 &local_goal, const Vec3 &global_goal) {
        static int id = 0;
        visualization_msgs::Marker marker_ball;
        Vec3 cur_pos = local_goal;
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = ros::Time::now();
        marker_ball.ns = "local_goal";
        marker_ball.id = id++;
        marker_ball.action = visualization_msgs::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::Marker::SPHERE;
        marker_ball.scale.x = 0.3;
        marker_ball.scale.y = 0.3;
        marker_ball.scale.z = 0.3;
        marker_ball.color = Color::Chartreuse();
        marker_ball.color.a = 0.8;

        geometry_msgs::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;

        goal_pub_.publish(marker_ball);

        cur_pos = global_goal;
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = ros::Time::now();
        marker_ball.ns = "global_goal";
        marker_ball.id = id;
        marker_ball.action = visualization_msgs::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::Marker::SPHERE;
        marker_ball.scale.x = 0.3;
        marker_ball.scale.y = 0.3;
        marker_ball.scale.z = 0.3;
        marker_ball.color = Color::Orange();
        marker_ball.color.a = 0.8;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();
        marker_ball.pose.position = p;
        goal_pub_.publish(marker_ball);
    }


    // @msg: sensor_msgs::PointCloud2
    void VisualizePointsInPointCloud(ros::Publisher pub_, const vector<Vec3> &pts) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        for (auto it : pts) {
            pc.push_back(pcl::PointXYZ(it.x(), it.y(), it.z()));
        }
        pc.header.frame_id = "world";
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(pc, pc2);
        pc2.header.frame_id = "world";
        pc2.header.stamp = ros::Time::now();
        pub_.publish(pc2);
    }

}


#endif //DECAY_MAP_VISUALIZATION_UTILS_H
