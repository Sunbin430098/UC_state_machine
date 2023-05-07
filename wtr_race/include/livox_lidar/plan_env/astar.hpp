#ifndef _DECAY_MAP_ASTAR_
#define _DECAY_MAP_ASTAR_

#include "Eigen/Dense"
#include "vector"
#include "decay_map.hpp"
#include "queue"
#include "ros/ros.h"
#include "livox_lidar/utils/visualization_utils.hpp"

namespace astar {
    constexpr double inf = 1 >> 20;
    struct GridNode;
    typedef GridNode *GridNodePtr;

    struct GridNode {
        enum enum_state {
            OPENSET = 1,
            CLOSEDSET = 2,
            UNDEFINED = 3
        };

        int rounds{0}; // Distinguish every call
        enum enum_state state
                {
                        UNDEFINED
                };
        Eigen::Vector3i index;

        double gScore{inf}, fScore{inf};
        GridNodePtr cameFrom{NULL};
    };

    class NodeComparator {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2) {
            return node1->fScore > node2->fScore;
        }
    };

    class Astar {
    private:
        ros::Publisher mkr_pub_, wpt_pub_;
        DecayMap::Ptr dcm_ptr_;
        double step_size_, inv_step_size_;
        Eigen::Vector3d center_;
        Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
        const double tie_breaker_ = 1.0 + 1.0 / 10000;

        std::vector<GridNodePtr> gridPath_;

        GridNodePtr ***GridNodeMap_;
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

        int rounds_{0};
        bool forceTimeout_;
        bool debug_;
        bool in_fov_search_;
        bool highreso_inf;
        double lambda_heu;
        enum HeuType {
            DIAG = 0,
            MANH,
            EUCL
        };

        double getHeu(GridNodePtr node1, GridNodePtr node2, HeuType type = DIAG) {
            switch (type) {
                case DIAG: {
                    double dx = abs(node1->index(0) - node2->index(0));
                    double dy = abs(node1->index(1) - node2->index(1));
                    double dz = abs(node1->index(2) - node2->index(2));

                    double h = 0.0;
                    int diag = min(min(dx, dy), dz);
                    dx -= diag;
                    dy -= diag;
                    dz -= diag;

                    if (dx == 0) {
                        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
                    }
                    if (dy == 0) {
                        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
                    }
                    if (dz == 0) {
                        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
                    }
                    return tie_breaker_ * h * lambda_heu;
                }
                case MANH: {
                    double dx = abs(node1->index(0) - node2->index(0));
                    double dy = abs(node1->index(1) - node2->index(1));
                    double dz = abs(node1->index(2) - node2->index(2));

                    return tie_breaker_ * (dx + dy + dz);
                }
                case EUCL: {
                    return tie_breaker_ * (node2->index - node1->index).norm();
                }
                default: {
                    ROS_ERROR(" -- [A*] Wrong hue type");
                    return 0;
                }

            }
        }

        Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const {
            return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
        };

        bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx, bool use_inf) const {
//	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;
            Vector3d p_aligned = dcm_ptr_->alignCoord(pt, use_inf);
            for (int i = 0; i < 3; ++i) {
                idx(i) = floor((p_aligned(i) - center_(i)) * inv_step_size_+ 0.5) + CENTER_IDX_[i];
            }
            if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 ||
                idx(2) >= POOL_SIZE_(2)) {
                ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
                return false;
            }

            return true;
        };


//  Eigen::Vector3d Index2Coord(const Eigen::Vector3i &id) const {
//	Vector3d pos;
//	for (int i = 0; i < 3; ++i) {
//	  pos(i) = (id(i) + 0.5) * step_size_ +
//		  center_(i);
//	}
//	return pos;
//  };
//
//  bool Coord2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &idx) const {
//	for (int i = 0; i < 3; ++i) {
//	  idx(i) = floor((pos(i) - center_(i)) * inv_step_size_);
//	}
//  };


    public:
        Astar( ros::NodeHandle &nh) {
            nh.param("astar/force_timeout", forceTimeout_, true);
            nh.param("astar/debug", debug_, false);
            nh.param("astar/highreso_inf", highreso_inf, false);
            nh.param("astar/lambda_heu", lambda_heu, 1.0);

            mkr_pub_ = nh.advertise<visualization_msgs::MarkerArray>("astar/debug", 10000);
            wpt_pub_ = nh.advertise<visualization_msgs::MarkerArray>("astar/waypt", 10000);
        }
        ~Astar() {}
        typedef shared_ptr<Astar> Ptr;
        void initMap(DecayMap::Ptr dcm) {

            POOL_SIZE_ = dcm->getPoolSize();
            CENTER_IDX_ = POOL_SIZE_ / 2;

            GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
            for (int i = 0; i < POOL_SIZE_(0); i++) {
                GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
                for (int j = 0; j < POOL_SIZE_(1); j++) {
                    GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
                    for (int k = 0; k < POOL_SIZE_(2); k++) {
                        GridNodeMap_[i][j][k] = new GridNode;
                    }
                }
            }

            dcm_ptr_ = dcm;
        }

        bool isOccupied(const Eigen::Vector3d &pos, bool use_inf = true) {
            if (use_inf) {
                return dcm_ptr_->isOccupiedInflate(pos, in_fov_search_);
            }
            return dcm_ptr_->isOccupied(pos, in_fov_search_, highreso_inf);
        }

        enum SearchType{
            LOW_RESO = 0,         // Connect from start to end (using guide_map)
            ESCAPE = 1,         // Find a path from start(in obstacle) to free (using obstacle_map)
            FREE_END = 2,       // Find first point out of searching horizon (using obstacle_map)
            HIGH_RESO = 3       // Connect from start to end (using obstacle_map)
        };

        bool AstarSearch(SearchType st,        //  distance per grid
                         Vector3d start_pt,            // start_point
                         Vector3d end_pt,                // goal_point
                         double &timeConsume,              // time consume
                         double timeout = 0.0,              // unit: s, force timeout 0->no timeout
                         double searching_horizon = -1,    // if > 0 the algo will return the first point who out of the searching horizon
                         bool in_fov_search = false,
                         bool use_plane_expansion = false,    // if true, the searcher will expand on plane only
                         bool allow_diag = true                // if true, the expansion can be diag
        ){

            in_fov_search_ = in_fov_search;
            string tType;
            switch (st) {
                case LOW_RESO:{
                    tType = "LR Astar";
                    break;
                }
                case HIGH_RESO:{
                    tType = "HR Astar";
                    break;
                }
                case ESCAPE: {
                    tType = "EscapeAstar";
                    break;
                }
                case FREE_END:{
                    tType = "FreeEndAstar";
                    break;
                }
            }
            TimeConsuming astR(tType, false);

            ros::Time time_1 = ros::Time::now();
            ++rounds_;
            int dz_range = 1;

            if (use_plane_expansion) {
                end_pt.z() = start_pt.z();
                dz_range = 0;
            }
            bool use_inf = true;
            switch(st){
                case LOW_RESO:{
                    step_size_ = dcm_ptr_->getResolution(1);
                    break;
                }
                case HIGH_RESO:
                case ESCAPE:{
                    step_size_ = dcm_ptr_->getResolution(0);
                    use_inf = false;
                    break;
                }
                case FREE_END:{
                    step_size_ = dcm_ptr_->getResolution();
                    if(searching_horizon <0){
                        print(fg(color::indian_red), " -- [WARN] Freeend but horizon is not given, force return.\n");
                        timeConsume = -1;
                        return false;
                    }
                    break;
                }
                default:
                {
                    step_size_ = dcm_ptr_->getResolution();
                }
            }

            inv_step_size_ = 1 / step_size_;
            // Make the center axis align
            center_ = dcm_ptr_->alignCoord((start_pt + end_pt) / 2, use_inf);
//    if(debug_ && st == LOW_RESO){
//        cout << "start:\n"<< start_pt << "\nend:\n" << end_pt << "\ncenter:\n" << (start_pt + end_pt) / 2 << "\naligned center:\n" << center_ << endl;
//    }

            int horizon_idx = ceil(searching_horizon / step_size_);
            Vector3i start_idx, end_idx;
            if (!Coord2Index(start_pt, start_idx, use_inf) ||
                !Coord2Index(end_pt, end_idx, use_inf)) {
                print(fg(color::red), " -- [Astar] Wrong start point or end point.\n");
                timeConsume = -1;
                return false;
            }
//    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx,free_end_pt)) {
//
//        return false;
//    }

            // if ( start_pt(0) > -1 && start_pt(0) < 0 )
            //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

            GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
            GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];
            if(debug_ && st == LOW_RESO){

                {
                    static int point_id = 0;
                    visualization_msgs::Marker point;
                    visualization_msgs::MarkerArray mkr_ary;
                    point.header.frame_id = "world";
                    point.ns = "astar_wpt_"+to_string(point_id);
                    point.id = point_id++;
                    point.action = visualization_msgs::Marker::ADD;
                    point.pose.orientation.w = 1.0;
                    point.type = visualization_msgs::Marker::CUBE;
                    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                    point.scale.x = step_size_;
                    point.scale.y = step_size_;
                    point.scale.z = step_size_;
                    // Line list is blue

                    point.color.a = 1;
//                    point.color.r = 125;
//                    point.color.g = 0;
//                    point.color.b = 125;
                    // Create the vertices for the points and lines
                    geometry_msgs::Point p;
                    auto pp = Index2Coord(start_idx);
                    p.x = pp.x();
                    p.y = pp.y();
                    p.z = pp.z();
                    point.pose.position = p;
                    point.color = Color::Red();
                    point.color.a = 1;
                    mkr_ary.markers.push_back(point);

                    point.header.frame_id = "world";
                    point.id = point_id++;
                    // Create the vertices for the points and lines
                    pp = Index2Coord(end_idx);
                    p.x = pp.x();
                    p.y = pp.y();
                    p.z = pp.z();
                    point.pose.position = p;
                    point.color = Color::Orange();
                    point.color.a = 1;
                    mkr_ary.markers.push_back(point);
                    wpt_pub_.publish(mkr_ary);
                }

                if (isOccupied(Index2Coord(start_idx),1)) {
                    print(fg(color::red), " -- [Astar] Start Index in obstacle.\n");
//          cout << "original:\n" << start_pt << endl << "idx2coord" << endl << Index2Coord(start_idx) << endl;
                    timeConsume = -1;
                    return false;
                }
                if (isOccupied(Index2Coord(end_idx),1)) {
                    print(fg(color::red), " -- [Astar] End Index in obstacle.\n");
//          cout << "original:\n" << end_pt << endl << "idx2coord" << endl << Index2Coord(end_idx) << endl;
                    timeConsume = -1;
                    return false;
                }
            }

            std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
            openSet_.swap(empty);

            GridNodePtr neighborPtr = NULL;
            GridNodePtr current = NULL;

            startPtr->index = start_idx;
            startPtr->rounds = rounds_;
            startPtr->gScore = 0;
            startPtr->fScore = getHeu(startPtr, endPtr);
            if(st == ESCAPE){
                startPtr->fScore = 0;
            }
            startPtr->state = GridNode::OPENSET; //put start node in open set
            startPtr->cameFrom = NULL;
            openSet_.push(startPtr); //put start in open set

            endPtr->index = end_idx;

            double tentative_gScore;

            int num_iter = 0;
            visualization_msgs::MarkerArray mkr_ary;
            while (!openSet_.empty()) {
                if ( timeout > 1e-5) {
                    ros::Duration dur = ros::Time::now() - time_1;
                    if(dur.toSec() > timeout && forceTimeout_){
                        timeConsume = dur.toSec();
                        mkr_pub_.publish(mkr_ary);
                        return false;
                    }
                }
                num_iter++;
                current = openSet_.top();

                if(debug_ && st == LOW_RESO){
                    static int point_id = 0;
                    visualization_msgs::Marker point;
                    point.header.frame_id = "world";
                    point.ns = "astar_debug";
                    point.id = point_id++;
                    point.action = visualization_msgs::Marker::ADD;
                    point.pose.orientation.w = 1.0;
                    point.type = visualization_msgs::Marker::CUBE;
                    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                    point.scale.x = step_size_;
                    point.scale.y = step_size_;
                    point.scale.z = step_size_;
                    // Line list is blue

                    point.color.a = 0.5;
                    point.color.r = 0;
                    point.color.g = 200;
                    point.color.b = 250;
                    // Create the vertices for the points and lines
                    geometry_msgs::Point p;
                    auto pp = Index2Coord(current->index);
                    p.x = pp.x();
                    p.y = pp.y();
                    p.z = pp.z();
                    point.pose.position = p;
                    mkr_ary.markers.push_back(point);
                }

                if(debug_ && st == HIGH_RESO){
                    static int point_id = 0;
                    visualization_msgs::Marker point;

                    point.header.frame_id = "world";
                    point.ns = "hr_astar_debug";
                    point.id = point_id++;
                    point.action = visualization_msgs::Marker::ADD;
                    point.pose.orientation.w = 1.0;
                    point.type = visualization_msgs::Marker::CUBE;
                    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                    point.scale.x = step_size_;
                    point.scale.y = step_size_;
                    point.scale.z = step_size_;
                    // Line list is blue

                    point.color.a = 0.5;
                    point.color.r = 0.5;
                    point.color.g = 0.5;
                    point.color.b = 0.3;
                    // Create the vertices for the points and lines
                    geometry_msgs::Point p;
                    auto pp = Index2Coord(current->index);
                    p.x = pp.x();
                    p.y = pp.y();
                    p.z = pp.z();
                    point.pose.position = p;
                    mkr_ary.markers.push_back(point);
                }


                openSet_.pop();

                if (current->index(0) == endPtr->index(0) &&
                    current->index(1) == endPtr->index(1) &&
                    current->index(2) == endPtr->index(2)) {
                    gridPath_ = retrievePath(current);
                    timeConsume = astR.stop();
                    mkr_pub_.publish(mkr_ary);
                    return true;
                }

                // Distance terminate condition
                if (st==FREE_END && (current->index - start_idx).norm() > horizon_idx) {
                    gridPath_ = retrievePath(current);
                    timeConsume = astR.stop();
                    mkr_pub_.publish(mkr_ary);
                    return true;
                }

                // Escape terminate  conditrion
                if (st == ESCAPE  && !isOccupied(Index2Coord(current->index))) {
                    gridPath_ = retrievePath(current);
                    timeConsume = astR.stop();
                    mkr_pub_.publish(mkr_ary);
                    return true;
                }

                current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dz = -dz_range; dz <= dz_range; dz++) {
                            if (!allow_diag &&
                                abs(dx) + abs(dy) + abs(dz) != 1) {
                                continue;
                            }
                            if (dx == 0 && dy == 0 && dz == 0){
                                continue;
                            }


                            Vector3i neighborIdx;
                            neighborIdx(0) = (current->index)(0) + dx;
                            neighborIdx(1) = (current->index)(1) + dy;
                            neighborIdx(2) = (current->index)(2) + dz;

                            if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 ||
                                neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 ||
                                neighborIdx(2) >= POOL_SIZE_(2) - 1) {
                                continue;
                            }

                            neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                            neighborPtr->index = neighborIdx;

                            bool flag_explored = neighborPtr->rounds == rounds_;

                            if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                                continue; //in closed set.
                            }

                            neighborPtr->rounds = rounds_;

                            if (isOccupied(Index2Coord(neighborPtr->index), use_inf)) {
                                continue;
                            }

                            double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                            tentative_gScore = current->gScore + static_cost;

                            double heu_score = getHeu(neighborPtr, endPtr);
                            if(st == ESCAPE){
                                heu_score = 0;
                            }
                            if (!flag_explored) {
                                //discover a new node
                                neighborPtr->state = GridNode::OPENSET;
                                neighborPtr->cameFrom = current;
                                neighborPtr->gScore = tentative_gScore;
                                neighborPtr->fScore = tentative_gScore + heu_score;
                                openSet_.push(neighborPtr); //put neighbor in open set and record it.
                            } else if (tentative_gScore < neighborPtr->gScore) { //in open set and need update
                                neighborPtr->cameFrom = current;
                                neighborPtr->gScore = tentative_gScore;
                                neighborPtr->fScore = tentative_gScore + heu_score;
                            }
                        }
                ros::Time time_2 = ros::Time::now();
                if ((time_2 - time_1).toSec() > 0.2 && (st != FREE_END && st != HIGH_RESO )) {
                    ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
                    timeConsume = -1;
                    mkr_pub_.publish(mkr_ary);
                    return false;
                }
            }

            ros::Time time_2 = ros::Time::now();

            if ((time_2 - time_1).toSec() > 0.1)
                ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);
            mkr_pub_.publish(mkr_ary);
            return false;
        }

        vector<GridNodePtr> retrievePath(GridNodePtr current) {
            vector<GridNodePtr> path;
            path.push_back(current);

            while (current->cameFrom != NULL) {
                current = current->cameFrom;
                path.push_back(current);
            }

            return path;
        }

        vector<Vector3d> getPath() {
            vector<Vector3d> path;

            for (auto ptr : gridPath_)
                path.push_back(Index2Coord(ptr->index));

            reverse(path.begin(), path.end());
            return path;
        }

    };
}

#endif
