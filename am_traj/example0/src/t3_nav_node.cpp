//
// Created by stdcat on 7/8/21.
//

#include "common_include.h"

#include "nav/am_traj.hpp"

#include "nav/GridMap.h"
#include "nav/AStar.h"
#include "math.h"
#include "config.h"

typedef enum{
    baseStop = 0, // 急停
    baseMode1,    // 跑到箭架前
    baseMode2,    // 撞箭架
    baseMode3,    // 后退一段距离
    baseZero,     // 调零
    baseManual,  // 手动
    baseNone      // 其他
}baseState_e;


Config cfg;

using namespace Eigen;
using namespace std;

AmTraj::Ptr am_traj_ptr;
Trajectory traj;

GridMap_t *gMap = nullptr;
AStar *astar = nullptr;

int isSet = 0;

Vector3d globGoal(0,0,0);
Vector3d curOdom(0,0,0);
Vector3d curVel(0,0,0);

geometry_msgs::PoseStamped frontJianJia, backJianJia, leaveJianJia;
geometry_msgs::Twist joyTwist;

ros::Publisher velPub,
                path_pub,
                astar_path_pub,
                astar_wp_pub,
                base_vel_pub,
                base_cmd_pub,
                goal_pub,
                base_fdb_pub,
                map_pub,
                base_state_pub,
                goal_odom_pub;

ros::Time sTime;

baseState_e curState = baseManual;



double angleMainValue( double  src){
    while( src < -pi ) src += pi;
    while( src > pi )  src -= pi;
    return src;
}

double errYawSum = 0;
double errXSum = 0;
double errYSum = 0;
bool isPubReachGoal = false;

void limitRange( double &src, double _min, double _max ){
    if(src < _min) src = _min;
    if(src > _max) src = _max;
}

void velCtrlThread_Callback( const ros::TimerEvent &e ){
    geometry_msgs::Twist ts;

    if( curState == baseManual ){
        ts.linear.x = joyTwist.linear.x;
        ts.linear.y = joyTwist.linear.y;
        ts.angular.z = joyTwist.angular.z;
    }
    else if( curState == baseZero || curState == baseNone || curState == baseStop ){
        ts.linear.x = ts.linear.y = ts.angular.z = 0;
    }
    else if(isSet == 2){// navigation
        ros::Time curTime = e.current_real;
        ros::Duration dur = curTime - sTime;
        double curDur = dur.toSec() > traj.getTotalDuration() ? traj.getTotalDuration():dur.toSec();

        Vector3d goalPos = traj.getPos(curDur);
        Vector3d goalVel = traj.getVel(curDur);
        Vector3d goalAcc = traj.getAcc(curDur);

        nav_msgs::Odometry goalOdom;
        goalOdom.header.frame_id = "map";
        goalOdom.pose.pose.position.x = goalPos.x();
        goalOdom.pose.pose.position.y = goalPos.y();
        goalOdom.twist.twist.linear.x = goalVel.x();
        goalOdom.twist.twist.linear.y = goalVel.y();

        goal_odom_pub.publish(goalOdom);


        double outVx, outVy, outVz;
        errYawSum += angleMainValue(globGoal.z() - curOdom.z());
        errXSum += goalPos.x() - curOdom.x();
        errYSum += goalPos.y() - curOdom.y();
        outVz = (cfg.KP_YAW * angleMainValue( globGoal.z() - curOdom.z() ) + cfg.KI_YAW * errYawSum );

        outVx = goalVel.x()*cfg.KP_FeedForward + cfg.KP_X * (goalPos.x() - curOdom.x() ) + cfg.KI_X * errXSum + cfg.KP_VX * (goalVel.x() - curVel.x() ) ;

        outVy = goalVel.y()*cfg.KP_FeedForward + cfg.KP_Y * (goalPos.y() - curOdom.y() ) + cfg.KI_Y * errYSum + cfg.KP_VY * (goalVel.y() - curVel.y() );




        ts.linear.x = outVx * cos(curOdom.z()) + outVy * sin(curOdom.z());
        ts.linear.y = outVy * cos(curOdom.z()) - outVx * sin(curOdom.z());
        ts.angular.z = outVz;

        limitRange(ts.linear.x, -1*cfg.safe_maxVel, cfg.safe_maxVel);
        limitRange(ts.linear.y, -1*cfg.safe_maxVel, cfg.safe_maxVel);
        limitRange(ts.angular.z, -1*cfg.safe_maxOmega, cfg.safe_maxOmega);

        int isReach = 0;
        if( sqrt( pow( curOdom.x() - globGoal.x(), 2 )+pow( curOdom.y() - globGoal.y(), 2 ) ) < cfg.minDisErr ){
            ts.linear.x = ts.linear.y = 0;
            isReach ++;
        }
        if( fabs(angleMainValue(globGoal.z() - curOdom.z())) < cfg.minYawErr*pi/180.0f ){
            ts.angular.z = 0;
            isReach ++;
        }

        if( isReach == 2 && isPubReachGoal == false ){
            isPubReachGoal = true;
            std_msgs::UInt8 u8Msg;
            u8Msg.data = 1;
            base_fdb_pub.publish(u8Msg);
            curState = baseManual;
        }

    }


    if(cfg.isPubVel){
        velPub.publish(ts);
        base_vel_pub.publish(ts);
    }


}

void publishTrajToNav(Trajectory traj_){

    nav_msgs::Path path_msg;
    double cur_t = 0.0;

    while(cur_t < traj_.getTotalDuration()){
        cur_t += 0.1;
        Vector3d cur = traj_.getPos(cur_t);
        Vector3d cVel = traj_.getVel(cur_t);
        geometry_msgs::PoseStamped ps;

        ps.pose.position.x = cur.x();
        ps.pose.position.y = cur.y();
        ps.pose.position.z = cur.z();
        path_msg.poses.push_back(ps);

//        nav_msgs::Odometry pVel;
//        pVel.header.frame_id = "world";
//        pVel.pose.pose.position.x = cur.x();
//        pVel.pose.pose.position.y = cur.y();
//        pVel.pose.pose.position.z = 0;
//        double yaw = atan2(cVel.y(), cVel.x());
//        tf::Quaternion quat =  tf::createQuaternionFromYaw(yaw);
//        tf::quaternionTFToMsg(quat, pVel.pose.pose.orientation);
//        pVel.twist.twist.linear.x = sqrt(pow(cVel.x(),2)+pow(cVel.y(),2));
//        vel_visual_pub.publish(pVel);

    }

    path_msg.header.frame_id = "map";
    path_pub.publish(path_msg);

}

void visualAStarWayPoint(ros::Publisher pub, vector<Vector3d> wps){
    visualization_msgs::MarkerArray markers;
    static int preSize = 0;
    int _id = 0;
    for (int i = 0; i < wps.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "AStar_waypoint";
        marker.id = _id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1;
        marker.color.r = 255;
        marker.color.g = 0;
        marker.color.b = 0;

        marker.pose.position.x = wps[i].x();
        marker.pose.position.y = wps[i].y();
        marker.pose.position.z = 0;


        markers.markers.push_back(marker);
    }
    if(wps.size() < preSize){
        for (int i = wps.size(); i < preSize; ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.ns = "AStar_waypoint";
            marker.id = _id++;
            marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(marker);
        }
    }
    preSize = wps.size();
    pub.publish(markers);

}

void generate_Traj(vector<Vector3d> wayPoints){
    Vector3d zeroVec(0.0, 0.0, 0.0);
    ros::Time t1 = ros::Time::now();
    traj = am_traj_ptr->genOptimalTrajDTC(wayPoints, zeroVec, zeroVec, zeroVec, zeroVec);
    ros::Time t2 = ros::Time::now();
    ROS_WARN("Finish generate traj with time: %lf ms", (double)((t2-t1).toSec())*1e3);
    ROS_WARN("traj Total Duration: %lf s", traj.getTotalDuration());

    publishTrajToNav(traj);

}

void generate_Traj(vector<Vector3d> wayPoints, Vector3d curVel, Vector3d curAcc){
    Vector3d zeroVec(0.0, 0.0, 0.0);
    ros::Time t1 = ros::Time::now();
    traj = am_traj_ptr->genOptimalTrajDTC(wayPoints, curVel, curAcc, zeroVec, zeroVec);
    ros::Time t2 = ros::Time::now();
    ROS_WARN("Finish generate traj with time: %lf ms", (double)((t2-t1).toSec())*1e3);
    ROS_WARN("traj Total Duration: %lf s", traj.getTotalDuration());

    publishTrajToNav(traj);

}

void goal_callback( const geometry_msgs::PoseStampedConstPtr &p ){
    globGoal.x() = p->pose.position.x;
    globGoal.y() = p->pose.position.y;
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(p->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    globGoal.z() = yaw;

    while(isSet  == 0);


    vector<Vector3d> wp;
    Vector3d cur3d(curOdom.x(), curOdom.y(), 0);

    delete astar;
    astar = new AStar(gMap);

    Vector2d cur2d(curOdom.x(), curOdom.y());
    Vector2d goal2d(globGoal.x(), globGoal.y());

    nav_msgs::Path ph = astar->getAStarPath(cur2d, goal2d);
    if (ph.poses.size() != 0){
        astar_path_pub.publish(ph);
        wp = astar->getWayPoints();
        visualAStarWayPoint(astar_wp_pub,wp);
        generate_Traj(wp);
        sTime = ros::Time::now();
        isSet = true;
        errXSum = errYSum = errYawSum = 0;
        if(isSet == 1) isSet ++;

                gMap->pubMsg(map_pub);
    }



}

void odom_callback( const nav_msgs::OdometryConstPtr &odom ){
    curOdom.x() = odom->pose.pose.position.x;
    curOdom.y() = odom->pose.pose.position.y;
    curOdom.z() = odom->pose.pose.position.z;

    curVel.x() = odom->twist.twist.linear.x;
    curVel.y() = odom->twist.twist.linear.y;
    curVel.z() = odom->twist.twist.angular.z;

}

void map_callback( const nav_msgs::OccupancyGridConstPtr &msg ){
    delete gMap;
    gMap = new GridMap_t( msg->info.width,
                          msg->info.height,
                          msg->info.resolution,
                          msg->info.origin.position.x,
                          msg->info.origin.position.y,
                          msg->data,
                          I_RECTANGLE );
    ros::Time t1 = ros::Time::now();
    gMap->inflate(cfg.inflate_r);
    ros::Time t2 = ros::Time::now();
    ROS_WARN("Finish Inflate Map with time: %lf ms", (double)((t2-t1).toSec())*1e3);
    if(isSet == 0) isSet ++;
}



void vel_callback( const geometry_msgs::TwistConstPtr & tw ){
    joyTwist.linear.x = tw->linear.x;
    joyTwist.linear.y = tw->linear.y;
    joyTwist.angular.z = tw->angular.z;
    limitRange(joyTwist.linear.x, -cfg.safe_maxVel, cfg.safe_maxVel);
    limitRange(joyTwist.linear.y, -cfg.safe_maxVel, cfg.safe_maxVel);
    limitRange(joyTwist.angular.z, -cfg.safe_maxOmega, cfg.safe_maxOmega);

}

void state_callback( const std_msgs::UInt8ConstPtr &data ){
    switch (data->data) {
        case baseStop:
            if(curState != baseStop){
                std_msgs::UInt8 u8Msg;
                u8Msg.data = 0x02;
                base_cmd_pub.publish(u8Msg);
                joyTwist.linear.x = joyTwist.linear.y = joyTwist.angular.z = 0;
            }
            curState = baseStop;
            break;
        case baseMode1:
            if(curState == baseZero) break;
            if(curState != baseMode1){
                std_msgs::UInt8 u8Msg;
                u8Msg.data = 0;
                base_fdb_pub.publish(u8Msg);

                goal_pub.publish(frontJianJia);
                joyTwist.linear.x = joyTwist.linear.y = joyTwist.angular.z = 0;
                ROS_WARN("[MOVE TO] Front JianJia");
            }
            curState = baseMode1;
            isPubReachGoal = false;
            break;
        case baseMode2:
            if(curState == baseZero) break;
            if(curState != baseMode2){
                std_msgs::UInt8 u8Msg;
                u8Msg.data = 0;
                base_fdb_pub.publish(u8Msg);

                goal_pub.publish(backJianJia);
                joyTwist.linear.x = joyTwist.linear.y = joyTwist.angular.z = 0;
                ROS_WARN("[MOVE TO] Back JianJia");
            }
            curState = baseMode2;
            isPubReachGoal = false;
            break;
        case baseMode3:
            if(curState == baseZero) break;
            if(curState != baseMode3){
                std_msgs::UInt8 u8Msg;
                u8Msg.data = 0;
                base_fdb_pub.publish(u8Msg);

                goal_pub.publish(leaveJianJia);
                joyTwist.linear.x = joyTwist.linear.y = joyTwist.angular.z = 0;
                ROS_WARN("[MOVE TO] LEAVE JianJia");
            }
            curState = baseMode3;
            isPubReachGoal = false;
            break;
        case baseZero:
            if(curState != baseZero){
                std_msgs::UInt8 u8Msg;
                u8Msg.data = 0x01;
                base_cmd_pub.publish(u8Msg);
                joyTwist.linear.x = joyTwist.linear.y = joyTwist.angular.z = 0;
            }
            curState = baseZero;
            break;
        default:
            curState = baseNone;
            break;
    }
}

void initGoal(){
    frontJianJia.header.frame_id = "map";
    frontJianJia.pose.position.x =  -11.1005239487;
    frontJianJia.pose.position.y = 4.4920873642;
    frontJianJia.pose.position.z = 0;
    frontJianJia.pose.orientation.x = 0.0;
    frontJianJia.pose.orientation.y = 0.0;
    frontJianJia.pose.orientation.z = 0.981509364656;
    frontJianJia.pose.orientation.w = -0.191414124589
;

//    frontJianJia.header.frame_id = "map";
//    frontJianJia.pose.position.x =  -3.22920227051;
//    frontJianJia.pose.position.y = 2.67516708374;
//    frontJianJia.pose.position.z = 0;
//    frontJianJia.pose.orientation.x = 0.0;
//    frontJianJia.pose.orientation.y = 0.0;
//    frontJianJia.pose.orientation.z = -0.433014536392;
//    frontJianJia.pose.orientation.w = 0.901386937598;
//

    backJianJia.header.frame_id = "map";
    backJianJia.pose.position.x = -5.2;
    backJianJia.pose.position.y = 10.5;
    backJianJia.pose.position.z = 0;
    backJianJia.pose.orientation.x = 0.0;
    backJianJia.pose.orientation.y = 0.0;
    backJianJia.pose.orientation.z = 0.911323538952;
    backJianJia.pose.orientation.w = -0.411690912399;

    leaveJianJia.header.frame_id = "map";
    leaveJianJia.pose.position.x = -6.7;
    leaveJianJia.pose.position.y = 9.0;
    leaveJianJia.pose.position.z = 0;
    leaveJianJia.pose.orientation.x = 0.0;
    leaveJianJia.pose.orientation.y = 0.0;
    leaveJianJia.pose.orientation.z = 0.911323538952;
    leaveJianJia.pose.orientation.w = -0.411690912399;

}

void base_fdb_callback( const std_msgs::UInt8ConstPtr &msg ){
    if(msg->data == 1 && curState == baseZero) curState = baseManual;
}

void baseStatePubThreadCallback( const ros::TimerEvent &e ){
    std_msgs::UInt8 u8Msg;
    u8Msg.data = curState;
    base_state_pub.publish(u8Msg);
}

int main( int argc, char ** argv ){
    ros::init(argc, argv, "lsq_nav");


    ros::NodeHandle nh;
    cfg.set(nh);
    initGoal();

    am_traj_ptr.reset(new AmTraj);
    am_traj_ptr->init( cfg.timeWeight, cfg.accWeight, cfg.jerkWeight,
                  cfg.maxVel, cfg.maxAcc, 23,0.02);

    ROS_WARN("Navigation Start");


    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);                                 // 给webots机器人发布速度
    path_pub = nh.advertise<nav_msgs::Path>("/lsq/Path", 1);                                    // 可视化am_traj优化的path
    astar_path_pub = nh.advertise<nav_msgs::Path>("/nav/AstarPath", 1);                         // 可视化Astar规划的path
    astar_wp_pub = nh.advertise<visualization_msgs::MarkerArray >("/nav/AstarWayPoints", 1);    // 可视化Astar选取的waypoints

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);                            // 发布导航目标点
    base_fdb_pub = nh.advertise<std_msgs::UInt8>("/baseControl_feedback", 1);                   // 自动导航完成后反馈
    map_pub = nh.advertise<visualization_msgs::MarkerArray>("/nav/inflateMap", 1);              // 可视化膨胀的地图
    base_vel_pub = nh.advertise<geometry_msgs::Twist>("/base_cmd_vel", 1);                      // 给底盘发布控制速度
    base_cmd_pub = nh.advertise<std_msgs::UInt8>("/base_command", 1);                           // 给底盘发布控制指令 控制调零或者强制停止
    base_state_pub = nh.advertise<std_msgs::UInt8>("/base_cur_state", 1);

    goal_odom_pub = nh.advertise<nav_msgs::Odometry>("/nav/goalOdom", 1);

    ros::Subscriber goalSub = nh.subscribe("/goal", 10, goal_callback);                         // 设置导航目标点
    ros::Subscriber odomSub = nh.subscribe("/transform/odom", 10, odom_callback);               // 读取里程计获取当前位置
    ros::Subscriber mapSub = nh.subscribe("/map", 1, map_callback);                             // 读取地图

    ros::Subscriber velSub = nh.subscribe("/base_vel", 10, vel_callback);                       // 接收手柄速度
//    ros::Subscriber stateSub = nh.subscribe("/base_control", 10, state_callback);               // 接收控制命令 选择自动或者手动或者调零或者强制停止
    ros::Subscriber base_fdb_sub = nh.subscribe("/base_feedback", 1, base_fdb_callback);        // 接收底盘反馈 是否调零成功

    ros::Timer velCtrlThread = nh.createTimer( ros::Duration(0.01), velCtrlThread_Callback );  // 控制速度
    ros::Timer baseStatePubThread = nh.createTimer( ros::Duration(0.1), baseStatePubThreadCallback );

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;

}