#include "wtr_race/wtr_am_traj.hpp"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "astar/astar.h"

// #include "livox_lidar/livox_lidar.h"

std_msgs::Float32 msg;

using namespace Eigen;
using namespace std;
using namespace ros;

#define refresh -1
#define start_ 0                     //判断是否是第一次更新

//process2---------
#define maxPointParts 20            
vector<int> PointNumberArray(maxPointParts);

//process3---------
#define maxSetNumber 30*3
vector<vector<double>>    CoordinatePointSet(maxSetNumber,vector<double>(3,0));

//process4---------
#define maxPointParts_  20          //最大设置20段
#define maxSetNumber_ 50*3          //最多设置50个坐标点
vector<int> PointNumberArray_(maxPointParts_);
vector< vector < vector<double> > > CoordinatePointSet_(maxPointParts_,vector< vector<double> >(maxSetNumber_,vector<double>(3,0)));

//process5---------
vector<double> FireZoneA_(3);
vector<double> FireZoneB_(3);
vector<double> FireZoneC_(3);
vector<double> StartFireZone_(3);   //为ABC三个区中的一个，是取环之后到达的第一个射环区
vector<double> StartMoveZone_(3);   //车的移动起点
vector<double> PickupZoneA_(3);
vector<double> PickupZoneB_(3);
vector<double> TargetZone_(3);
vector <double>  LivoxZone_(3);
float livox_odom_w;

//auto_decision---------------
#define PillarNumber  11
#define ZoneNumber    3
vector<double> HitRateArray(PillarNumber);                            //命中率
vector<int> HitPointArray(PillarNumber);                              //得分
vector<float> PointRateArray(PillarNumber);                           //得分率
vector<float> AdjustTime_A_(PillarNumber);                            //调整时间
vector<float> AdjustTime_B_(PillarNumber);
vector<float> AdjustTime_C_(PillarNumber);
vector<float> ZoneTransTime_(ZoneNumber*(ZoneNumber-1));                //转移时间
vector<vector<float>> CompletePointTimeRate_(ZoneNumber*(ZoneNumber-1),vector<float>(PillarNumber,0));   
                                                    //该柱子的得分率比调整+转移的完成时间(从ABC出发->到达调整后发射的柱子)
vector<int> OverallSituation(PillarNumber);                           //最上方环的状态

int OwnScore;   
int OpponentScore;  //先考虑己方得分与对方得分，如果识别够准添加对方策略判断
bool centralCondition; //中间大柱子情况
int StartZone_Index;
int TargetPillar_Index;
int TargetZone_Index;
vector<float> chasis2map_dxyz(3);
bool decisionFlag = false;  //是否进行决策
bool FirstDecision = false;
bool FirstMove = true;
bool SecondMove = false;
bool PickupAgain = false;
int traj_time_count = 0;

typedef enum{
    StartZoneModel = 0 ,//射环区域之间转移
    LivoxZoneModel //任意一点转移到射环区
}StateControl;
StateControl StartState = StartZoneModel;

typedef enum{
    process1 = 0 ,
    process2 ,
    process3 ,
    process4 ,
    process5 
}debugProcessModel;
debugProcessModel ProcessModel = process5;

class Config
{
    public:
        Config(const ros::NodeHandle &nh_priv)
        {
            nh_priv.getParam("TrajectoryTopic", trajectoryTopic);
            nh_priv.getParam("WayPointsTopic", wayPointsTopic);
            nh_priv.getParam("RouteTopic", routeTopic);
            nh_priv.getParam("AccTopic", accTopic);
            nh_priv.getParam("FrameName", frameName);
            nh_priv.getParam("WeightT", weightT);
            nh_priv.getParam("WeightAcc", weightAcc);
            nh_priv.getParam("WeightJerk", weightJerk);
            nh_priv.getParam("MaxAccRate", maxAccRate);
            nh_priv.getParam("MaxVelRate", maxVelRate);
            nh_priv.getParam("Iterations", iterations);
            nh_priv.getParam("Epsilon", epsilon);
        }
    // Advertised Topics
    string trajectoryTopic;
    string wayPointsTopic;
    string routeTopic;
    string accTopic;
    // Frame Name
    std::string frameName;
    // Params
    double weightT;
    double weightAcc;
    double weightJerk;
    double maxAccRate;
    double maxVelRate;
    int iterations;
    double epsilon;

};

class Visualizer                //可视化工具   Marker标记
{
public:
    Visualizer(const Config &conf, ros::NodeHandle &nh) : config(conf)
    {
        trajectoryPub = nh.advertise<visualization_msgs::Marker>(config.trajectoryTopic, 1);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>(config.wayPointsTopic, 1);
        routePub = nh.advertise<visualization_msgs::Marker>(config.routeTopic, 1);
        accPub = nh.advertise<visualization_msgs::MarkerArray>(config.accTopic, 1);
    }

private:
    Config config;                              //加载配置的参数
    ros::Publisher routePub;                    //发布轨迹
    ros::Publisher wayPointsPub;                //发布鼠标点下的坐标点
    ros::Publisher trajectoryPub;               //发布弹道
    ros::Publisher accPub;                      //发布加速度

public:
    void visualize(const Trajectory &traj, const vector<Vector3d> &route, int id = 0)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker, accMarker;
        visualization_msgs::MarkerArray accMarkers;

        routeMarker.id = id;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = config.frameName;
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.05;

        wayPointsMarker = routeMarker;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 1.00;
        wayPointsMarker.color.b = 1.00;
        wayPointsMarker.scale.x = 0.30;
        wayPointsMarker.scale.y = 0.30;
        wayPointsMarker.scale.z = 0.30;

        trajMarker = routeMarker;
        trajMarker.ns = "trajectory";
        trajMarker.scale.x = 0.15;
        if (id == 0)
        {
            trajMarker.color.r = 1.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 0.00;
        }
        else if (id == 1)
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 1.00;
            trajMarker.color.b = 0.00;
        }
        else
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 1.00;
        }

        accMarker = routeMarker;
        accMarker.type = visualization_msgs::Marker::ARROW;
        accMarker.header.stamp = ros::Time::now();
        accMarker.ns = "acc";
        if (id == 0)
        {
            accMarker.color.r = 255.0 / 255.0;
            accMarker.color.g = 20.0 / 255.0;
            accMarker.color.b = 147.0 / 255.0;
        }
        else if (id == 1)
        {
            accMarker.color.r = 60.0 / 255.0;
            accMarker.color.g = 179.0 / 255.0;
            accMarker.color.b = 113.0 / 255.0;
        }
        else
        {
            accMarker.color.r = 30.0 / 255.0;
            accMarker.color.g = 144.0 / 255.0;
            accMarker.color.b = 255.0 / 255.0;
        }
        accMarker.scale.x = 0.05;
        accMarker.scale.y = 0.15;
        accMarker.scale.z = 0.30;

        if (route.size() > 0)
        {
            bool first = true;
            Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;

                wayPointsMarker.points.push_back(point);
            }

            routePub.publish(routeMarker);
        }

        if (route.size() > 0)
        {
            for (auto it : route)
            {
                geometry_msgs::Point point;
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                wayPointsMarker.points.push_back(point);
            }
            wayPointsPub.publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub.publish(trajMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            if (id == 0)
            {
                accMarker.action = visualization_msgs::Marker::DELETEALL;
                accMarkers.markers.push_back(accMarker);
                accPub.publish(accMarkers);
                accMarkers.markers.clear();
                accMarker.action = visualization_msgs::Marker::ADD;
            }

            double T = 0.08;
            for (double t = 0; t < traj.getTotalDuration(); t += T)
            {
                accMarker.id += 3;
                accMarker.points.clear();
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                X += traj.getAcc(t);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                accMarkers.markers.push_back(accMarker);
            }
            accPub.publish(accMarkers);
        }
    }
};

class TrajPlan_3D : public Astar_planner::AstarPlannerROS
{
    public:
        TrajPlan_3D();
        void pointCallBack(const geometry_msgs::PoseStamped::ConstPtr &point_msg);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void sim_odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
        void gogogo();
        void auto_decision(const ros::TimerEvent&);
        void pillar_detect();
        void get_target();
    private:
        ros::NodeHandle traj_nh_ ;
        ros::Subscriber point_sub;
        ros::Subscriber joy_sub;
        
        Trajectory traj;
        std::vector<Eigen::Vector3d> wPs;
        // void motion_plan_callback(const ros::TimerEvent &e);

        int ConfigProcessModel;
        //process1---------
        int point_count ;
        int pointNumber;
        int refreshTime=start_;
        
        //process2---------
        int maxParts;
        int IntervalNumber = 0;

        //process3---------
        int maxPointSetNumber;

        //process4---------
        int maxParts_;
        int IntervalNumber_ = 0;
        int maxPointSetNumber_;

        //process5---------
        int A_Zone_Button_, B_Zone_Button_, C_Zone_Button_, hang_Button_;
        bool protectFlag;

        //mavros simulator communication-----------
        ros::Publisher motion_pub;  
        ros::Publisher sim_motion_pub;
            // mavros_msgs::Speed motion_msg;
        geometry_msgs::Twist motion_msg;

        ros::Subscriber odom_sub; 
        tf::TransformBroadcaster odom_link_broadcaster; 
        tf::TransformBroadcaster odom_world_broadcaster; 
        tf2_ros::Buffer buffer; 

        //grid_map---------
        ros::Subscriber map_sub;

        ros::Timer timer;
        ros::Time start_time;

        float lidar_decay_time;

};

TrajPlan_3D::TrajPlan_3D()
{
    // ros::NodeHandle my_livox_nh("~");
    // LivoxDetect livoxdetect(my_livox_nh);
    // pub1 = nh_.advertise<std_msgs::Float32>("/Dipan/assembly/Empty_front_Joint/vel_cmd",10);
    // pub2 = nh_.advertise<std_msgs::Float32>("/Dipan/assembly/Empty_left_Joint/vel_cmd",10);
    // pub3 = nh_.advertise<std_msgs::Float32>("/Dipan/assembly/Empty_right_Joint/vel_cmd",10);
    start_time = ros::Time::now();
    msg.data = 60;
    point_count = 1;
    point_sub = traj_nh_.subscribe<geometry_msgs::PoseStamped>("/goal",10,&TrajPlan_3D::pointCallBack,this);
    motion_pub = traj_nh_.advertise<geometry_msgs::Twist>("/mavros/speed_control/send_topic",10);
    sim_motion_pub = traj_nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    joy_sub = traj_nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TrajPlan_3D::joyCallback, this);
    odom_sub = traj_nh_.subscribe<nav_msgs::Odometry>("/wtr_robot_odom",10,&TrajPlan_3D::sim_odomCallback,this);
    map_sub = traj_nh_.subscribe<nav_msgs::OccupancyGrid>("map", 10, &TrajPlan_3D::map_callback,this);
    timer = traj_nh_.createTimer(ros::Duration(1), &TrajPlan_3D::auto_decision,this);
    traj_nh_.getParam("traj_plan_3D/auto_decesion/LidarPart/LidarDecayTime",lidar_decay_time);
    traj_nh_.getParam("traj_plan_3D/auto_decesion/Chasis/ChasisDxyz",chasis2map_dxyz);

    traj_nh_.param("A_Zone_Button", A_Zone_Button_, A_Zone_Button_);
    traj_nh_.param("B_Zone_Button", B_Zone_Button_, B_Zone_Button_);
    traj_nh_.param("C_Zone_Button", C_Zone_Button_, C_Zone_Button_);
    traj_nh_.param("hang_Button", hang_Button_, hang_Button_);

    traj_nh_.getParam("traj_plan_3D/auto_decesion/ZoneA/HitRate", HitRateArray);
    traj_nh_.getParam("traj_plan_3D/auto_decesion/HitPoint", HitPointArray);
    for(int i=0;i<PillarNumber;i++)
    {
        PointRateArray[i] = HitRateArray[i]*HitPointArray[i];
    }
    traj_nh_.getParam("traj_plan_3D/auto_decesion/Time/AdjustTime/ZoneA",AdjustTime_A_);
    traj_nh_.getParam("traj_plan_3D/auto_decesion/Time/AdjustTime/ZoneA",AdjustTime_B_);
    traj_nh_.getParam("traj_plan_3D/auto_decesion/Time/AdjustTime/ZoneA",AdjustTime_C_);
    traj_nh_.getParam("traj_plan_3D/auto_decesion/Time/TransTime",ZoneTransTime_);
    for(int i=0;i<ZoneNumber*(ZoneNumber-1);i++)
    {
        for(int j=0;j<PillarNumber;j++)
        {
            if(i<(ZoneNumber-1)){CompletePointTimeRate_[i][j] = PointRateArray[j]/(ZoneTransTime_[i]+AdjustTime_A_[j]);}//在A区出发
            else if(i<2*(ZoneNumber-1)){CompletePointTimeRate_[i][j] = PointRateArray[j]/(ZoneTransTime_[i]+AdjustTime_B_[j]);}//在B区出发
            else if(i<3*(ZoneNumber-1)){CompletePointTimeRate_[i][j] = PointRateArray[j]/(ZoneTransTime_[i]+AdjustTime_C_[j]);}//在C区出发
        }
    }

    switch (ProcessModel)
    {
        case process1:
        {
            traj_nh_.getParam("/traj_plan_3D/PointNumber", pointNumber);   //load number of points
            break;
        }
        case process2:
        {
            traj_nh_.getParam("/traj_plan_3D/MaxParts",maxParts);          //load point parts array
            vector<int> TempPointNumberArray(maxParts,0);
            traj_nh_.getParam("/traj_plan_3D/PointArray",TempPointNumberArray);
            for(int i=0;i<maxParts;i++)
            {
                PointNumberArray[i] = TempPointNumberArray[i];
                ROS_INFO("PointNumberArray[%d]=%d",i,PointNumberArray[i]);
            }
            break; 
        }
        case process3:
        {
            traj_nh_.getParam("/traj_plan_3D/MaxPointSetNumber", maxPointSetNumber);
            vector<float> TempCoordinatePointSet(maxPointSetNumber,0);
            traj_nh_.getParam("/traj_plan_3D/PointSet", TempCoordinatePointSet);
            for(int i=0;i<maxPointSetNumber/3;i++)
            {
                for(int j=0;j<3;j++)
                {
                    CoordinatePointSet[i][j] = TempCoordinatePointSet[i*3+j];
                    ROS_INFO("CoordinatePointSet[%d][%d]=%f",i,j,TempCoordinatePointSet[i*3+j]);
                    if((i*3+j+1)%3==0)
                    {
                        std::cout<<"  "<<std::endl;
                    }
                }
            }
            break; 
        }
        case process4:
        {
            traj_nh_.getParam("/traj_plan_3D/MaxParts_",maxParts_);          //区间的段数4
            vector<int> TempPointArray_(maxParts_,0);                   //加载设置每段点数的向量 [3,3,4,4]
            traj_nh_.getParam("/traj_plan_3D/PointArray_",TempPointArray_);  
            for(int i=0;i<maxParts_;i++)
            {
                PointNumberArray_[i] = TempPointArray_[i];
                ROS_INFO("PointNumberArray[%d]=%d",i,PointNumberArray_[i]);
            }
            traj_nh_.getParam("/traj_plan_3D/MaxPointSetNumber_", maxPointSetNumber_);   //加载所有元素的总数，即坐标数*3  42
            vector<float> TempCoordinatePointSet_(maxPointSetNumber_,0); 
            traj_nh_.getParam("/traj_plan_3D/PointSet_",TempCoordinatePointSet_);     //一维数组加载所有的点  42个元素组成的数组
            int base_count = 3*TempPointArray_[0];
            for(int i=0;i<maxParts_;i++)
            {
                for(int j=0;j<TempPointArray_[i];j++)
                {   
                    for(int k=0;k<3;k++)
                    {
                        CoordinatePointSet_[i][j][k] = TempCoordinatePointSet_[-1*TempPointArray_[0]*3+base_count+j*3+k];
                        ROS_INFO("CoordinatePointSet_[%d][%d][%d]=%f",i,j,k,CoordinatePointSet_[i][j][k]);
                    }
                }
                base_count += 3*TempPointArray_[i];
                std::cout<<" "<<std::endl;
            }
            break; 
        }
        case process5:
        {
            traj_nh_.getParam("/traj_plan_3D/FireZoneA",FireZoneA_);
            traj_nh_.getParam("/traj_plan_3D/FireZoneB",FireZoneB_);
            traj_nh_.getParam("/traj_plan_3D/FireZoneC",FireZoneC_);
            traj_nh_.getParam("/traj_plan_3D/StartFireZone",StartFireZone_);
            traj_nh_.getParam("/traj_plan_3D/StartMoveZone",StartMoveZone_);
            traj_nh_.getParam("/traj_plan_3D/PichUpZoneA",PickupZoneA_);
            traj_nh_.getParam("/traj_plan_3D/PickUpZoneB",PickupZoneB_);
            protectFlag=true;
            for(int i=0;i<3;i++)
            {
                FireZoneA_[i] = FireZoneA_[i]-chasis2map_dxyz[i];
                FireZoneB_[i] = FireZoneB_[i]-chasis2map_dxyz[i];
                FireZoneC_[i] = FireZoneC_[i]-chasis2map_dxyz[i];
                PickupZoneA_[i] = PickupZoneA_[i]-chasis2map_dxyz[i];
                PickupZoneB_[i] = PickupZoneB_[i]-chasis2map_dxyz[i];
                StartFireZone_[i] = StartFireZone_[i]-chasis2map_dxyz[i];
                StartMoveZone_[i] = StartMoveZone_[i]-chasis2map_dxyz[i];
            }
            break;     
        }
        default:
            break;
    }
}

void TrajPlan_3D::gogogo()
{
    ros::NodeHandle nh_priv("~");
    Config config(nh_priv);
    Visualizer viz(config, traj_nh_);
    ros::Rate rate(10);
    Eigen::Vector3d iV(-0.015, -0.01, 0.0), fV(0.0, 0.0, 0.0);
    Eigen::Vector3d iA(0.0, 0.0, 0.0), fA(0.0, 0.0, 0.0); //规定航点处的速度和加速度
    AmTraj amTrajOpt(config.weightT, config.weightAcc, config.weightJerk,config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon);
    
    traj = amTrajOpt.genOptimalTrajDTC(wPs, iV, iA, fV, fA);
    ROS_INFO("Draw trail start");
    ros::Time begin = ros::Time::now();
    while (ros::ok())
    {                
        viz.visualize(traj, wPs, 0);
        ros::Duration time_diff = ros::Time::now() - begin;
        //实际x为0,y为1
        motion_msg.linear.x = traj.getVel(time_diff.toSec())(0);
        motion_msg.linear.y = traj.getVel(time_diff.toSec())(1);
        if(time_diff.toSec()>traj.getTotalDuration() && time_diff.toSec()< traj.getTotalDuration()+0.15)
        {
            motion_msg.linear.x = 0;
            motion_msg.linear.y = 0;
            motion_msg.angular.z = 0;
        }
        else if(time_diff.toSec()>traj.getTotalDuration() && time_diff.toSec()> traj.getTotalDuration()+0.15)
        {
            motion_msg.linear.x = 0;
            motion_msg.linear.y = 0;
            motion_msg.angular.z = 0;
            ROS_WARN("Stop!!!!");
            point_count = refresh;
            break;
        }
        motion_pub.publish(motion_msg);
        ROS_INFO("time = %f,vx = %f,vy = %f",time_diff.toSec(), motion_msg.linear.x, motion_msg.linear.y);
        rate.sleep();
    }
}

void TrajPlan_3D::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    if(ProcessModel == process5)
    {
        initialize(map_msg);
    }
}    

void TrajPlan_3D::sim_odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::TransformStamped odom_world_trans;
    odom_world_trans.header.stamp = current_time;
    odom_world_trans.header.frame_id = "world";
    odom_world_trans.child_frame_id = "base_footprint";
    odom_world_trans.transform.translation.x = odom->pose.pose.position.x;
    odom_world_trans.transform.translation.y = odom->pose.pose.position.y;
    odom_world_trans.transform.translation.z = 0;
    odom_world_trans.transform.rotation = odom->pose.pose.orientation;
    odom_world_broadcaster.sendTransform(odom_world_trans);
}

void TrajPlan_3D::pointCallBack(const geometry_msgs::PoseStamped::ConstPtr &point_msg)
{
    float x = point_msg->pose.position.x;
    float y = point_msg->pose.position.y;
    float z = point_msg->pose.position.z;

    switch (ProcessModel)
    {
        case process1:
        {
            if(point_count== refresh)
            {
                int i;
                if(refreshTime != start_){i=0;}
                else{i=1;refreshTime=-1;}
                for(;i<pointNumber;i++)
                {
                    std::vector<Eigen::Vector3d>::iterator k = wPs.begin();
                    wPs.erase(k);//删除第一个元素
                }
                point_count = 1;
            }
            if(point_count<=pointNumber)
            {
                wPs.emplace_back(x,y,z);
                point_count ++;
            }
            std::cout<<"Receive x = "<<x<<"y = "<<y<<"z = "<<z<<std::endl;
            std::cout<<"count = "<<point_count<<std::endl;
            if(point_count==pointNumber+1)
            {
                gogogo();
            }
            break;
        }
        case process2:
        {
           if(point_count== refresh)
            {
                int i;

                if(refreshTime != start_){i=0;}
                else{i=1;refreshTime=-1;}
                for(;i<PointNumberArray[IntervalNumber-1];i++)
                {
                    std::vector<Eigen::Vector3d>::iterator k = wPs.begin();
                    wPs.erase(k);//删除第一个元素
                }
                point_count = 1;
            }
            if(point_count<=PointNumberArray[IntervalNumber])
            {
                wPs.emplace_back(x,y,z);
                point_count ++;
                ROS_INFO("PointNumberArray[%d]=%d",IntervalNumber,PointNumberArray[IntervalNumber]);
            }
            std::cout<<"Receive x = "<<x<<"y = "<<y<<"z = "<<z<<std::endl;
            std::cout<<"count = "<<point_count<<std::endl;
            if(point_count==PointNumberArray[IntervalNumber]+1)
            {
                if(IntervalNumber<maxParts){IntervalNumber++;}
                else{ROS_WARN("too many parts of points.");}
                gogogo();
            }
            break;
        }
        default:
            break;
    }
}

void TrajPlan_3D::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    switch (ProcessModel)
    {
        case process3:
        {
            if(joy->buttons[A_Zone_Button_]==1)
            {
                ROS_INFO("joy start model");
                for(int i=0;i<maxPointSetNumber/3;i++)
                {
                    float x = CoordinatePointSet[i][0];
                    float y = CoordinatePointSet[i][1];
                    float z = CoordinatePointSet[i][2];
                    wPs.emplace_back(x,y,z);
                    std::cout<<"Add x = "<<x<<"y = "<<y<<"z = "<<z<<std::endl;
                }
                gogogo();
            }
            else if(joy->buttons[hang_Button_]==1&&joy->buttons[A_Zone_Button_]==0)
            {
                ROS_INFO("joy stop mdoel");
            } 
            break;
        }
        case process4:
        {
            if(joy->buttons[A_Zone_Button_]==1)
            {
                ROS_INFO("joy start model");
                if(IntervalNumber_>=maxParts_)
                {
                    ROS_WARN("too many parts than set");
                }
                else{
                    for(int i=0;i<PointNumberArray_[IntervalNumber_];i++)
                    {
                        float x = CoordinatePointSet_[IntervalNumber_][i][0];
                        float y = CoordinatePointSet_[IntervalNumber_][i][1];
                        float z = CoordinatePointSet_[IntervalNumber_][i][2];
                        wPs.emplace_back(x,y,z);
                        std::cout<<"Add x = "<<x<<"y = "<<y<<"z = "<<z<<std::endl;
                    }
                    gogogo();
                    int popTemp;
                    if(IntervalNumber_ != start_){popTemp=0;}
                    else{popTemp=1;}
                    for(;popTemp<PointNumberArray_[IntervalNumber_];popTemp++)
                    {
                        std::vector<Eigen::Vector3d>::iterator k = wPs.begin();
                        wPs.erase(k);//删除第一个元素
                    }
                }
                IntervalNumber_++;
            }
            else if(joy->buttons[hang_Button_]==1&&joy->buttons[A_Zone_Button_]==0)
            {
                ROS_INFO("joy stop mdoel");
            }
            break;
        }
          
        case process5:
        {
            if(joy->buttons[A_Zone_Button_]==1||joy->buttons[B_Zone_Button_]==1||joy->buttons[C_Zone_Button_]==1)
            {
                if(FirstMove==true)
                {
                    ROS_INFO("Move to pickup zoneA !");
                    if(StartState == StartZoneModel)
                    {
                        wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                        wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                        wPs = makePlan(StartMoveZone_,PickupZoneA_);
                    }
                    else if(StartState == LivoxZoneModel)
                    {
                        //plus----------------addLivoxOdom----------------------------
                        traj_nh_.getParam("/fastlio_mapping/livox_odom_x",LivoxZone_[0]);
                        traj_nh_.getParam("/fastlio_mapping/livox_odom_y",LivoxZone_[1]);
                        traj_nh_.getParam("/fastlio_mapping/livox_odom_z",LivoxZone_[2]);
                        traj_nh_.getParam("/fastlio_mapping/livox_odom_w",livox_odom_w);
                        wPs.emplace_back(LivoxZone_[0],LivoxZone_[1],LivoxZone_[2]);
                        wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                        wPs = makePlan(LivoxZone_,LivoxZone_);
                    } 
                    gogogo();
                    for(int popTemp = 0;popTemp<wPs.size();popTemp++)
                    {
                        std::vector<Eigen::Vector3d>::iterator k = wPs.begin();
                        wPs.erase(k);//删除第一个元素
                    }
                    StartMoveZone_[0] = PickupZoneA_[0];
                    StartMoveZone_[1] = PickupZoneA_[1];
                    StartMoveZone_[2] = PickupZoneA_[2];
                    FirstMove = false;
                    SecondMove = true;
                }
                else if(FirstMove==false&&PickupAgain==true)
                {
                    ROS_INFO("Move to pickup zoneB !");
                    wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                    wPs.emplace_back(PickupZoneB_[0],PickupZoneB_[1],PickupZoneB_[2]);
                    wPs = makePlan(StartMoveZone_,PickupZoneB_);
                    gogogo();
                    for(int popTemp = 0;popTemp<wPs.size();popTemp++)
                    {
                        std::vector<Eigen::Vector3d>::iterator k = wPs.begin();
                        wPs.erase(k);//删除第一个元素
                    }
                    StartMoveZone_[0] = PickupZoneB_[0];
                    StartMoveZone_[1] = PickupZoneB_[1];
                    StartMoveZone_[2] = PickupZoneB_[2];
                    PickupAgain=false;
                    // traj_time_count++;
                }
                else if(FirstMove==false&&PickupAgain==false)
                {
                    traj_time_count++;
                    if(traj_time_count==10)
                    {
                        PickupAgain = true;
                    }
                    else{PickupAgain = false;}
                    ROS_INFO("joy start fire model");
                    if(StartState == StartZoneModel)
                    {
                        wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                    }
                    // else if(StartState == LivoxZoneModel)
                    // {
                    //     wPs.emplace_back(LivoxZone_[0],LivoxZone_[1],LivoxZone_[2]);
                    // }
                    if(joy->buttons[A_Zone_Button_]==1)
                    {
                        ROS_INFO("Move to firezoneA !");
                        TargetZone_[0] = FireZoneA_[0];
                        TargetZone_[1] = FireZoneA_[1];
                        TargetZone_[2] = FireZoneA_[2];
                    }
                    else if(joy->buttons[B_Zone_Button_]==1)
                    {
                        ROS_INFO("Move to firezoneB !");
                        TargetZone_[0] = FireZoneB_[0];
                        TargetZone_[1] = FireZoneB_[1];
                        TargetZone_[2] = FireZoneB_[2];
                    }
                    else if(joy->buttons[C_Zone_Button_]==1)
                    {
                        ROS_INFO("Move to firezoneC !");
                        TargetZone_[0] = FireZoneC_[0];
                        TargetZone_[1] = FireZoneC_[1];
                        TargetZone_[2] = FireZoneC_[2];
                    }
                    if((TargetZone_[0] == LivoxZone_[0])&&(TargetZone_[1] == LivoxZone_[1]))
                    {
                        protectFlag=false;
                    }
                    else if((TargetZone_[0] == StartMoveZone_[0])&&(TargetZone_[1] == StartMoveZone_[1]))
                    {
                        protectFlag=false;
                    }
                    else{
                        protectFlag=true;
                    }
                    std::cout<<"Start point x = "<<StartMoveZone_[0]<<"y = "<<StartMoveZone_[1]<<"z = "<<StartMoveZone_[2]<<std::endl;
                    // std::cout<<"Start point x = "<<LivoxZone_[0]<<"y = "<<LivoxZone_[1]<<"z = "<<LivoxZone_[2]<<std::endl;
                    std::cout<<"Final target x = "<<TargetZone_[0]<<"y = "<<TargetZone_[1]<<"z = "<<TargetZone_[2]<<std::endl;
                    if(protectFlag==true)
                    {
                        wPs.emplace_back(TargetZone_[0],TargetZone_[1],TargetZone_[2]);
                        if(StartState == StartZoneModel)
                        {
                            wPs = makePlan(StartMoveZone_,TargetZone_);
                        }
                        // else if(StartState == LivoxZoneModel)
                        // {
                        //     wPs = makePlan(LivoxZone_,TargetZone_);
                        // }
                        gogogo();
                        for(int popTemp = 0;popTemp<wPs.size();popTemp++)
                        {
                            std::vector<Eigen::Vector3d>::iterator k = wPs.begin();
                            wPs.erase(k);//删除第一个元素
                        }
                        StartMoveZone_[0] = TargetZone_[0];
                        StartMoveZone_[1] = TargetZone_[1];
                        StartMoveZone_[2] = TargetZone_[2];
                        protectFlag=true;
                    }
                    else{
                        ROS_WARN("You can not set the same TargetZone !");
                    }
                    if(SecondMove==true)
                    {
                        FirstDecision = true;  
                        StartFireZone_ =  StartMoveZone_;
                        SecondMove = false;
                        decisionFlag=true;
                    }
                    else{FirstDecision = false; }
                }    
            }
            else if(joy->buttons[hang_Button_]==1)
            {
                ROS_INFO("joy stop mdoel");
            }
            break;
        } 
        default:
            break;
    }
}

void TrajPlan_3D::get_target()
{
    if(FirstDecision==true)
    {
        // ROS_INFO("test---------------------------------------------");
        if(StartFireZone_ == FireZoneA_){StartZone_Index = 0;}
        else if(StartFireZone_ == FireZoneB_){StartZone_Index = 1;}
        else if(StartFireZone_ == FireZoneC_){StartZone_Index = 2;}
        // std::cout<<StartZone_Index<<std::endl;
    }
    else StartZone_Index = TargetZone_Index;
    float max = CompletePointTimeRate_[0][0];
    for (int i = 0; i < ZoneNumber*(ZoneNumber-1); i++) {
        for (int j = 0; j < PillarNumber; j++) 
        {
            //当前位置不能去    (0->A->12;1->B->34;2->C->56)
            if((i!=StartZone_Index*2+1)&&(i!=StartZone_Index*2+2))
            {
                CompletePointTimeRate_[i][j] *= 0;
            }
            else{
                CompletePointTimeRate_[i][j] *= 1;
            }
            //最上面已经射中的柱子不用再射
            if(OverallSituation[j] == 1)
            {
                CompletePointTimeRate_[i][j] *= 0;
            }
            else if(OverallSituation[j] == 2)
            {
                CompletePointTimeRate_[i][j] *= 10;
            }
            else{CompletePointTimeRate_[i][j] *= 1;}
            //综合处理得到的最大索引
            if (CompletePointTimeRate_[i][j] > max) {
                max = CompletePointTimeRate_[i][j];
                TargetZone_Index = i;
                TargetPillar_Index = j;
            }
        }
    }
    FirstDecision = false;
    traj_nh_.setParam("traj_plan_3D/auto_decesion/TargetPillar",TargetPillar_Index);
}

void TrajPlan_3D::pillar_detect()
{
    traj_nh_.getParam("/decay_map_test/decay_map/OverallSituation",OverallSituation);
}

//尽快大胜
void TrajPlan_3D::auto_decision(const ros::TimerEvent&)
{
    ros::Time right_now = ros::Time::now();
    ros::Duration pass_time = right_now-start_time;
    if(decisionFlag&&pass_time.toSec()>lidar_decay_time&&decisionFlag==true)
    {
        pillar_detect();
        get_target();
        // std::cout<<"main transfer"<<std::endl;
    }
    // ROS_INFO("pass time : %f",pass_time.toSec()); 
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "traj_node");
    
    TrajPlan_3D tp_3D;
    ROS_INFO("Traj_plan start, point number");
    // ros::spin();

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    
    return 0;
}