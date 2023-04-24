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

std_msgs::Float32 msg;

using namespace Eigen;
using namespace std;
using namespace ros;

#define refresh -1
#define start 0                     //判断是否是第一次更新

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
vector<double> StartZone_(3);
vector<double> TargetZone_(3);
vector <double>  LivoxZone_(3);
float livox_odom_w;

//auto_decision---------------
vector<double> HitRateArray(11);
vector<int> HitPointArray(11);
//柱子最上面的圆环颜色0表示没有，1表示红,2表示蓝，需要从视觉模块通过动态参数传过来
vector<int> OverallSituation(11);
bool decisionFlag;  //是否进行决策
int OwnScore;   
int OpponentScore;  //先考虑己方得分与对方得分，如果识别够准添加对方策略判断
bool centralCondition; //中间大柱子情况

typedef enum{
    StartZoneModel = 0 ,
    LivoxZoneModel ,
}StateControl;
StateControl StartState = LivoxZoneModel;

typedef enum{
    process1 = 0 ,
    process2 ,
    process3 ,
    process4 ,
    process5 ,
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
        int auto_decision();
    private:
        ros::NodeHandle nh_ ;
        ros::Subscriber point_sub;
        ros::Subscriber joy_sub;
        
        Trajectory traj;
        std::vector<Eigen::Vector3d> wPs;
        void motion_plan_callback(const ros::TimerEvent &e);

        int ConfigProcessModel;
        //process1---------
        int point_count ;
        int pointNumber;
        int refreshTime=start;
        
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

};

TrajPlan_3D::TrajPlan_3D()
{
    // pub1 = nh_.advertise<std_msgs::Float32>("/Dipan/assembly/Empty_front_Joint/vel_cmd",10);
    // pub2 = nh_.advertise<std_msgs::Float32>("/Dipan/assembly/Empty_left_Joint/vel_cmd",10);
    // pub3 = nh_.advertise<std_msgs::Float32>("/Dipan/assembly/Empty_right_Joint/vel_cmd",10);
    msg.data = 60;
    point_count = 1;
    point_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/goal",10,&TrajPlan_3D::pointCallBack,this);
    motion_pub = nh_.advertise<geometry_msgs::Twist>("/mavros/speed_control/send_topic",10);
    sim_motion_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TrajPlan_3D::joyCallback, this);
    odom_sub = nh_.subscribe<nav_msgs::Odometry>("/wtr_robot_odom",10,&TrajPlan_3D::sim_odomCallback,this);
    map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 10, &TrajPlan_3D::map_callback,this);
    
    nh_.param("A_Zone_Button", A_Zone_Button_, A_Zone_Button_);
    nh_.param("B_Zone_Button", B_Zone_Button_, B_Zone_Button_);
    nh_.param("C_Zone_Button", C_Zone_Button_, C_Zone_Button_);
    nh_.param("hang_Button", hang_Button_, hang_Button_);

    nh_.getParam("HitRate", HitRateArray);
    nh_.getParam("HitPoint", HitPointArray);
    nh_.getParam("OverallSituation",OverallSituation);

    switch (ProcessModel)
    {
        case process1:
        {
            nh_.getParam("/traj_plan_3D/PointNumber", pointNumber);   //load number of points
            break;
        }
        case process2:
        {
            nh_.getParam("/traj_plan_3D/MaxParts",maxParts);          //load point parts array
            vector<int> TempPointNumberArray(maxParts,0);
            nh_.getParam("/traj_plan_3D/PointArray",TempPointNumberArray);
            for(int i=0;i<maxParts;i++)
            {
                PointNumberArray[i] = TempPointNumberArray[i];
                ROS_INFO("PointNumberArray[%d]=%d",i,PointNumberArray[i]);
            }
            break; 
        }
        case process3:
        {
            nh_.getParam("/traj_plan_3D/MaxPointSetNumber", maxPointSetNumber);
            vector<float> TempCoordinatePointSet(maxPointSetNumber,0);
            nh_.getParam("/traj_plan_3D/PointSet", TempCoordinatePointSet);
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
            nh_.getParam("/traj_plan_3D/MaxParts_",maxParts_);          //区间的段数4
            vector<int> TempPointArray_(maxParts_,0);                   //加载设置每段点数的向量 [3,3,4,4]
            nh_.getParam("/traj_plan_3D/PointArray_",TempPointArray_);  
            for(int i=0;i<maxParts_;i++)
            {
                PointNumberArray_[i] = TempPointArray_[i];
                ROS_INFO("PointNumberArray[%d]=%d",i,PointNumberArray_[i]);
            }
            nh_.getParam("/traj_plan_3D/MaxPointSetNumber_", maxPointSetNumber_);   //加载所有元素的总数，即坐标数*3  42
            vector<float> TempCoordinatePointSet_(maxPointSetNumber_,0); 
            nh_.getParam("/traj_plan_3D/PointSet_",TempCoordinatePointSet_);     //一维数组加载所有的点  42个元素组成的数组
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
            nh_.getParam("/traj_plan_3D/FireZoneA",FireZoneA_);
            nh_.getParam("/traj_plan_3D/FireZoneB",FireZoneB_);
            nh_.getParam("/traj_plan_3D/FireZoneC",FireZoneC_);
            nh_.getParam("/traj_plan_3D/StartZone",StartZone_);
            protectFlag=true;
            //plus----------------addLivoxOdom----------------------------
            nh_.getParam("fastlio_mapping/livox_odom_x",LivoxZone_[0]);
            nh_.getParam("fastlio_mapping/livox_odom_y",LivoxZone_[1]);
            nh_.getParam("fastlio_mapping/livox_odom_z",LivoxZone_[2]);
            nh_.getParam("fastlio_mapping/livox_odom_w",livox_odom_w);
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
    Visualizer viz(config, nh_);
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
                if(refreshTime != start){i=0;}
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

                if(refreshTime != start){i=0;}
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
                    if(IntervalNumber_ != start){popTemp=0;}
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
                ROS_INFO("joy start mdoel");
                if(StartState == StartZoneModel)
                {
                    wPs.emplace_back(StartZone_[0],StartZone_[1],StartZone_[2]);
                }
                else if(StartState == LivoxZoneModel)
                {
                    wPs.emplace_back(LivoxZone_[0],LivoxZone_[1],LivoxZone_[2]);
                }
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
                else if((TargetZone_[0] == StartZone_[0])&&(StartZone_[1] == StartZone_[1]))
                {
                    protectFlag=false;
                }
                else{
                    protectFlag=true;
                }
                // std::cout<<"Start point x = "<<StartZone_[0]<<"y = "<<StartZone_[1]<<"z = "<<StartZone_[2]<<std::endl;
                std::cout<<"Start point x = "<<LivoxZone_[0]<<"y = "<<LivoxZone_[1]<<"z = "<<LivoxZone_[2]<<std::endl;
                std::cout<<"Final target x = "<<TargetZone_[0]<<"y = "<<TargetZone_[1]<<"z = "<<TargetZone_[2]<<std::endl;
                if(protectFlag==true)
                {
                    wPs.emplace_back(TargetZone_[0],TargetZone_[1],TargetZone_[2]);
                    if(StartState == StartZoneModel)
                    {
                        wPs = makePlan(StartZone_,TargetZone_);
                    }
                    else if(StartState == LivoxZoneModel)
                    {
                        wPs = makePlan(LivoxZone_,TargetZone_);
                    }
                    gogogo();
                    for(int popTemp = 0;popTemp<wPs.size();popTemp++)
                    {
                        std::vector<Eigen::Vector3d>::iterator k = wPs.begin();
                        wPs.erase(k);//删除第一个元素
                    }
                    StartZone_[0] = TargetZone_[0];
                    StartZone_[1] = TargetZone_[1];
                    StartZone_[2] = TargetZone_[2];
                    protectFlag=true;
                }
                else{
                    ROS_WARN("You can not set the same TargetZone !");
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

int TrajPlan_3D::auto_decision()
{
    if(decisionFlag)
    {

    }
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "traj_node");
    
    TrajPlan_3D tp_3D;
    ROS_INFO("Traj_plan start, point number");
    ros::spin();
    
    return 0;
}