#ifndef LIVOX_LIDAR_H
#define LIVOX_LIDAR_H

#include "livox_lidar/plan_env/decay_map.hpp"

#define ThreshholdPoints 10
#define PointSizeNumber 11

class LivoxDetect
{
    public:
        LivoxDetect(ros::NodeHandle &n);
        void lidarCallback(const ros::TimerEvent&);
        // vector<int> lidarCallback(vector<vector<double>> PillarLocation,int TargetPillar);

    private:
        ros::NodeHandle my_livox_nh;
        DecayMap::Ptr dcm_ptr;
        shared_ptr<DecayMapConfig> dcm_cfg_ptr;
        float delta_x;
        float delta_y;
        float delta_z;
        ros::Timer timer;
        int TargetPillar;
        ros::Time start_time;
        float lidar_decay_time;
        int visual_detect;
        int update_count;
};

#endif