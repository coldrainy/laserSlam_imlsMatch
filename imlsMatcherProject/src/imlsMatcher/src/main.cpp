#include "imls_icp.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"

#include "pcl-1.7/pcl/io/pcd_io.h"

//pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");

class imlsDebug
{
public:
    imlsDebug()
    {

        m_FrameID = 0;

        m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
    }

    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
                                                   Eigen::Vector3d odomPose,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {
        //变换矩阵
        Eigen::Matrix3d Tmatrix;
        Tmatrix << std::cos(odomPose(2)),-std::sin(odomPose(2)),odomPose(0),
                   std::sin(odomPose(2)), std::cos(odomPose(2)),odomPose(1),
                   0,0,1;

        //转换到里程计坐标系中．
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size();i++)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if(std::isnan(lx) || std::isinf(ly)||
               std::isnan(ly) || std::isinf(ly))
                continue;

            Eigen::Vector3d lpt(lx,ly,1.0);

            Eigen::Vector3d opt = Tmatrix * lpt;

            eigen_pts.push_back(Eigen::Vector2d(opt(0),opt(1)));
        }
    }

    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        static bool isFirstFrame = true;

        Eigen::Vector3d nowPose;
        if(getOdomPose(msg->header.stamp,nowPose) == false)
        {
            std::cout <<"Failed to get Odom Pose"<<std::endl;
            return ;
        }

        if(isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            isFirstFrame = false;
            m_prevLaserPose = nowPose;
            ConvertChampionLaserScanToEigenPointCloud(msg,nowPose,m_prevPointCloud);
            return ;
        }

        double delta_dist2 = std::pow(nowPose(0) - m_prevLaserPose(0),2) + std::pow(nowPose(1) - m_prevLaserPose(1),2);
        double delta_angle = std::fabs(tfNormalizeAngle(nowPose(2) - m_prevLaserPose(2)));

        if(delta_dist2 < 0.2 * 0.2 &&
           delta_angle < tfRadians(10.0))
        {
            return ;
        }

        m_prevLaserPose = nowPose;


        std::vector<Eigen::Vector2d> nowPts;
        ConvertChampionLaserScanToEigenPointCloud(msg,nowPose,nowPts);

        //TODO
        //调用imls进行icp匹配，并输出结果．
        m_imlsMatcher.setSourcePointCloud(nowPts);
        m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

        Eigen::Matrix3d rPose,rCovariance;
        if(m_imlsMatcher.Match(rPose,rCovariance))
        {
            std::cout <<"IMLS Match Successful:"<<rPose(0,2)<<","<<rPose(1,2)<<","<<atan2(rPose(1,0),rPose(0,0))*57.295<<std::endl;
        }
        else
        {
            std::cout <<"IMLS Match Failed!!!!"<<std::endl;
        }

        //end of TODO

        m_laserscanSub.shutdown();
    }

    bool getOdomPose(ros::Time t,
                     Eigen::Vector3d& pose)
    {
        // Get the robot's pose
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                   tf::Vector3(0,0,0)), t, "/base_link");
        tf::Stamped<tf::Transform> odom_pose;
        try
        {
            m_tfListener.transformPose("/odom", ident, odom_pose);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }

        double yaw = tf::getYaw(odom_pose.getRotation());
        pose << odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw;

        return true;
    }

    int m_FrameID;

    ros::NodeHandle m_nh;

    IMLSICPMatcher m_imlsMatcher;

    Eigen::Vector3d m_prevLaserPose;

    std::vector<Eigen::Vector2d> m_prevPointCloud;

    tf::TransformListener m_tfListener;
    ros::Subscriber m_laserscanSub;
    ros::Publisher m_pointcloudPub;
    ros::Publisher m_normalsPub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_debug");

    imlsDebug imls_debug;

    ros::spin();

    return (0);
}

