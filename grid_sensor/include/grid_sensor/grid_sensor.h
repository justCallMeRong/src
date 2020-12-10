#ifndef SRC_GRID_SENSOR_H
#define SRC_GRID_SENSOR_H

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <boost/unordered_map.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace Eigen;
using namespace std;

//something should be other place
typedef Matrix<float, 11, 1> Vector_Xxf;

typedef Matrix<float, Dynamic, Dynamic, RowMajor> MatrixXf_row;
typedef Matrix<uint8_t, Dynamic, Dynamic, RowMajor> MatrixX8u_row;

namespace ca{

    class GridSensor{
    public:
        typedef pcl::PointCloud<pcl::PointXYZRGB>  PointCloudXYZRGB;

        GridSensor(ros::NodeHandle &n): initialized_(false), counter(0) {
            n.param<int>("Grid_SG_BARELY_FREE", Grid_SG_BARELY_FREE, Grid_SG_BARELY_FREE);
            n.param<int>("Grid_SG_BARELY_OCCUPIED", Grid_SG_BARELY_OCCUPIED, Grid_SG_BARELY_OCCUPIED);
            n.param<int>("Grid_SG_BELIEF_UPDATE_POS", Grid_SG_BELIEF_UPDATE_POS, Grid_SG_BELIEF_UPDATE_POS);
            n.param<int>("Grid_SG_BELIEF_UPDATE_NEG", Grid_SG_BELIEF_UPDATE_NEG, Grid_SG_BELIEF_UPDATE_NEG);

            n.param<string>("/grid_sensor/pointCloudFrame", point_cloud_frame_, "/camera_rgb_optical_frame");
            n.param<string>("/grid_sensor/worldFrame", world_frame_, "/world");
            n.param<string>("grid_sensor/sharedGridIdentifer", shared_grid_identifer_, "shared_grid_map");

            raw_cloud_pub = n.advertise<PointCloudXYZRGB>("/rgbd_grid/raw_stereo_cloud", 50);
            raw_rgb_img_pub = n.advertise<sensor_msgs::Image>("/raw_rgb_img", 10);
            raw_label_img_pub = n.advertise<sensor_msgs::Image>("/raw_label_img", 10);
            odom_pub = n.advertise<nav_msgs::Odometry>("/odom_pose", 10);

            //proxy_ = new ca:: ProxyGridMap(shared_grid_identifer_);
            ROS_INFO_STREAM("Grid sensor : " << point_cloud_frame_ << " , " << world_frame_);

            label_to_color_mat.resize(Vector_Xxf().rows(), 3);
            label_to_color_mat <<
            128,   0,   0, // 0.building
            128, 128, 128, // 1.sky
            128,  64, 128, // 2.road
            128, 128,   0, // 3.vegetation
              0,   0, 192, // 4.sidewalk
             64,   0, 128, // 5.car
             64,  64,   0, // 6.pedestrian
              0, 128, 192, // 7.cyclist
            192, 128, 128, // 8.signate
             64,  64, 128, // 9.fence
            192, 192, 128; // 10.pole

        }

        void set_up_calibration(const Matrix3f &calibration_mat, const int img_height, const int img_width);

        void AddDepthImg(const cv::Mat &rgb_img, const cv::Mat &label_img, const cv::Mat &depth_img,
                         const Matrix4f cur_trans_to_world, MatrixXf_row &frame_label_prob);

        void preprocess_pose (ros::Time cur_time,
                              const cv::Mat &rgb_img, const cv::Mat &depth_img, const cv::Mat &label_img,
                              const Matrix4f cur_trans_to_world);

    private:
        bool initialized_;
        int counter;
        int32_t Grid_SG_BARELY_FREE = 117, Grid_SG_BARELY_OCCUPIED = 135;
        int32_t Grid_SG_BELIEF_UPDATE_POS = 10, Grid_SG_BELIEF_UPDATE_NEG = 4;
        string point_cloud_frame_, world_frame_, shared_grid_identifer_;
        ros::Publisher raw_cloud_pub, raw_rgb_img_pub, raw_depth_img_pub, raw_label_img_pub, odom_pub;

        // depth to cloud
        cv::Mat_<float> matx_to3d_, maty_to3d_;
        float depth_scaling = 1000.0, depth_ignore_th = 50.0;

        int sky_label = 1;

        pcl::PCLPointCloud2 cloud_;
    public:
        MatrixXi label_to_color_mat;
        MatrixX8u_row label_to_color;

    };

}

#endif //SRC_GRID_SENSOR_H
