#include "grid_sensor/grid_sensor.h"


void ca::GridSensor::set_up_calibration(const Matrix3f &calibration_mat, const int img_height,
                                        const int img_width) {
    matx_to3d_.create(img_height, img_width);
    maty_to3d_.create(img_height, img_width);
    float center_x = calibration_mat(0, 2);
    float center_y = calibration_mat(1, 2);
    float fx_inv = 1.0 / calibration_mat(0, 0);
    float fy_inv = 1.0 / calibration_mat(1, 1);

    for(int v = 0; v < img_height; v++){
        for(int u = 0; u < img_width; u++){
            matx_to3d_(v, u) = (u - center_x) * fx_inv;// Pw(x,)
            maty_to3d_(v, u) = (v - center_y) * fy_inv;//Pw(,y)
        }
    }
}

void ca::GridSensor::AddDepthImg(const cv::Mat &rgb_img, const cv::Mat &label_img, const cv::Mat &depth_img,
                                 const Matrix4f cur_trans_to_world, MatrixXf_row &frame_label_prob) {
    ros::Time cur_time = ros::Time::now();
    preprocess_pose(cur_time, rgb_img, depth_img, label_img, cur_trans_to_world);

    PointCloudXYZRGB::Ptr point_cloud(new PointCloudXYZRGB());

    float pixel_depth;
    pcl::PointXYZRGB pt;
    int pixel_label;
    for(int32_t i = 0; i < depth_img.cols * depth_img.rows; i++){
        int ux = i % depth_img.cols;
        int uy = i / depth_img.cols;
        pixel_depth = (float) depth_img.at<uint16_t>(uy, ux);
        pixel_depth /= depth_scaling;
        if(pixel_depth > depth_ignore_th) continue;

        frame_label_prob.row(i).maxCoeff(&pixel_label);
        if(pixel_label == sky_label) continue;

        pt.z = pixel_depth;
        pt.x = matx_to3d_(uy, ux) * pixel_depth;
        pt.y = maty_to3d_(uy, ux) * pixel_depth;

        VectorXf global_pt;
        VectorXf pt_homo = cur_trans_to_world * Vector4f (pt.x, pt.y, pt.z, 1);
        if(pt_homo.rows() == 4)
            global_pt = pt_homo.head(3) / pt_homo(3);
        else if(pt_homo.rows() == 3)
            global_pt = pt_homo.head(2) / pt_homo(2);

        pt.x = global_pt(0);
        pt.y = global_pt(1);
        pt.z = global_pt(2);
//        pt.r = rgb_img.at<cv::Vec3b>(uy, ux)[2];
//        pt.g = rgb_img.at<cv::Vec3b>(uy, ux)[1];
//        pt.b = rgb_img.at<cv::Vec3b>(uy, ux)[0];
        pt.r = label_to_color(pixel_label, 0);
        pt.g = label_to_color(pixel_label, 1);
        pt.b = label_to_color(pixel_label, 2);

        point_cloud->points.push_back(pt);
    }
    pcl::toPCLPointCloud2(*point_cloud, cloud_);
    point_cloud->header.frame_id = "/world";
    point_cloud->header.stamp = (cur_time.toNSec() / 1000ull);
    raw_cloud_pub.publish(*point_cloud);
}

void ca::GridSensor::preprocess_pose(ros::Time cur_time,
                                     const cv::Mat &rgb_img, const cv::Mat &depth_img, const cv::Mat &label_img,
                                     const Matrix4f cur_trans_to_world) {
    cv_bridge::CvImage out_img_msg;
    out_img_msg.header.stamp = cur_time;

    cv::Mat rgb_img_small;
    cv::resize(rgb_img, rgb_img_small, cv::Size(), 0.5, 0.5);
    out_img_msg.image = rgb_img_small;
    out_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    raw_rgb_img_pub.publish(out_img_msg.toImageMsg());

    cv::Mat label_img_small;
    cv::resize(label_img, label_img_small, cv::Size(), 0.5, 0.5);
    out_img_msg.image = label_img_small;
    out_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    raw_label_img_pub.publish(out_img_msg.toImageMsg());

    cv::Mat depth_img_small;
    cv::resize(depth_img, depth_img_small, cv::Size(), 0.5, 0.5);
    out_img_msg.image = depth_img_small;
    //out_img_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    //raw_depth_img_pub.publish(out_img_msg.toImageMsg());

    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = cur_trans_to_world(0, 3);
    odom_msg.pose.pose.position.y = cur_trans_to_world(1, 3);
    odom_msg.pose.pose.position.z = cur_trans_to_world(2, 3);
    Matrix3f pose_rot = cur_trans_to_world.block(0, 0, 3, 3);
    Quaternionf pose_quat(pose_rot);
    odom_msg.pose.pose.orientation.w = pose_quat.w();
    odom_msg.pose.pose.orientation.x = pose_quat.x();
    odom_msg.pose.pose.orientation.y = pose_quat.y();
    odom_msg.pose.pose.orientation.z = pose_quat.z();
    odom_msg.header.frame_id = "/world";
    odom_msg.header.stamp = cur_time;
    odom_pub.publish(odom_msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(cur_trans_to_world(0, 3),
                                    cur_trans_to_world(1, 3),
                                    cur_trans_to_world(2, 3)));
    transform.setBasis(tf::Matrix3x3(cur_trans_to_world(0, 0), cur_trans_to_world(0, 1), cur_trans_to_world(0, 2),
                                     cur_trans_to_world(1, 0), cur_trans_to_world(1, 1), cur_trans_to_world(1, 2),
                                     cur_trans_to_world(2, 0), cur_trans_to_world(2, 1), cur_trans_to_world(2, 2)));
    br.sendTransform(tf::StampedTransform(transform, cur_time, "/world", "/base_frame"));
}
