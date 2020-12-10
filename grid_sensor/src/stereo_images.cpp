#include <iostream>
#include <string>
#include <ctime>
#include <ros/ros.h>

#include "grid_sensor/grid_sensor.h"

using namespace std;

class dataset_wrapper {
public:
    dataset_wrapper(){
        n.param("kitti/root_folder/", root_folder, root_folder);

        raw_img_folder = root_folder + "rgb_img/";
        depth_img_folder = root_folder + "depth_img/";
        label_img_folder = root_folder + "label_visual/";
        label_bin_folder = root_folder + "label_binary/";
        trajectory_file = root_folder + "CameraTrajectory.txt";

        grid_sensor = new ca::GridSensor(n);

        calibration_mat <<
        707.0912, 0, 601.8873,
        0, 707.0912, 183.1104,
        0, 0, 1;

        init_trans_to_ground <<
        1, 0, 0, 0,
        0, 0, 1, 0,
        0, -1, 0, 1,
        0, 0, 0, 1;

        grid_sensor->set_up_calibration(calibration_mat, img_height, img_width);
        grid_sensor->label_to_color = grid_sensor->label_to_color_mat.cast<uint8_t>();
    }

    void process_frame(){
        if(img_counter > total_img_num) {
            ROS_INFO_STREAM("Exceed maximum images");
            return;
        }
        char frame_index_c[256];
        sprintf(frame_index_c, "%06d", img_counter);
        string frame_index(frame_index_c);
        string left_img_name = raw_img_folder + frame_index + ".png";
        string depth_img_name = depth_img_folder + frame_index + ".png";
        string label_img_name = label_img_folder + frame_index + "_color.png";
        string label_bin_name = label_bin_folder + frame_index + ".bin";

        cv::Mat left_img = cv::imread(left_img_name, 1);
        cv::Mat depth_img = cv::imread(depth_img_name, CV_LOAD_IMAGE_ANYDEPTH);
        cv::Mat label_img = cv::imread(label_img_name, 1);

        if(left_img.data)
            cout << "read image " << frame_index_c << endl;
        else{
            std::cout << "cannt read image " << left_img_name << endl;
            return;
        }
        if(!depth_img.data){
            std::cout << "cannt read image " << depth_img_name << endl;
            return;
        }
        if(!label_img.data){
            std::cout << "cannt read image " << label_img_name << endl;
            return;
        }

        const int num_class = Vector_Xxf().rows();
        MatrixXf_row frame_label_prob;
        frame_label_prob.resize(img_width * img_height, num_class);

        if(ifstream(label_bin_name)){
            ifstream f_lables(label_bin_name.c_str(), ios::in | ios::binary);
            if(f_lables.is_open()){
                int mat_byte_size = sizeof(float) * frame_label_prob.rows() * frame_label_prob.cols();
                float *mat_field = frame_label_prob.data();
                f_lables.read((char*)mat_field, mat_byte_size);
                f_lables.close();
            }
            else
                ROS_INFO_STREAM("cannot open bianry label file" << label_bin_name);
        }
        else
            ROS_INFO_STREAM("cannot read label file " << label_bin_name);

        if(ifstream(trajectory_file)){
            all_poses.resize(total_img_num, 12);
            ifstream f_poses;
            f_poses.open(trajectory_file.c_str());
            int counter = 0;
            while(!f_poses.eof()){
                string s;
                getline(f_poses, s);
                if(!s.empty()){
                    stringstream ss;
                    ss << s;
                    float t;
                    for(int i = 0; i < 12; i++){
                        ss >> t;
                        all_poses(counter, i) = t;
                    }
                    counter++;
                    if(counter >= total_img_num) break;
                }
            }
            f_poses.close();
        }
        else
            ROS_INFO_STREAM("cannot read file " << trajectory_file);

        Matrix4f cur_trans_to_world;
        cur_trans_to_world.setIdentity();
        VectorXf cur_pose = all_poses.row(img_counter);
        cur_trans_to_world.block(0, 0, 3, 4) = Map<MatrixXf_row>(cur_pose.data(), 3, 4);
        cur_trans_to_world = init_trans_to_ground * cur_trans_to_world;

        grid_sensor->AddDepthImg(left_img, label_img, depth_img, cur_trans_to_world, frame_label_prob);
        img_counter++;

    }

private:
    ros::NodeHandle n;
    string root_folder, raw_img_folder, depth_img_folder, label_img_folder, label_bin_folder, trajectory_file;
    ca::GridSensor *grid_sensor;

    int img_counter = 0, total_img_num = 20, img_width = 1226, img_height = 370;
    Matrix3f calibration_mat;
    Matrix4f init_trans_to_ground;
    MatrixXf all_poses;

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "stereo_images_node");
    ros::NodeHandle n;

    dataset_wrapper image_wrap;

    ros::Rate loop_rate(10);
    while(ros::ok()){
        image_wrap.process_frame();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}

