#ifndef MICROSCOPIC_VISUAL_SERVOING
#define MICROSCOPIC_VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;
using namespace std;

struct  camera_intrinsic
{
    double c_u;
    double c_v;
    double Z_f;
    double D_f_k_uv;
    double R_f;
};

class Microscopic_Visual_Servoing
{
public:
    Mat image_gray_desired_;
    Mat image_gray_current_;
    Mat image_gray_initial_;
    camera_intrinsic camera_intrinsic_;
    Mat object_velocity_; // vx vy vz wx wy wz
    Mat pose_desired_;
    double lambda_;
    double epsilon_;
    int resolution_x_;
    int resolution_y_;
    Mat L_e_;
    Mat error_s_;
    Mat Ad_Tbc_;
    int iteration_num_;
    clock_t start_VS_time_;
    double cost_function_value_;
    struct data
    {
        Mat velocity_;
        Mat pose_; // 平移+四元数wxyz
        Mat pose_desired_; // 平移+四元数wxyz
        Mat error_feature_;
        Mat image_gray_init_;
        Mat image_gray_desired_;
        Mat time_vs_;
    } data_vs_;

public:
    Microscopic_Visual_Servoing(int resolution_x, int resolution_y);

    void init_VS(double lambda, double epsilon, Mat& image_gray_desired, camera_intrinsic& camera_parameters, Mat pose_desired, Mat& Toc);

    Mat skewSymmetric(const Mat& v);

    Mat get_object_velocity();

    virtual bool is_success();

    virtual void get_feature_error_interaction_matrix() = 0;

    void set_image_gray_desired(Mat& image_gray_desired);

    void set_image_gray_current(Mat& image_gray_current);

    void set_image_gray_initial(const Mat& image_gray_initial);

    void set_pose_desired(Mat& pose_desired);

    void save_pose_desired();

    void save_data_image();

    void save_data_object_velocity();

    void save_data_error_feature();

    void save_data_camera_pose(Mat& pose);

    void save_data_vs_time();

    virtual void save_data_other_parameter() {};

    virtual void init_other_parameter() {};

    void save_all_data(Mat pose);

    void write_data();

    void write_visual_servoing_data(ofstream& oFile);

    virtual void write_other_data(ofstream& oFile);

    string get_save_file_name();

    string get_date_time();

    virtual string get_method_name();

    void write_to_excel(Mat data, ofstream& oFile);
};


#endif
