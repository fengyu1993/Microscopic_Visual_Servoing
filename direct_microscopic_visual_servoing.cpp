#include "direct_microscopic_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <string>


Direct_Microscopic_Visual_Servoing::Direct_Microscopic_Visual_Servoing(int resolution_x, int resolution_y) : Microscopic_Visual_Servoing(resolution_x, resolution_y)
{
    this->L_e_ = Mat::zeros(resolution_x*resolution_y, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(resolution_x*resolution_y, 1, CV_64FC1);

    this->div_col_ = Mat::ones(resolution_y, resolution_x, CV_64FC1) * 2;
    this->div_col_.col(0).setTo(cv::Scalar(1.0));
    this->div_col_.col(this->div_col_.cols-1).setTo(cv::Scalar(1.0));

    this->div_row_ = Mat::ones(resolution_y, resolution_x, CV_64FC1) * 2;
    this->div_row_.row(0).setTo(cv::Scalar(1.0));
    this->div_row_.row(this->div_row_.rows-1).setTo(cv::Scalar(1.0));
}

// 计算直接显微视觉伺服特征误差 交互矩阵
void Direct_Microscopic_Visual_Servoing::get_feature_error_interaction_matrix()
{
    this->image_gray_error_ =  this->image_gray_current_ - this->image_gray_desired_;
    this->error_s_ = this->image_gray_error_.reshape(0, this->image_gray_error_.rows*this->image_gray_error_.cols);
     get_interaction_matrix_gray();
}

void Direct_Microscopic_Visual_Servoing::get_interaction_matrix_gray()
{
    Mat I_u, I_v, I_uu, I_vv, Delta_I;
    get_image_gradient_u(this->image_gray_current_, I_u);
    get_image_gradient_v(this->image_gray_current_, I_v);
    get_image_gradient_u(I_u, I_uu);
    get_image_gradient_v(I_v, I_vv);  
    cv::add(I_uu, I_vv, Delta_I);

    Mat Mat_div_Z = this->A_ * this->Mat_u_ + this->B_ * this->Mat_v_ + this->C_;
    Mat Mat_div_Zf_Z = 1 / this->camera_intrinsic_.Z_f - Mat_div_Z;
    Mat Mat_uIu_vIv = this->Mat_u_.mul(I_u) + this->Mat_v_.mul(I_v);
    Mat Mat_Phi_Delta_I = this->Phi_ * Delta_I;

    Mat L_Ic_vx = -I_u.mul(Mat_div_Z) * this->camera_intrinsic_.D_f_k_uv;
    Mat L_Ic_vy = -I_v.mul(Mat_div_Z) * this->camera_intrinsic_.D_f_k_uv;
    Mat L_Ic_vz = Mat_uIu_vIv.mul(Mat_div_Z) 
                    + this->camera_intrinsic_.D_f_k_uv * Mat_Phi_Delta_I.mul(Mat_div_Z).mul( Mat_div_Zf_Z);
    Mat L_Ic_wx = this->camera_intrinsic_.D_f_k_uv * I_v 
                    + Mat_uIu_vIv.mul(this->Mat_v_) / this->camera_intrinsic_.D_f_k_uv
                    + Mat_Phi_Delta_I.mul(this->Mat_v_).mul(Mat_div_Zf_Z);
    Mat L_Ic_wy = -this->camera_intrinsic_.D_f_k_uv * I_u
                    - Mat_uIu_vIv.mul(this->Mat_u_) / this->camera_intrinsic_.D_f_k_uv
                    - Mat_Phi_Delta_I.mul(this->Mat_u_).mul(Mat_div_Zf_Z);
    Mat L_Ic_wz = this->Mat_v_.mul(I_u) - this->Mat_u_.mul(I_v);
    int totalElements = static_cast<int>(this->image_gray_current_.total());
    cv::Mat L_e_col1 = this->L_e_.col(0);
    cv::Mat L_e_col2 = this->L_e_.col(1);
    cv::Mat L_e_col3 = this->L_e_.col(2);
    cv::Mat L_e_col4 = this->L_e_.col(3);
    cv::Mat L_e_col5 = this->L_e_.col(4);
    cv::Mat L_e_col6 = this->L_e_.col(5);
    // // 使用 parallel_for_ 并行赋值
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col1.at<double>(i) = L_Ic_vx.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col2.at<double>(i) = L_Ic_vy.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col3.at<double>(i) = L_Ic_vz.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col4.at<double>(i) = L_Ic_wx.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col5.at<double>(i) = L_Ic_wy.at<double>(i);
        }
    });
    cv::parallel_for_(cv::Range(0, totalElements), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            L_e_col6.at<double>(i) = L_Ic_wz.at<double>(i);
        }
    });
    this->L_e_.col(2).setTo(0);
    this->L_e_.col(3).setTo(0);
    this->L_e_.col(4).setTo(0);
    this->L_e_.col(5).setTo(0);
}

// 计算矩阵u方向上的梯度
void Direct_Microscopic_Visual_Servoing::get_image_gradient_u(const Mat& image, Mat& I_u)
{
    I_u = cv::Mat::zeros(image.rows, image.cols, CV_64FC1);
    // 中间部分：(i+1)列 - (i-1)列
    cv::subtract(image.colRange(2, image.cols), image.colRange(0, image.cols - 2), I_u.colRange(1, image.cols - 1)); // 中间部分直接相减
    cv::subtract(image.col(1), image.col(0), I_u.col(0));
    cv::subtract(image.col(image.cols - 1), image.col(image.cols - 2), I_u.col(image.cols - 1));
    cv::divide(I_u, this->div_col_, I_u);
}

// 计算矩阵v方向上的梯度
void Direct_Microscopic_Visual_Servoing::get_image_gradient_v(const Mat& image, Mat& I_v)
{
    I_v = cv::Mat::zeros(image.rows, image.cols, CV_64FC1);
    // 中间部分：(i+1)行 - (i-1)行
    cv::subtract(image.rowRange(2, image.rows), image.rowRange(0, image.rows - 2), I_v.rowRange(1, image.rows - 1)); // 中间部分直接相减
    cv::subtract(image.row(1), image.row(0), I_v.row(0));
    cv::subtract(image.row(image.rows - 1), image.row(image.rows - 2), I_v.row(image.rows - 1));
    cv::divide(I_v, this->div_row_, I_v);
}

 void Direct_Microscopic_Visual_Servoing::updeta_planar_paramters(double A, double B, double C)
{
     this->A_ = A;
     this->B_ = B;
     this->C_ = C;
}

void Direct_Microscopic_Visual_Servoing::init_other_parameter()
{
    this->Phi_ = (pow(this->camera_intrinsic_.R_f,2) * this->camera_intrinsic_.D_f_k_uv) / (9*this->camera_intrinsic_.Z_f);

    cv::Mat Mat_u = cv::Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    cv::parallel_for_(cv::Range(0, Mat_u.rows), [&](const cv::Range& range) {
        for (int r = range.start; r < range.end; ++r) {
            double* rowPtr = Mat_u.ptr<double>(r);
            for (int c = 0; c < Mat_u.cols; ++c) {
                rowPtr[c] = c;
            }
        }
    });
    this->Mat_u_ = Mat_u - this->camera_intrinsic_.c_u;

    Mat Mat_v = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    cv::parallel_for_(cv::Range(0, Mat_v.rows), [&](const cv::Range& range) {
        for (int r = range.start; r < range.end; ++r) {
            double* rowPtr = Mat_v.ptr<double>(r);
            for (int c = 0; c < Mat_v.cols; ++c) {
                rowPtr[c] = r;  // 每个元素赋值为行号
            }
        }
    });
    this->Mat_v_ = Mat_v - this->camera_intrinsic_.c_v;

    // cout << "div_col_(1:5, 1:6): \n" << div_col_(cv::Range(0, 5), cv::Range(0, 6)).clone() << endl;
    // cout << "div_row_(1:5, 1:6): \n" << div_row_(cv::Range(0, 5), cv::Range(0, 6)).clone() << endl;
    // cout << "Phi_ \n" << Phi_ << endl;
    // cout << "Mat_u(1:5, 1:6): \n" << Mat_u(cv::Range(0, 5), cv::Range(0, 6)).clone() << endl;
    // cout << "Mat_v(1:5, 1:6): \n" << Mat_v(cv::Range(0, 5), cv::Range(0, 6)).clone() << endl;
    // cout << "Mat_u_(1:5, 1:6): \n" << Mat_u_(cv::Range(0, 5), cv::Range(0, 6)).clone() << endl;
    // cout << "Mat_v_(1:5, 1:6): \n" << Mat_v_(cv::Range(0, 5), cv::Range(0, 6)).clone() << endl;
}

string Direct_Microscopic_Visual_Servoing::get_method_name()
{
    return "Direct_Microscopic_Visual_Servoing";
}



