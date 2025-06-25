#include "microscopic_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <chrono>

Microscopic_Visual_Servoing::Microscopic_Visual_Servoing(int resolution_x, int resolution_y)
{
    this->resolution_x_ = resolution_x;
    this->resolution_y_ = resolution_y;
    this->image_gray_desired_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->image_gray_current_ = Mat::zeros(this->resolution_y_, this->resolution_x_, CV_64FC1);
    this->object_velocity_ = Mat::zeros(6, 1, CV_64FC1);
    this->iteration_num_ = 0;
    this->Ad_Tbc_ = Mat::zeros(6, 6, CV_64FC1);
}

// 初始化
void Microscopic_Visual_Servoing::init_VS(double lambda, double epsilon, Mat& image_gray_desired, camera_intrinsic& camera_parameters, Mat pose_desired, Mat& Tbc)
{
    this->lambda_ = lambda;
    this->epsilon_ = epsilon;
    this->camera_intrinsic_ = camera_parameters;

    Tbc(cv::Rect(0, 0, 3, 3)).copyTo(this->Ad_Tbc_(cv::Rect(0, 0, 3, 3)));
    Tbc(cv::Rect(0, 0, 3, 3)).copyTo(this->Ad_Tbc_(cv::Rect(3, 3, 3, 3)));
    cv::Mat t_x_R = skewSymmetric(Tbc(cv::Rect(3, 0, 1, 3))) * Tbc(cv::Rect(0, 0, 3, 3));
    t_x_R.copyTo(this->Ad_Tbc_(cv::Rect(3, 0, 3, 3)));

    set_image_gray_desired(image_gray_desired);
    set_pose_desired(pose_desired);
    save_pose_desired();

    init_other_parameter();
}

Mat Microscopic_Visual_Servoing::skewSymmetric(const Mat& v) {
    CV_Assert(v.rows == 3 && v.cols == 1);

    double vx = v.at<double>(0);
    double vy = v.at<double>(1);
    double vz = v.at<double>(2);

    cv::Mat skew = cv::Mat::zeros(3, 3, CV_64FC1);

    skew.at<double>(0, 0) = 0;     skew.at<double>(0, 1) = -vz;   skew.at<double>(0, 2) = vy;
    skew.at<double>(1, 0) = vz;    skew.at<double>(1, 1) = 0;     skew.at<double>(1, 2) = -vx;
    skew.at<double>(2, 0) = -vy;   skew.at<double>(2, 1) = vx;    skew.at<double>(2, 2) = 0;

    return skew;
}

// 计算物体速度
Mat Microscopic_Visual_Servoing::get_object_velocity()
{
    this->iteration_num_++;
    get_feature_error_interaction_matrix();
    // cout << "L_e(1:5, 1:6): \n" << this->L_e_(cv::Range(0, 5), cv::Range(0, 6)).clone() << endl;
    // cout << "error_s_(1:5): \n" << this->L_e_(cv::Range(0, 5), cv::Range(0, 1)).clone() << endl;
    Mat L_e_transpose = this->L_e_.t();
    Mat L_e_left_inverse = (L_e_transpose * this->L_e_).inv() * L_e_transpose;
    Mat camera_velocity = -this->lambda_ * L_e_left_inverse * this->error_s_;
    this->object_velocity_ = this->Ad_Tbc_ * camera_velocity;
    return this->object_velocity_;
}

// 判断是否伺服成功
bool Microscopic_Visual_Servoing::is_success()
{
    this->cost_function_value_ = cv::norm(this->error_s_, cv::NORM_L2SQR);

    if(this->cost_function_value_ < this->epsilon_)
    {
        cout << "Visual Servoing Success" << endl;
        return true;
    }
    else
    {
        return false;
    }
}

// 设置期望灰度图像
void Microscopic_Visual_Servoing::set_image_gray_desired(Mat& image_gray_desired)
{
    image_gray_desired.copyTo(this->image_gray_desired_);
}

// 设置当前灰度图像
void Microscopic_Visual_Servoing::set_image_gray_current(Mat& image_gray_current)
{
    image_gray_current.copyTo(this->image_gray_current_);
}

// 设置初始图像
void Microscopic_Visual_Servoing::set_image_gray_initial(const Mat& image_gray_initial)
{
    image_gray_initial.copyTo(this->image_gray_initial_);

}

// 保存期望位姿
void Microscopic_Visual_Servoing::set_pose_desired(Mat& pose_desired)
{
    pose_desired.copyTo(this->pose_desired_);
}


// 保存图像数据
void Microscopic_Visual_Servoing::save_pose_desired()
{
    this->pose_desired_.copyTo(this->data_vs_.pose_desired_);
}

// 保存图像数据
void Microscopic_Visual_Servoing::save_data_image()
{
    // 保存期望图像
    this->image_gray_desired_.copyTo(this->data_vs_.image_gray_desired_);
    // 保存初始图像
    this->image_gray_initial_.copyTo(this->data_vs_.image_gray_init_);
}

// 保存相机速度
void Microscopic_Visual_Servoing::save_data_object_velocity()
{
    this->data_vs_.velocity_.push_back(this->object_velocity_.t());
}

// 保存特征误差
void Microscopic_Visual_Servoing::save_data_error_feature()
{

    this->data_vs_.error_feature_.push_back(this->cost_function_value_);
}

// 保存相机位姿
void Microscopic_Visual_Servoing::save_data_camera_pose(Mat& pose)
{
    this->data_vs_.pose_.push_back(pose);
}

// 保存所有数据
void Microscopic_Visual_Servoing::save_all_data(Mat pose)
{
    save_data_object_velocity();
    save_data_camera_pose(pose);
    save_data_error_feature();
    save_data_vs_time();
    save_data_other_parameter();
}

// 记录时间
void Microscopic_Visual_Servoing::save_data_vs_time()
{

    if (this->iteration_num_ == 1)
    {
        this->start_VS_time_ = clock();
        this->data_vs_.time_vs_.push_back(0.0);
    }
    else
    {
        this->data_vs_.time_vs_.push_back((double)(clock() - this->start_VS_time_) / CLOCKS_PER_SEC);
    }
}


// 将数据保存在文件中
void Microscopic_Visual_Servoing::write_data()
{
    save_data_image();
    string file_name = get_save_file_name();
    string location = "E:/QT/Microscopic_Visual_Servoing/resources/data/";
    // 保存图像
    string saveImage_desired = location + file_name + "_desired_image.png";
    string saveImage_initial = location + file_name + "_initial_image.png";
    imwrite(saveImage_desired, this->data_vs_.image_gray_desired_*255);
    imwrite(saveImage_initial, this->data_vs_.image_gray_init_*255);
    // 保存数据
    ofstream oFile;
    string excel_name = location + file_name + "_data.xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    write_visual_servoing_data(oFile);
    write_other_data(oFile);
    // 关闭文件
    oFile.close();
}

// 写入基本视觉伺服数据到文件
void Microscopic_Visual_Servoing::write_visual_servoing_data(ofstream& oFile)
{
    oFile << "camera velocity" << endl;
    write_to_excel(this->data_vs_.velocity_, oFile);
    oFile << "camera pose" << endl;
    write_to_excel(this->data_vs_.pose_, oFile);
    oFile << "camera desired pose" << endl;
    write_to_excel(this->data_vs_.pose_desired_, oFile);
    oFile << "error feature" << endl;
    write_to_excel(this->data_vs_.error_feature_, oFile);
    oFile << "visual servoing time" << endl;
    write_to_excel(this->data_vs_.time_vs_, oFile);
}

void Microscopic_Visual_Servoing::write_other_data(ofstream& oFile)
{
    oFile << "visual servoing end" << endl;
}

// 存储数据文件命名
string Microscopic_Visual_Servoing::get_save_file_name()
{
    return get_date_time() + "_" + get_method_name();
}

// 视觉伺服方法名字
string Microscopic_Visual_Servoing::get_method_name()
{
    return "Visual_Servoing";
}

// 获取当前计算机时间
string Microscopic_Visual_Servoing::get_date_time()
{
    auto to_string = [](const std::chrono::system_clock::time_point& t)->std::string
    {
        auto as_time_t = std::chrono::system_clock::to_time_t(t);
        struct tm tm;
#if defined(WIN32) || defined(_WINDLL)
        localtime_s(&tm, &as_time_t);  //win api，线程安全，而std::localtime线程不安全
#else
        localtime_r(&as_time_t, &tm);//linux api，线程安全
#endif

        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
        char buf[128];
        snprintf(buf, sizeof(buf), "%04d_%02d_%02d_%02d_%02d_%02d",
                 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        return buf;
    };

    std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
    return to_string(t);
}

void Microscopic_Visual_Servoing::write_to_excel(Mat data, ofstream& oFile)
{
    int channels = data.channels();            //获取图像channel
    int nrows = data.rows;                     //矩阵的行数
    int ncols = data.cols*channels;            //矩阵的总列数=列数*channel分量数

    //循环用变量
    int i = 0;
    int j = 0;

    if (data.depth() == CV_8U)//uchar
    {
        for (i = 0; i<nrows; i++)
        {
            for (j = 0; j<ncols; j++)
            {
                int tmpVal = (int)data.ptr<uchar>(i)[j];
                oFile << tmpVal << '\t';
            }
            oFile << endl;
        }
    }
    else if (data.depth() == CV_16S)//short
    {
        for (i = 0; i<nrows; i++)
        {
            for (j = 0; j<ncols; j++)
            {
                oFile << (short)data.ptr<short>(i)[j] << '\t';
            }
            oFile << endl;
        }
    }
    else if (data.depth() == CV_16U)//unsigned short
    {
        for (i = 0; i<nrows; i++)
        {
            for (j = 0; j<ncols; j++)
            {
                oFile << (unsigned short)data.ptr<unsigned short>(i)[j] << '\t';
            }
            oFile << endl;
        }
    }
    else if (data.depth() == CV_32S)//int
    {
        for (i = 0; i<nrows; i++)
        {
            for (j = 0; j<ncols; j++)
            {
                oFile << (int)data.ptr<int>(i)[j] << '\t';
            }
            oFile << endl;
        }
    }
    else if (data.depth() == CV_32F)//float
    {
        for (i = 0; i<nrows; i++)
        {
            for (j = 0; j<ncols; j++)
            {
                oFile << (float)data.ptr<float>(i)[j] << '\t';
            }
            oFile << endl;
        }
    }
    else//CV_64F double
    {
        for (i = 0; i < nrows; i++)
        {
            for (j = 0; j < ncols; j++)
            {
                oFile << (double)data.ptr<double>(i)[j] << '\t';
            }
            oFile << endl;
        }
    }
}
