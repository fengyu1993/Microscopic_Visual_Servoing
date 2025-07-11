#include "MicroscopeCalibration.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

MicroscopeCalibration::MicroscopeCalibration(int resolution_x, int resolution_y)
{
    microscopicParameter.c_u = resolution_x / 2.0;
    microscopicParameter.c_v = resolution_y / 2.0;
}

void MicroscopeCalibration::calibration(const Calibration_Data& calibrationData, Microscopic_Parameter& microscopicParameter)
{
    // 构建delta_Puv和delta_Pxy
    Mat pointsPuvMoveXYD0; Mat pointsPXYZMoveXYD0;
    hconcat(calibrationData.pointsPuvMoveXD0, calibrationData.pointsPuvMoveYD0, pointsPuvMoveXYD0);
    hconcat(calibrationData.pointsPXYZMoveXD0, calibrationData.pointsPXYZMoveYD0, pointsPXYZMoveXYD0);
    Mat col1_repeated;
    repeat(pointsPuvMoveXYD0.col(0), 1, pointsPuvMoveXYD0.cols-1, col1_repeated);
    Mat delta_Puv_D0 = pointsPuvMoveXYD0.colRange(1, pointsPuvMoveXYD0.cols) - col1_repeated;
    repeat(pointsPXYZMoveXYD0.col(0), 1, pointsPXYZMoveXYD0.cols-1, col1_repeated);
    Mat delta_Pxy_D0 = pointsPXYZMoveXYD0.colRange(1, pointsPXYZMoveXYD0.cols) - col1_repeated;
    repeat(calibrationData.pointsPuvMoveXYDn.col(0), 1, calibrationData.pointsPuvMoveXYDn.cols-1, col1_repeated);
    Mat delta_Puv_Dn = calibrationData.pointsPuvMoveXYDn.colRange(1, calibrationData.pointsPuvMoveXYDn.cols) - col1_repeated;
    repeat(calibrationData.pointsPXYZMoveXYDn.col(0), 1, calibrationData.pointsPXYZMoveXYDn.cols-1, col1_repeated);
    Mat delta_Pxy_Dn = calibrationData.pointsPXYZMoveXYDn.colRange(1, calibrationData.pointsPXYZMoveXYDn.cols) - col1_repeated;
    repeat(calibrationData.pointsPuvMoveXYDp.col(0), 1, calibrationData.pointsPuvMoveXYDp.cols-1, col1_repeated);
    Mat delta_Puv_Dp = calibrationData.pointsPuvMoveXYDp.colRange(1, calibrationData.pointsPuvMoveXYDp.cols) - col1_repeated;
    repeat(calibrationData.pointsPXYZMoveXYDp.col(0), 1, calibrationData.pointsPXYZMoveXYDp.cols-1, col1_repeated);
    Mat delta_Pxy_Dp = calibrationData.pointsPXYZMoveXYDp.colRange(1, calibrationData.pointsPXYZMoveXYDp.cols) - col1_repeated;
    // 计算Z_f和D_f_k_uv
    double d0 = 0;
    double dp = mean(calibrationData.pointsPXYZMoveXD0.row(2))[0] - mean(calibrationData.pointsPXYZMoveXYDp.row(2))[0];
    double dn = mean(calibrationData.pointsPXYZMoveXD0.row(2))[0] - mean(calibrationData.pointsPXYZMoveXYDn.row(2))[0];
    Mat Li_D0, li_D0, A_D0, b_D0, Li_Dp, li_Dp, A_Dp, b_Dp, Li_Dn, li_Dn, A_Dn, b_Dn;
    cv::magnitude(delta_Puv_D0.row(0), delta_Puv_D0.row(1), li_D0);
    cv::magnitude(delta_Pxy_D0.row(0), delta_Pxy_D0.row(1), Li_D0);
    hconcat(Li_D0.t(), -li_D0.t(), A_D0);
    b_D0 = li_D0.t() * d0;
    cv::magnitude(delta_Puv_Dp.row(0), delta_Puv_Dp.row(1), li_Dp);
    cv::magnitude(delta_Pxy_Dp.row(0), delta_Pxy_Dp.row(1), Li_Dp);
    hconcat(Li_Dp.t(), -li_Dp.t(), A_Dp);
    b_Dp = li_Dp.t() * dp;
    cv::magnitude(delta_Puv_Dn.row(0), delta_Puv_Dn.row(1), li_Dn);
    cv::magnitude(delta_Pxy_Dn.row(0), delta_Pxy_Dn.row(1), Li_Dn);
    hconcat(Li_Dn.t(), -li_Dn.t(), A_Dn);
    b_Dn = li_Dn.t() * dn;
    Mat A, b;
    vconcat(std::vector<cv::Mat>{A_D0, A_Dp, A_Dn}, A);
    vconcat(std::vector<cv::Mat>{b_D0, b_Dp, b_Dn}, b);
   Mat X_Df_Zf = (A.t() * A).inv() * A.t() * b;
    microscopicParameter.D_f_k_uv = X_Df_Zf.at<double>(0,0) / 1000.0;
    microscopicParameter.Z_f = X_Df_Zf.at<double>(1,0) / 1000.0; // 单位：μm
    // 计算rx
    repeat(calibrationData.pointsPuvMoveXD0.col(0), 1, calibrationData.pointsPuvMoveXD0.cols-1, col1_repeated);
    Mat delta_Puv_MoveXD0 = calibrationData.pointsPuvMoveXD0.colRange(1, calibrationData.pointsPuvMoveXD0.cols) - col1_repeated;
    for(int i = 0; i < delta_Puv_MoveXD0.cols; i++)
    {
        cv::normalize(delta_Puv_MoveXD0.col(i), delta_Puv_MoveXD0.col(i), 1.0, 0.0, cv::NORM_L2);
    }
    Mat rx = Mat::zeros(3, 1, CV_64F);
    rx.at<double>(0, 0) = mean(delta_Puv_MoveXD0.row(0))[0];
    rx.at<double>(1, 0) = mean(delta_Puv_MoveXD0.row(1))[0];
    rx = rx / norm(rx);
    // 计算ry
    repeat(calibrationData.pointsPuvMoveYD0.col(0), 1, calibrationData.pointsPuvMoveYD0.cols-1, col1_repeated);
    Mat delta_Puv_MoveYD0 = calibrationData.pointsPuvMoveYD0.colRange(1, calibrationData.pointsPuvMoveYD0.cols) - col1_repeated;
    for(int i = 0; i < delta_Puv_MoveYD0.cols; i++)
    {
        cv::normalize(delta_Puv_MoveYD0.col(i), delta_Puv_MoveYD0.col(i), 1.0, 0.0, cv::NORM_L2);
    }
    Mat ry = Mat::zeros(3, 1, CV_64F);
    ry.at<double>(0, 0) = mean(delta_Puv_MoveYD0.row(0))[0];
    ry.at<double>(1, 0) = mean(delta_Puv_MoveYD0.row(1))[0];
    ry = ry / norm(ry);
    // 计算R_cb
    double r11 = rx.at<double>(0, 0);
    double r21 = rx.at<double>(1, 0);
    double r12 = ry.at<double>(0, 0);
    double r22 = ry.at<double>(1, 0);
    double px = r22 - r11;
    double py = r12 + r21;
    double theta = atan2(py, px) + CV_PI;
    Mat R_cb = (cv::Mat_<double>(3, 3)
                    <<cos(theta), -sin(theta), 0,
                        -sin(theta), -cos(theta), 0,
                          0, 0, -1);
    // 计算t_cb
    Mat A_t_cb = Mat::zeros(0, 3, CV_64F);
    Mat b_t_cb = Mat::zeros(0, 1, CV_64F);
    Mat R_be0 = calibrationData.posesTsRotateZD0[0].rowRange(0, 3).colRange(0,3);
    Mat t_be0 = calibrationData.posesTsRotateZD0[0].rowRange(0, 3).col(3);
    Mat P_c0 = (cv::Mat_<double>(3, 1) <<
                    (calibrationData.pointsPuvRotateZD0.at<double>(0, 0) - microscopicParameter.c_u) * microscopicParameter.Z_f / microscopicParameter.D_f_k_uv,
                (calibrationData.pointsPuvRotateZD0.at<double>(1, 0) - microscopicParameter.c_v) * microscopicParameter.Z_f / microscopicParameter.D_f_k_uv,
                microscopicParameter.Z_f);
    for(int j = 1; j < calibrationData.posesTsRotateZD0.size(); j++)
    {
        Mat R_ej = calibrationData.posesTsRotateZD0[j].rowRange(0, 3).colRange(0,3);
        Mat t_ej = calibrationData.posesTsRotateZD0[j].rowRange(0, 3).col(3);
        Mat P_cj = (cv::Mat_<double>(3, 1) <<
                        (calibrationData.pointsPuvRotateZD0.at<double>(0, j) - microscopicParameter.c_u) * microscopicParameter.Z_f / microscopicParameter.D_f_k_uv,
                    (calibrationData.pointsPuvRotateZD0.at<double>(1, j) - microscopicParameter.c_v) * microscopicParameter.Z_f / microscopicParameter.D_f_k_uv,
                    microscopicParameter.Z_f);
        Mat temp_A = (R_be0.t() - R_ej.t()) * R_cb.t();
        Mat temp_b = (R_be0.t()  * (R_cb.t() * P_c0 - t_be0) - R_ej.t() * (R_cb.t() * P_cj - t_ej));
        vconcat(A_t_cb, temp_A, A_t_cb);
        vconcat(b_t_cb, temp_b, b_t_cb);
    }
    Mat A_t_cb_pinv;
    invert(A_t_cb, A_t_cb_pinv, DECOMP_SVD);
    Mat t_cb = A_t_cb_pinv * b_t_cb / 1000.0;  // 单位：μm
    t_cb.at<double>(2, 0) = microscopicParameter.Z_f;
    // 计算Tbc
    microscopicParameter.Tbc.rowRange(0,3).colRange(0,3) = R_cb.t();
    microscopicParameter.Tbc.rowRange(0,3).col(3) = -R_cb.t() * t_cb;
    // 计算Rf
    Mat r_ai = calibrationData.radiusZMoveZ.row(0).clone();
    Mat Z_i = calibrationData.radiusZMoveZ.row(1).clone();
    Mat r_npi = microscopicParameter.Z_f * calibrationData.rf / Z_i;
    Mat r_i = r_ai - r_npi;
    Mat temp_AA = (microscopicParameter.D_f_k_uv * (1.0 / microscopicParameter.Z_f - 1.0 / Z_i)).t();
    Mat tamp_bb = r_i.t();
    Mat AA_pinv;
    invert(temp_AA, AA_pinv, DECOMP_SVD);
    Mat Mat_Rf = AA_pinv * tamp_bb;
    microscopicParameter.R_f = Mat_Rf.at<double>(0,0);
}

void MicroscopeCalibration::writeCalibrationData(const Calibration_Data& calibrationData)
{
        string file_name = "calibrationData";
        string location = LOCATION;
        ofstream oFile;
        string excel_name = location + file_name + ".xls";
        oFile.open(excel_name, ios::out|ios::trunc);
        oFile << "pointsPuvMoveXD0" << endl;
        write_to_excel(calibrationData.pointsPuvMoveXD0, oFile);
        oFile << "pointsPXYZMoveXD0" << endl;
        write_to_excel(calibrationData.pointsPXYZMoveXD0, oFile);
        oFile << "pointsPuvMoveYD0" << endl;
        write_to_excel(calibrationData.pointsPuvMoveYD0, oFile);
        oFile << "pointsPXYZMoveYD0" << endl;
        write_to_excel(calibrationData.pointsPXYZMoveYD0, oFile);
        oFile << "pointsPuvMoveXYDn" << endl;
        write_to_excel(calibrationData.pointsPuvMoveXYDn, oFile);
        oFile << "pointsPXYZMoveXYDn" << endl;
        write_to_excel(calibrationData.pointsPXYZMoveXYDn, oFile);
        oFile << "pointsPuvMoveXYDp" << endl;
        write_to_excel(calibrationData.pointsPuvMoveXYDp, oFile);
        oFile << "pointsPXYZMoveXYDp" << endl;
        write_to_excel(calibrationData.pointsPXYZMoveXYDp, oFile);
        oFile << "pointsPuvRotateZD0" << endl;
        write_to_excel(calibrationData.pointsPuvRotateZD0, oFile);
        oFile << "posesTsRotateZD0" << endl;
        for(int i = 0; i < calibrationData.posesTsRotateZD0.size(); i++)
        {
            write_to_excel(calibrationData.posesTsRotateZD0[i], oFile);
        }
        oFile.close();
}

void MicroscopeCalibration::writeCalibrationResult(const Microscopic_Parameter& microscopicParameter)
{
    string file_name = "calibrationResult";
    string location = LOCATION;
    ofstream oFile;
    string excel_name = location + file_name + ".xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    oFile << "c_u" << endl;
    oFile << microscopicParameter.c_u << endl;
    oFile << "c_v" << endl;
    oFile << microscopicParameter.c_v << endl;
    oFile << "D_f_k_uv" << endl;
    oFile << microscopicParameter.D_f_k_uv << endl;
    oFile << "Z_f" << endl;
    oFile << microscopicParameter.Z_f << endl;
    oFile << "R_f" << endl;
    oFile << microscopicParameter.R_f << endl;
    oFile << "Tbc" << endl;
    write_to_excel(microscopicParameter.Tbc, oFile);
    oFile.close();
}

void MicroscopeCalibration::write_to_excel(Mat data, ofstream& oFile)
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

