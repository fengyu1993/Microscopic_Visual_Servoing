#ifndef DIRECT_MICROSCOPIC_VISUAL_SERVOING
#define DIRECT_MICROSCOPIC_VISUAL_SERVOING

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "microscopic_visual_servoing.h"

using namespace cv;
using namespace std;

class Direct_Microscopic_Visual_Servoing: public Microscopic_Visual_Servoing
{
    private:
        Mat div_col_;
        Mat div_row_;
        double Phi_;
        Mat Mat_u_;
        Mat Mat_v_;
        double A_;
        double B_;
        double C_;

    public: 
        Direct_Microscopic_Visual_Servoing(int resolution_x=1600, int resolution_y=1200);

        virtual ~Direct_Microscopic_Visual_Servoing() = default;

        virtual void get_feature_error_interaction_matrix();

        void get_interaction_matrix_gray();

        void get_image_gradient_u(const Mat& image, Mat& I_x);

        void get_image_gradient_v(const Mat& image, Mat& I_y);

        void updeta_planar_paramters(double A, double B, double C);

        virtual string get_method_name();
};


#endif
