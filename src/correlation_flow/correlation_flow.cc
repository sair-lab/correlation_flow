// Copyright (c) <2017>, <Nanyang Technological University> All rights reserved.

// This file is part of correlation_flow.

//     correlation_flow is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     Foobar is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

#include <math.h>
#include <ros/ros.h>
#include "correlation_flow/correlation_flow.h"


CorrelationFlow::CorrelationFlow(ros::NodeHandle nh):nh(nh)
{
    width = 360;
    height = 240;
    lamda = 0.1;
    sigma = 0.2;

    ArrayXXf target = ArrayXXf::Zero(width, height);
    target(width/2, height/2) = 1;
    target_fft = fft(target);
    filter_fft = fft(ArrayXXf::Zero(width, height));

    // max_rotation = 180.0;
    rot_resolution = 1.0;
    // target_dim = 2*int(max_rotation/rot_resolution)+2;
    target_dim = 360 / rot_resolution;
    ArrayXXf target_rot = ArrayXXf::Zero(target_dim, 1);
    target_rot(target_dim/2, 0) = 1;
    target_rot_fft = fft(target_rot);
    filter_rot_fft = fft(ArrayXXf::Zero(target_dim, 1));

    max_scale = 2;
    scale_res = 0.1;
    sca_target_dim = 2*max_scale/scale_res;
    ArrayXXf target_sca = ArrayXXf::Zero(sca_target_dim, 1);
    target_sca(sca_target_dim/2, 0) = 1;
    target_sca_fft = fft(target_sca);
    filter_sca_fft = fft(ArrayXXf::Zero(sca_target_dim, 1));


    initialized = false;
}


void CorrelationFlow::callback(const sensor_msgs::ImageConstPtr& msg)
{
    timer.tic();

    cv::resize(cv_bridge::toCvShare(msg, "bgr8")->image, image, cv::Size(width, height));
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    image.convertTo(sample_cv, CV_32FC1);
    sample = Eigen::Map<ArrayXXf>(&sample_cv.at<float>(0,0), width, height)/255.0;
    sample_fft = fft(sample);

    if (initialized == false)
    {
        train = sample;
        train_fft = sample_fft;
        kernel = gaussian_kernel();
        filter_fft = target_fft/(kernel + lamda);

        rotation_base(sample_cv);
        kernel_rot = rotation_kernel(train);
        filter_rot_fft = target_rot_fft/(kernel_rot + lamda);

        scale_base(sample_cv);
        kernel_sca = scale_kernel(train);
        filter_sca_fft = target_sca_fft/(kernel_sca + lamda);

        initialized = true;
        return;
    }

    //motion of current frame
    kernel = gaussian_kernel(sample_fft);
    output = ifft(filter_fft*kernel);
    max_response = output.maxCoeff(&(max_index[0]), &(max_index[1]));

    kernel_rot = rotation_kernel(sample);
    output_rot = ifft(kernel_rot*filter_rot_fft);
    max_responseR = output_rot.maxCoeff(&(max_indexR[0]), &(max_indexR[1]));

    kernel_sca = scale_kernel(sample);
    output_sca = ifft(kernel_sca*filter_sca_fft);
    max_responseS = output_sca.maxCoeff(&(max_indexS[0]), &(max_indexS[1]));


    //update filter
    train = sample;
    train_fft = sample_fft;
    kernel = gaussian_kernel();
    filter_fft = target_fft/(kernel + lamda);

    rotation_base(sample_cv);
    kernel_rot = rotation_kernel(train);
    filter_rot_fft = target_rot_fft/(kernel_rot + lamda);

    scale_base(sample_cv);
    kernel_sca = scale_kernel(train);
    filter_sca_fft = target_sca_fft/(kernel_sca + lamda);

    timer.toc("callback:");
    ROS_WARN("x=%d, y=%d\n", int(max_index[0] - width/2), int(max_index[1] - height/2));
    ROS_WARN("rotaion angle is %f\n", (max_indexR[0]-target_dim/2)*rot_resolution);
    ROS_WARN("scaling factor is %f\n", (max_indexS[0]-sca_target_dim/2)*scale_res+1);
}


inline ArrayXXcf CorrelationFlow::fft(const ArrayXXf& x)
{
    ArrayXXcf xf = ArrayXXcf(x.rows()/2+1, x.cols());

    fft_plan = fftwf_plan_dft_r2c_2d(x.cols(), x.rows(), (float(*))(x.data()), 
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major
    
    fftwf_execute(fft_plan);
    
    return xf;
}


inline ArrayXXf CorrelationFlow::ifft(const ArrayXXcf& xf)
{
    ArrayXXf x = ArrayXXf((xf.rows()-1)*2, xf.cols());
    
    ArrayXXcf cxf = xf;
    
    fft_plan = fftwf_plan_dft_c2r_2d(xf.cols(), (xf.rows()-1)*2, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    
    fftwf_execute(fft_plan);
    
    return x/x.size();
}


inline ArrayXXcf CorrelationFlow::gaussian_kernel()
{
    unsigned int N = height * width;

    train_square = train.square().abs().sum();

    float xx = train_square;

    float yy = train_square;

    train_fft_conj = train_fft.conjugate();
    
    xyf = train_fft * train_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline ArrayXXcf CorrelationFlow::gaussian_kernel(const ArrayXXcf& xf)
{
    unsigned int N = height * width;

    float xx = ifft(xf).square().abs().sum();

    float yy = train_square;
    
    xyf = xf * train_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline void CorrelationFlow::rotation_base(const cv::Mat& img)
{
    rot_base.clear();
    rot_base.push_back(sample);
    cv::Mat rot_mat(2, 3, CV_32FC1);
    cv::Mat img_rot;
    double scale = 1.0;
    double angle; //= -1.0*int(max_rotation/rot_resolution+1)*rot_resolution;

    for (int i=1; i<target_dim; i++)
    {
        angle = i*rot_resolution;
        rot_mat = cv::getRotationMatrix2D(cv::Point(width/2, height/2), angle, scale);
        cv::warpAffine(img, img_rot, rot_mat, img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,127);
        basis = Eigen::Map<ArrayXXf>(&img_rot.at<float>(0,0), width, height)/255.0;
        rot_base.push_back(basis);
    }
}


inline ArrayXXcf CorrelationFlow::rotation_kernel(const ArrayXXf& arr0)
{
    unsigned int N = height * width;
    
    ArrayXXf ker = ArrayXXf::Zero(target_dim, 1);

    for(int i=0; i<target_dim; i++)
    {
        float diff_square = (arr0 - rot_base.at(i)).square().abs().sum()/N;

        ker(i, 0) = -1/(sigma*sigma)*diff_square;
    }

    return fft(ker.exp());
}


inline void CorrelationFlow::scale_base(const cv::Mat& imgs)
{
    sca_base.clear();
//    sca_base.push_back(sample);
    cv::Mat sca_mat(2, 3, CV_32FC1);
    cv::Mat img_scale;

    for (int i=1; i<=sca_target_dim/2; i++)
    {
        sca_mat = cv::getRotationMatrix2D(cv::Point(width/2, height/2), 0, 0.1*i);
        cv::warpAffine(imgs, img_scale, sca_mat, imgs.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,127);
        basis_s = Eigen::Map<ArrayXXf>(&img_scale.at<float>(0,0), width, height)/255.0;
        sca_base.push_back(basis_s);
    }
}


inline ArrayXXcf CorrelationFlow::scale_kernel(const ArrayXXf& arr1)
{
    unsigned int N = height * width;
    
    ArrayXXf ker = ArrayXXf::Zero(sca_target_dim, 1);

    for(int i=0; i<sca_target_dim; i++)
    {
        if (i<sca_target_dim/4)
        {
            ker(i, 0) = 0;
        }
        else if (i>=0.75*sca_target_dim)
        {
            ker(i, 0) = 0;
        }
        else
        {
            float diff_square = (arr1 - sca_base.at(i-sca_target_dim/4)).square().abs().sum()/N;

            ker(i, 0) = exp(-1/(sigma*sigma)*diff_square);
        }

    }
    
    return fft(ker);
}

/*
inline double CorrelationFlow::rotation_estimation(const ArrayXXcf& arr1)
{
    kernel_rot = rotation_kernel(arr1);

    output_rot = ifft(kernel_rot*filter_rot_fft);


}
*/