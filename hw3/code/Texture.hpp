//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        
        // cannot find neighbouring texel
        if (u_img < 0.5 || v_img < 0.5 || width - u_img < 0.5 || height - v_img < 0.5) {
            auto color = image_data.at<cv::Vec3b>(v_img, u_img);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }

        Eigen::Vector2f u00, u01, u10, u11;
        float delta_u = u_img - int(u_img);
        float delta_v = v_img - int(v_img);
        // find 4 nearest sample locations
        if (delta_u < 0.5) {
            if (delta_v < 0.5) u00 = Eigen::Vector2f(int(u_img) - 0.5, int(v_img) - 0.5);
            else u00 = Eigen::Vector2f(int(u_img) - 0.5, int(v_img));
        }
        else {
            if (delta_v < 0.5) u00 = Eigen::Vector2f(int(u_img), int(v_img) - 0.5);
            else u00 = Eigen::Vector2f(int(u_img), int(v_img));
        }
        u01 = u00 + Eigen::Vector2f(0.0f, 1.0f);
        u10 = u00 + Eigen::Vector2f(1.0f, 0.0f);
        u11 = u00 + Eigen::Vector2f(1.0f, 1.0f);

        // get color of nearest 4 sample locations
        float s = u_img - u00.x();
        float t = v_img - u00.y();
        auto c1 = image_data.at<cv::Vec3b>(u00.y(), u00.x());
        auto c2 = image_data.at<cv::Vec3b>(u10.y(), u10.x());
        auto c3 = image_data.at<cv::Vec3b>(u01.y(), u01.x());
        auto c4 = image_data.at<cv::Vec3b>(u11.y(), u11.x());
        Eigen::Vector3f color1 = Eigen::Vector3f(c1[0], c1[1], c1[2]);
        Eigen::Vector3f color2 = Eigen::Vector3f(c2[0], c2[1], c2[2]);
        Eigen::Vector3f color3 = Eigen::Vector3f(c3[0], c3[1], c3[2]);
        Eigen::Vector3f color4 = Eigen::Vector3f(c4[0], c4[1], c4[2]);
        
        // bilinear interpolation
        auto color_u0 = color1 + s * (color2 - color1);
        auto color_u1 = color3 + s * (color4 - color3);
        return color_u0 + t * (color_u1 - color_u0); 
    }

};
#endif //RASTERIZER_TEXTURE_H
