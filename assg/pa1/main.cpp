#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;
    // translate view back to origin point
    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(Eigen::Vector4f axis, float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    rotation_angle = (rotation_angle / 180) * MY_PI;

    // Apply the Rodrigues Rotation Formula
    
    float x_axis = axis(0);
    float y_axis = axis(1);
    float z_axis = axis(2);

    Eigen::Matrix4f rodrigues_N = Eigen::Matrix4f::Identity();
    rodrigues_N << 0,       -z_axis,   y_axis, 0,
                   z_axis,   0,       -x_axis, 0,
                  -y_axis,   x_axis,   0,      0,
                   0,        0,        0,      1;
    
    model << cos(rotation_angle) * Eigen::Matrix4f::Identity() + 
             (1 - cos(rotation_angle)) * axis * axis.transpose() +
             sin(rotation_angle) * rodrigues_N;
             
    // make sure the last element is 1
    // this could be done more elegantly by initially making a 3x3 matrix, then extends it to 4x4
    // but this is life         
    model(3,3) = 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
    /*  Create then return the projection matrix

                     second     first
        ProjMatrix : Mortho * Mpers->ortho 
        Output     : Canonical Cube 
    */
    
    Eigen::Matrix4f perspective = Eigen::Matrix4f::Identity();

    perspective <<  zNear,  0,        0,              0,
                    0,      zNear,    0,              0,
                    0,      0,        zNear + zFar,   -(zNear * zFar),
                    0,      0,        1,              0;
    
    // calculate the sizes of the frustum
    eye_fov         = (eye_fov / 180) * MY_PI;
    float yTop      =  abs(zNear) * tan(eye_fov / 2);
    float yBottom   = -yTop;
    float xRight    =  yTop * aspect_ratio;
    float xLeft     = -xRight;
    
    // calculate the ortho projection matrix
    Eigen::Matrix4f ortho;
    ortho << 2 / (xRight - xLeft), 0,                    0,                  0,
             0,                    2 / (yTop - yBottom), 0,                  0,
             0,                    0,                    2 / (zNear - zFar), 0,
             0,                    0,                    0,                  1;

    Eigen::Matrix4f trans;
    trans << 1,    0,    0,     -(xRight + xLeft) / 2,
             0,    1,    0,     -(yTop + yBottom) / 2,
             0,    0,    1,     -(zNear + zFar)   / 2,
             0,    0,    0,      1;

    /* 1. perspective: do perspective projection to "shrink the projection" down to the "screen" size
       2. trans:       move the prohection to the "screen" direction
       3. ortho:       actually project the image to the display plane 
    */          
    return  ortho * trans * perspective;
    
}
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    Eigen::Vector4f axis(0.0f, 0.0f, 1.0f, 0.0f);

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        //std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'x') {
            axis[0] = 1.0f;
            axis[1] = 0.0f;
            axis[2] = 0.0f;
        }
        else if (key == 'y') {
            axis[0] = 0.0f;
            axis[1] = 1.0f;
            axis[2] = 0.0f;
        }
        else if (key == 'z') {
            axis[0] = 0.0f;
            axis[1] = 0.0f;
            axis[2] = 1.0f;
        }
    }

    return 0;
}
