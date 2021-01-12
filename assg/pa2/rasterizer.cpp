// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    /*
                 A  _v[0]  {Ax, Ay, Az}
                / \
               / P \
              /(x,y)\
             B-------C  _v[2]  {Cx, Cy, Cz}
             |
            _v[1]  {Bx, By, Bz}

    ____________________C O D E_______________________
    
    Vector3f vector_A_to_B {Bx - Ax, By- Ay, Z};
    Vector3f vector_A_to_P {x - _Ax, y - Ay, Z};
    Vector3f vector_B_to_C {Cx - Bx, ...};
    ...
    ...
                                                            | take Z value
    auto result_ABxAP = (vector_A_to_B   X   vector_A_to_P)[2];
    auto result_BCxBP = (vector_B_to_C   X   vector_B_to_P)[2];
    auto result_CAxCP = (vector_C_to_A   X   vector_C_to_P)[2];
    
    return (all positive or all negative)
    */
    Vector3f vector_A_to_B {_v[1][0] - _v[0][0], _v[1][1] - _v[0][1], _v[0][2]};
    Vector3f vector_A_to_P {x - _v[0][0], y - _v[0][1], _v[0][2]};
    Vector3f vector_B_to_C {_v[2][0] - _v[1][0], _v[2][1] - _v[1][1], _v[0][2]};
    Vector3f vector_B_to_P {x - _v[1][0], y - _v[1][1], _v[0][2]};
    Vector3f vector_C_to_A {_v[0][0] - _v[2][0], _v[0][1] - _v[2][1], _v[0][2]};
    Vector3f vector_C_to_P {x - _v[2][0], y - _v[2][1], _v[0][2]};
    // since on same z-plane, we take the z value to determine relative position
    auto result_ABxAP = vector_A_to_B.cross(vector_A_to_P)[2];
    auto result_BCxBP = vector_B_to_C.cross(vector_B_to_P)[2];
    auto result_CAxCP = vector_C_to_A.cross(vector_C_to_P)[2];
    // check if all positive(left) or negative(right)
    return ((result_ABxAP < 0 && result_BCxBP < 0 && result_CAxCP < 0) || 
       (result_ABxAP > 0 && result_BCxBP > 0 && result_CAxCP > 0));
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    /* YH: change from 3d coordinates to screen coordinates
       for example, TRIANGLE 1  
        2  to  141.928
        0  to  350
       -2  to -38009.4
        */

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
                
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    
    auto v = t.toVector4();
    int  index = 0;
    bool flag = true;
    int  depth_index, count;
    float sample_x, sample_y;
    // define screen area
    constexpr int max_width  {700};
    constexpr int max_height {700};
    constexpr int super_sample_pixel {4};
    constexpr float z_far_end = -9999;

    // initialize buffers
    int pixel_amount =  max_width * max_height;

    static std::vector<Eigen::Vector3f> color_buf(pixel_amount, Vector3f (0,0,0));

    // depth buffers initialized with negative infinites
    static std::vector<float> depth_buf(super_sample_pixel * pixel_amount, z_far_end);
    /*  y
        |2 ->
        |1 ->
        |0 ->
        ---------> x
    */
    // iterate through every pixel
    for (float y {0}; y < max_width; ++y)
    {
        for (float x {0}; x < max_height; ++x)
        {
            index = y * max_width + x;
            count = 0;
            // apply super sampling inside each pixel
            for (size_t p = 0; p < super_sample_pixel; p++)
            {
                // sub-pixel coordinates
                // WATCH OUT TYPES!
                sample_x = x + 1/(float)super_sample_pixel + (p % 2) * 2 / (float)super_sample_pixel;
                sample_y = y + 1/(float)super_sample_pixel + (p / 2) * 2 / (float)super_sample_pixel;
                
                depth_index = index * super_sample_pixel + p;

                if (insideTriangle(sample_x, sample_y, t.v))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(sample_x, sample_y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // if the current sample should be draw on top of the former sample (or default)
                    if (z_interpolated > depth_buf[depth_index])
                    {
                        depth_buf[depth_index] = z_interpolated;
                        color_buf[index] += t.getColor() / 4;
                    }
                }
            }
            set_pixel(Vector3f(x, y, 1), color_buf[index]);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on