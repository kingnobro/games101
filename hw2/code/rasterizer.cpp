// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <float.h>


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

auto to_vec3(const Eigen::Vector4f& v4)
{
    return Vector3f(v4.x(), v4.y(), v4.z());
}

auto to_vec2(const Eigen::Vector3f& v3)
{
    return Vector2f(v3.x(), v3.y());
}

float cross_product(const Eigen::Vector2f& A, const Eigen::Vector2f& B)
{
    return A.x() * B.y() - A.y() * B.x();
}


static bool insideTriangle(float x, float y, const Vector3f* v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f O(x, y);
    Vector2f A = to_vec2(v[0]);
    Vector2f B = to_vec2(v[1]);
    Vector2f C = to_vec2(v[2]);
    Vector2f AB = B - A;
    Vector2f BC = C - B;
    Vector2f CA = A - C;
    Vector2f AO = O - A;
    Vector2f BO = O - B;
    Vector2f CO = O - C;

    float val1 = cross_product(AB, AO);
    float val2 = cross_product(BC, BO);
    float val3 = cross_product(CA, CO);

    return (val1 > 0 && val2 > 0 && val3 > 0) || (val1 < 0 && val2 < 0 && val3 < 0);
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
    Vector3f _v[3] = {
        to_vec3(v[0]),
        to_vec3(v[1]),
        to_vec3(v[2])
    };
    
    // Find out the bounding box of current triangle.
    float minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;
    for (const auto &vertex : v)
    {
        float x = vertex.x(), y = vertex.y();
        if (x > maxX) maxX = x;
        if (x < minX) minX = x;
        if (y > maxY) maxY = y;
        if (y < minY) minY = y;
    }
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int baseX = minX; baseX < maxX; baseX += 1) {
        for (int baseY = minY; baseY < maxY; baseY += 1) {
            // divide each pixel into 2*2 for super-sampling
            int subPixelInsideTri = 0;
            for (int offsetX = 0; offsetX <= 1; offsetX += 1) {
                for (int offsetY = 0; offsetY <= 1; offsetY += 1) {
                    float x = baseX + offsetX * 0.5;
                    float y = baseY + offsetY * 0.5;

                    if (insideTriangle(x, y, _v)) {
                        // If so, use the following code to get the interpolated z value.
                        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                        int index = offsetX * 2 + offsetY;
                        if (z_interpolated < depth_buf[x * width + y][index]) {
                            depth_buf[x * width + y][index] = z_interpolated;
                            subPixelInsideTri += 1;
                        }
                    }
                }
            }
            if (subPixelInsideTri > 0) set_pixel(Vector3f(baseX, baseY, 0.0f), subPixelInsideTri / 4.0f * t.getColor());
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
        std::array<float, 4> pixelDepth;
        pixelDepth.fill(std::numeric_limits<float>::infinity());
        std::fill(depth_buf.begin(), depth_buf.end(), pixelDepth);
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