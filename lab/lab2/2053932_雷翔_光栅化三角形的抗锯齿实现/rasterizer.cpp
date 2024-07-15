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

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

double cross(double v1[2], double v2[2])
{
    return v1[0] * v2[1] - v1[1] * v2[0];
}


/**
 * @brief 判断一个二维点 (x, y) 是否在由三个三维顶点 _v[0], _v[1], _v[2] 定义的三角形内部
 * 只考虑三维坐标的x和y，z为深度，无需考虑
 * 
 * @param x 
 * @param y 
 * @param _v 
 * @return true 
 * @return false 
 */
static bool insideTriangle(double x, double y, const Eigen::Vector4f *_v)
{
    // // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    double v_ab[2], v_bc[2], v_ca[2];
    double v_ap[2], v_bp[2], v_cp[2];

    v_ab[0] = _v[1].x() - _v[0].x();
    v_ab[1] = _v[1].y() - _v[0].y();
    v_bc[0] = _v[2].x() - _v[1].x();
    v_bc[1] = _v[2].y() - _v[1].y();
    v_ca[0] = _v[0].x() - _v[2].x();
    v_ca[1] = _v[0].y() - _v[2].y();

    v_ap[0] = x - _v[0].x();
    v_ap[1] = y - _v[0].y();
    v_bp[0] = x - _v[1].x();
    v_bp[1] = y - _v[1].y();
    v_cp[0] = x - _v[2].x();
    v_cp[1] = y - _v[2].y();


    bool dir1 = cross(v_ap, v_ab) >= 0;
    bool dir2 = cross(v_bp, v_bc) >= 0;
    bool dir3 = cross(v_cp, v_ca) >= 0;

    return (dir1 == dir2 && dir2 == dir3);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
               (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() -
                v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
               (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() -
                v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
               (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() -
                v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    std::cout << "选择\"" << modeToString(mode) << "\"采样模式" << std::endl;
    for (auto &i: ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        // Homogeneous division
        for (auto &vec: v)
        {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto &vert: v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = -vert.z() * f1 + f2;
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

        // 超采样
        if (mode == Mode::SSAA)
            rasterize_triangle_with_ssaa(t);
        else if (mode == Mode::MSAA1)
            rasterize_triangle_with_msaa1(t);
        else if (mode == Mode::MSAA2)
            rasterize_triangle_with_msaa2(t);
        else
            rasterize_triangle_with_sample(t);
    }

    // 对于 SSAA，最后再给像素赋值
    if (mode == Mode::SSAA)
        rasterize_triangle_with_pixel();
}

double getInterpolatedZ(double x, double y, const Triangle &t)
{
    // 获取矫正深度的函数的错误原因
    // 这里的 v 是经过透视除法的，所以 w = 1
    // 即空间深度默认是 1，这在渲染远处的物体时会出现问题

    auto v = t.toVector4();

    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;

    return z_interpolated;
}

Eigen::Vector3f getAverageColor(rst::sample_color_list scl)
{
    Eigen::Vector3f color1, color2, color3, color4;
    color1 = scl.s1_color;
    color2 = scl.s2_color;
    color3 = scl.s3_color;
    color4 = scl.s4_color;

    double r = (color1.x() + color2.x() + color3.x() + color4.x()) / 4.0;
    double g = (color1.y() + color2.y() + color3.y() + color4.y()) / 4.0;
    double b = (color1.z() + color2.z() + color3.z() + color4.z()) / 4.0;
    return Eigen::Vector3f(r, g, b);
}

void rst::rasterizer::rasterize_triangle_with_pixel()
{
    // 4x 下采样
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            Eigen::Vector3f avg_color = Eigen::Vector3f(0, 0, 0);
            double avg_depth = 0.0;
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    auto ind = (2 * y + j) * width * 2 + (2 * x + i);

                    // 计算颜色平均值
                    avg_color += frame_buf_4x[ind];
                    // 计算深度平均值
                    avg_depth += depth_buf_4x[ind];
                }
            }
            avg_color /= 4.0;
            avg_depth /= 4.0;
            set_pixel(Eigen::Vector3f(x, y, 0), avg_color);
        }
    }
}

// Screen space rasterization
/**
 * @brief 对屏幕空间中的三角形进行光栅化，光栅化是指将几何图形（如三角形）转换为屏幕像素的过程
 * 
 * @param t 
 */
void rst::rasterizer::rasterize_triangle_with_sample(const Triangle &t)
{
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // 1. get min{x,y}, max{x,y}.
    // 计算三角形的边界框
    int min_x, max_x, min_y, max_y;
    min_x = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    max_x = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    min_y = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    max_y = std::max(std::max(v[0].y(), v[1].y()), v[2].y());


    // 2. iterate every pixel and check if the current pixel is inside the triangle.
    //    If so, use the 'getInterpolatedZ' function to get the interpolated z value.
    //    Finally, set the current pixel (use the 'set_pixel' function) to the color of the triangle (use 'getColor' function) if it should be painted.
    for (int x = min_x; x <= max_x; x++)
    {
        for (int y = min_y; y <= max_y; y++)
        {
            // insideTriangle：检查一个像素是否在三角形内
            // +0.5 是为了将坐标偏移到像素中心，参考：https://zhuanlan.zhihu.com/p/161457977
            if (insideTriangle(x + 0.5, y + 0.5, v.data()))
            {
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);  // 计算重心坐标
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());  // 计算重心坐标的倒数
                float z_interpolated =
                        alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() +
                        gamma * v[2].z() / v[2].w();   // 计算插值后的z值
                z_interpolated *= w_reciprocal;  // 乘以重心坐标的倒数

                auto ind = (height - 1 - y) * width + x;  // 计算像素索引
                if (depth_buf[ind] < z_interpolated) // 深度测试
                    continue;

                // else depth_buf[ind] >= z_interpolated 
                depth_buf[ind] = z_interpolated;

                // t.getColor()：获取三角形的颜色 (Only one color per triangle.)
                set_pixel(Eigen::Vector3f(x, y, z_interpolated), t.getColor());  // 设置像素颜色
            }
        }
    }
}

/**
 * SSAA v.s. MSAA
 * 简单地说，SSAA需要为像素内的每个子采样点调用一次getColor()，而MSAA只需要整个像素调用一次。
 */

/**
 * @brief 使用 SSAA 超采样反走样(Super Sampling AA) 对屏幕空间中的三角形进行光栅化
 * 对每个原始像素内部的多个子像素进行采样，然后计算这些子像素的颜色平均值作为原始像素的颜色
 * 
 * @param t 
 */
void rst::rasterizer::rasterize_triangle_with_ssaa(const Triangle &t)
{
    auto v = t.toVector4();

    int min_x, max_x, min_y, max_y;
    min_x = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    max_x = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    min_y = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    max_y = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    // 4 times buffer 4 倍大小的缓冲区
    // eg. 800 x 600 -> 1600 x 1200
    // 即先超采样，然后下采样
    int scale_factor = 2;
    int num_samples = scale_factor * scale_factor;

    // 4x 超采样
    for (int x = min_x; x < max_x; x++)
    {
        for (int y = min_y; y < max_y; y++)
        {
            // 对每个像素内部的子像素进行采样
            for (int i = 0; i < scale_factor; i++)
            {
                for (int j = 0; j < scale_factor; j++)
                {
                    // new_x, new_y 类型！！！！！！！！！！！！！
                    double new_x = x + (i + 0.5) * 1.0 / 2.0;
                    double new_y = y + (j + 0.5) * 1.0 / 2.0;

                    // insideTriangle：检查一个像素是否在三角形内
                    if (insideTriangle(new_x, new_y, v.data()))
                    {
                        float z_interpolated = getInterpolatedZ(new_x, new_y, t);

                        // 这里的顺序要注意！！！（纠结好久
                        // 对 x = 0 y = 0
                        // i = 0, j = 0 -> 索引 0
                        // i = 0, j = 1 -> 索引 width * 2
                        // i = 1, j = 0 -> 索引 1
                        // i = 1, j = 1 -> 索引 width * 2 + 1

                        // 4 buffer
                        // 列索引：(2 * x + i)
                        // 行索引：(2 * y + j)
                        auto ind = (2 * y + j) * width * 2 + (2 * x + i);
                        if (depth_buf_4x[ind] < z_interpolated) // 深度测试
                            continue;

                        depth_buf_4x[ind] = z_interpolated;
                        frame_buf_4x[ind] = t.getColor();
                    }
                }
            }
        }
    }
}


/**
 * @brief 使用 MSAA 多采样反走样(Multi-Sampling AA) 对屏幕空间中的三角形进行光栅化
 * 只计算有几个采样点会被三角形覆盖，利用像素中心坐标计算一次颜色
 * 【有黑线版本】
 * @param t 
 */
void rst::rasterizer::rasterize_triangle_with_msaa1(const Triangle &t)
{
    auto v = t.toVector4();

    int min_x, max_x, min_y, max_y;
    min_x = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    max_x = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    min_y = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    max_y = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    int sample_num = 4;  // 采样点数量
    std::vector<Eigen::Vector2f> sample_positions = {{0.25, 0.25},
                                                     {0.75, 0.25},
                                                     {0.25, 0.75},
                                                     {0.75, 0.75}};

    for (int x = min_x; x <= max_x; x++)
    {
        for (int y = min_y; y <= max_y; y++)
        {
            int count = 0;
            for (int i = 0; i < sample_num; i++)
            {
                float sub_x = x + sample_positions[i].x();
                float sub_y = y + sample_positions[i].y();

                // 检查子像素是否在三角形内
                if (insideTriangle(sub_x, sub_y, v.data()))
                {
                    count++;
                }
            }

            if (insideTriangle(x + 0.5, y + 0.5, v.data()))
            {
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated =
                        alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                auto ind = (height - 1 - y) * width + x;
                if (depth_buf[ind] < z_interpolated) // 深度测试
                    continue;

                // else depth_buf[ind] >= z_interpolated
                depth_buf[ind] = z_interpolated;
                set_pixel(Eigen::Vector3f(x, y, z_interpolated),
                          t.getColor() * ((double) count / sample_num));  // 一定要记得转换成 double 再相除！！！
            }
        }
    }
}

/**
 * @brief 使用 MSAA 多采样反走样(Multi-Sampling AA) 对屏幕空间中的三角形进行光栅化
 * 只计算有几个采样点会被三角形覆盖，利用像素中心坐标计算一次颜色
 * 【无黑线版本】：采用平均思想，将每个像素点的颜色值设置为所有采样点的颜色值的平均值
 * @param t
 */
void rst::rasterizer::rasterize_triangle_with_msaa2(const Triangle &t)
{
    auto v = t.toVector4();

    int min_x, max_x, min_y, max_y;
    min_x = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    max_x = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    min_y = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    max_y = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    // 4 times buffer 4 倍大小的缓冲区
    // eg. 800 x 600 -> 1600 x 1200
    // 即先超采样，然后下采样
    int scale_factor = 2;
    int num_samples = scale_factor * scale_factor;

    // 4x 超采样
    for (int x = min_x; x < max_x; x++)
    {
        for (int y = min_y; y < max_y; y++)
        {
            int count = 0;
            // 对每个像素内部的子像素进行采样
            for (int i = 0; i < scale_factor; i++)
            {
                for (int j = 0; j < scale_factor; j++)
                {
                    // new_x, new_y 类型！！！！！！！！！！！！！
                    double new_x = x + (i + 0.5) * 1.0 / 2.0;
                    double new_y = y + (j + 0.5) * 1.0 / 2.0;

                    // insideTriangle：检查一个像素是否在三角形内
                    if (insideTriangle(new_x, new_y, v.data()))
                    {
                        float z_interpolated = getInterpolatedZ(new_x, new_y, t);

                        auto ind = (2 * y + j) * width * 2 + (2 * x + i);
                        if (depth_buf_4x[ind] < z_interpolated) // 深度测试
                            continue;

                        depth_buf_4x[ind] = z_interpolated;
                        frame_buf_4x[ind] = t.getColor();

                        count++;
                    }
                }
            }
            // 之所以前面方法有黑边，是因为对三角形光栅化时时仅仅考虑了背景色和该三角形颜色，没有考虑其他三角形的颜色
            Eigen::Vector3f color = {0, 0, 0};
            if (count > 0)
            {
                Eigen::Vector3f color1 = frame_buf_4x[(2 * y) * width * 2 + (2 * x)];
                Eigen::Vector3f color2 = frame_buf_4x[(2 * y) * width * 2 + (2 * x + 1)];
                Eigen::Vector3f color3 = frame_buf_4x[(2 * y + 1) * width * 2 + (2 * x)];
                Eigen::Vector3f color4 = frame_buf_4x[(2 * y + 1) * width * 2 + (2 * x + 1)];

                color = (color1 + color2 + color3 + color4) / 4.0; // 一定要记得转换成 double 再相除！！！
                set_pixel(Eigen::Vector3f(x, y, 0), color);
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
{
    projection = p;
}

void rst::rasterizer::set_mode(Mode aa_mode)
{
    mode = aa_mode;
}

std::string modeToString(Mode mode)
{
    switch (mode)
    {
        case Mode::None:
            return "None";
        case Mode::MSAA1:
            return "MSAA 有黑边";
        case Mode::MSAA2:
            return "MSAA 无黑边";
        case Mode::SSAA:
            return "SSAA";
        default:
            return "Unknown";
    }
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_4x.begin(), frame_buf_4x.end(), Eigen::Vector3f{0, 0, 0});

        for (auto &it: sampling_color_buf)
        {
            it.s1_color = Eigen::Vector3f{0, 0, 0};
            it.s2_color = Eigen::Vector3f{0, 0, 0};
            it.s3_color = Eigen::Vector3f{0, 0, 0};
            it.s4_color = Eigen::Vector3f{0, 0, 0};
        }

    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_4x.begin(), depth_buf_4x.end(), std::numeric_limits<float>::infinity());

        for (auto &it: sampling_depth_buf)
        {
            it.s1_depth = std::numeric_limits<float>::infinity();
            it.s2_depth = std::numeric_limits<float>::infinity();
            it.s3_depth = std::numeric_limits<float>::infinity();
            it.s4_depth = std::numeric_limits<float>::infinity();
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_4x.resize(w * h * 4);
    depth_buf_4x.resize(w * h * 4);

    sampling_color_buf.resize(w * h);
    sampling_depth_buf.resize(w * h);

}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// clang-format on