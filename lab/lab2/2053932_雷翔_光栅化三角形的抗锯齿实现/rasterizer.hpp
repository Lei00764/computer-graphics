//
// Created by goksu on 4/6/19.
//

#pragma once

#include <algorithm>
#include <eigen3/Eigen/Eigen>

#include "Triangle.hpp"
#include "global.hpp"

using namespace Eigen;

enum class Mode
{
    None,
    MSAA1,  // 多采样反走样 有黑边
    MSAA2,  // 多采样反走样 无黑边
    SSAA   // 超采样反走样
};

std::string modeToString(Mode mode);  // 将采样模式转换为字符串

namespace rst
{
    enum class Buffers
    {
        Color = 1, Depth = 2
    };

    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int) a | (int) b);
    }

    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int) a & (int) b);
    }

    enum class Primitive
    {
        Line, Triangle
    };

/*
 * For the curious : The draw function takes two buffer id's as its arguments.
 * These two structs make sure that if you mix up with their orders, the
 * compiler won't compile it. Aka : Type safety
 * */
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    struct ind_buf_id
    {
        int ind_id = 0;
    };

    struct col_buf_id
    {
        int col_id = 0;
    };

//////////////////////////////////////
// my definition: 1 pixel of 4 sample
    struct sample_depth_list
    {
        double s1_depth, s2_depth, s3_depth, s4_depth;
    };

    struct sample_color_list
    {
        Vector3f s1_color, s2_color, s3_color, s4_color;
    };
//////////////////////////////////////

    class rasterizer
    {
    public:
        rasterizer(int w, int h);

        pos_buf_id load_positions(const std::vector<Eigen::Vector3f> &positions);

        ind_buf_id load_indices(const std::vector<Eigen::Vector3i> &indices);

        col_buf_id load_colors(const std::vector<Eigen::Vector3f> &colors);

        void set_model(const Eigen::Matrix4f &m);

        void set_view(const Eigen::Matrix4f &v);

        void set_projection(const Eigen::Matrix4f &p);

        void set_mode(Mode mode);  // 设置采样模式

        void set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color);

        void clear(Buffers buff);

        void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer,
                  Primitive type);

        std::vector<Eigen::Vector3f> &frame_buffer() { return frame_buf; }

    private:
        void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);

        void rasterize_triangle_with_sample(const Triangle &t);

        void rasterize_triangle_with_ssaa(const Triangle &t);

        void rasterize_triangle_with_msaa1(const Triangle &t);  // 有黑边

        void rasterize_triangle_with_msaa2(const Triangle &t);  // 无黑边

        void rasterize_triangle_with_pixel();

        // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI ->
        // FRAGSHADER

    private:
        Eigen::Matrix4f model;
        Eigen::Matrix4f view;
        Eigen::Matrix4f projection;
        Mode mode;

        std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
        std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
        std::map<int, std::vector<Eigen::Vector3f>> col_buf;

        std::vector<Eigen::Vector3f> frame_buf;

        std::vector<float> depth_buf;

//        // add new property
        std::vector<sample_depth_list> sampling_depth_buf;
        std::vector<sample_color_list> sampling_color_buf;

        // 存储超采样后的颜色
        std::vector<Eigen::Vector3f> frame_buf_4x;
        // 存储超采样后的深度
        std::vector<float> depth_buf_4x;

        int get_index(int x, int y);

        int width, height;

        int next_id = 0;

        int get_next_id() { return next_id++; }
    };
}  // namespace rst
