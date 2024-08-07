// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.


    double n = -zNear, f = -zFar;
    double height = 2 * zNear * tan(eye_fov / 2 / 180.0 * MY_PI);
    double width = height * aspect_ratio;
    double l = -width / 2, r = width / 2, b = -height / 2, t = height / 2;

    Eigen::Matrix4f translate;
    translate << 2 * n / (r - l), 0, 0, 0,
            0, 2 * n / (t - b), 0, 0,
            0, 0, (f + n) / (n - f), 2 * f * n / (f - n),
            0, 0, 1, 0;
    projection = translate * projection;


    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = true;
    // 选择想要实现的反走样模式
    Mode mode = Mode::SSAA;
//    Mode mode = Mode::MSAA1;  // 有黑边
//    Mode mode = Mode::MSAA2;  // 无黑边
//    Mode mode = Mode::None;
    std::string filename = "output_" + modeToString(mode) + ".png";

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.set_mode(mode);

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);


        cv::imwrite(filename, image);
        std::cout << "Saved " << filename << '\n';

        return 0;
    }

    // while(key != 27)
    // {
    //     r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    //     r.set_model(get_model_matrix(angle));
    //     r.set_view(get_view_matrix(eye_pos));
    //     r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    //     r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

    //     cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    //     image.convertTo(image, CV_8UC3, 1.0f);
    //     cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    //     cv::imshow("image", image);
    //     key = cv::waitKey(5);

    //     std::cout << "frame count: " << frame_count++ << '\n';
    // }

    return 0;
}

// clang-format on