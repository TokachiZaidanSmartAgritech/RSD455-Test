/* =========================================================== */
/*
 * @file ColorizerChange.cpp
 * @License MIT License. See LICENSE file in directory.
 * @brief DepthColorView.cppで表示した画像の深度の色付けを変更した画像
 * @date 08/22/24
 * @copyright (c) 2024 TOKACHI ZAIDAN.
 */
/* =========================================================== */

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

int main()
{
    // RealSense の初期化と開始
    rs2::pipeline pipeline;
    pipeline.start();

    // Windowの初期化
    window window(1280, 720, "RealSense Capture Example");
    while (window)
    {
        // RealSense からフレームセットを取得
        rs2::frameset frameset = pipeline.wait_for_frames();

        // 深度フレームを取得
        rs2::depth_frame depth_frame = frameset.get_depth_frame();

        // 深度フレームの幅と高さを取得
        const int w = depth_frame.get_width();
        const int h = depth_frame.get_height();

        // 画像の中心の深度を取得
//        float distance_center = depth_frame.get_distance(w / 2, h / 2);

        // 取得した深度を出力
//        std::cout << "distance_center : " << distance_center << "\n";

        // 閾値フィルター
        rs2::threshold_filter threshold_filter(0.15f, 2.5f);
        depth_frame = threshold_filter.process(depth_frame);

        // 色付けフィルターを設定
        rs2::colorizer colorizer;
        colorizer.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        colorizer.set_option(RS2_OPTION_MAX_DISTANCE, 2.5f);
        rs2::video_frame video_frame = colorizer.colorize(depth_frame);

        // フィルターの適用結果を Window に表示
        window.show(video_frame);
    }

    return EXIT_SUCCESS;
}
