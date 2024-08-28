/* =========================================================== */
/*
 * @file DepthMeasurement.cpp
 * @License MIT License. See LICENSE file in directory.
 * @brief RealSense Depth Cameraから画像の中心までの深度を出力（単位はm）
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

    while (true)
    {
        // RealSense からフレームセットを取得
        rs2::frameset frameset = pipeline.wait_for_frames();

        // 深度フレームを取得
        rs2::depth_frame depth_frame = frameset.get_depth_frame();

        // 深度フレームの幅と高さを取得
        const int w = depth_frame.get_width();
        const int h = depth_frame.get_height();

        // 画像の中心の深度を取得
        float distance_center = depth_frame.get_distance(w / 2, h / 2);

        // 取得した深度を出力
        std::cout << "distance : " << distance_center << "\n";

    }

    return EXIT_SUCCESS;
}
