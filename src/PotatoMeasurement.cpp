/* =========================================================== */
/*
 * @file PotatoMeasurement.cpp
 * @License MIT License. See LICENSE file in directory.
 * @brief RealSense Depth Cameraにて取得した物体の横幅、縦幅、奥行き幅を表示（単位はcm）
 * @date 08/22/24
 * @copyright (c) 2024 TOKACHI ZAIDAN.
 */
/* =========================================================== */

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "cv-helpers.hpp"

int main(int argc, char* argv[])
{
    // RealSense の初期化と開始
    const int frame_width = 640;
    const int frame_height = 480;
    const int frame_rate = 15;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_RGB8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, frame_rate);

    rs2::pipeline pipeline;
    auto profile = pipeline.start(cfg);
    float depth_scale = profile.get_device().query_sensors().front().as<rs2::depth_sensor>().get_depth_scale();

    // RGBカメラの色情報を深度カメラ用のビューポートに補正するためのフィルター
    rs2::align align_to_depth(RS2_STREAM_DEPTH);

    // 深度の閾値フィルター
    const float depth_threshold_max = 2.0f;
    const float depth_threshold_min = 0.1f;
    rs2::threshold_filter threshold_filter(depth_threshold_min, depth_threshold_max);

    while (cv::waitKey(1) < 0)
    {
        // RealSense からフレームセットを取得
        rs2::frameset frameset = pipeline.wait_for_frames();

        // RGBカメラの色情報を深度カメラ用のビューポートに補正する
        frameset = align_to_depth.process(frameset);

        // フレームを取得
        rs2::depth_frame depth_frame = frameset.get_depth_frame();
        rs2::video_frame video_frame = frameset.get_color_frame();

        // 深度の閾値フィルターを適用
        depth_frame = threshold_filter.process(depth_frame);

        // OpenCVで画像処理
        cv::Mat depth_mat = frame_to_mat(depth_frame);
        cv::Mat color_mat = frame_to_mat(video_frame);
        cv::Mat depth_median;
        cv::Mat sobel_normalized;
        cv::Mat sobel_tmp;
        cv::Mat sobel_x;
        cv::Mat sobel_y;
        cv::Mat sobel_binalized;

        cv::medianBlur(depth_mat, depth_median, 5);

        // 画像横方向の１次微分
        cv::Sobel(depth_median, sobel_tmp, CV_32F, 1, 0);
        sobel_x = cv::abs(sobel_tmp);

        // 画像縦方向の１次微分
        cv::Sobel(depth_median, sobel_tmp, CV_32F, 0, 1);
        sobel_y = cv::abs(sobel_tmp);

        // 縦方向と横方向を合成
        cv::add(sobel_x, sobel_y, sobel_tmp);
        cv::imshow("sobel", sobel_tmp);

        // [0, 1]に正規化
        cv::normalize(sobel_tmp, sobel_normalized, 0.0, 1.0, cv::NORM_MINMAX);
        cv::imshow("sobel_normalized", sobel_normalized);

        // ２値化
        cv::threshold(sobel_normalized, sobel_binalized, 0.0043, 1.0, cv::THRESH_BINARY);
        cv::imshow("sobel_binalized", sobel_binalized);

        // ２値化した画像から輪郭を取得
        sobel_binalized.convertTo(sobel_binalized, CV_8UC1, 255); // ２値化の準備として 8bit に変換
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(sobel_binalized, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // 輪郭ごとの解析と情報の表示
        for (size_t i = 0, contours_size = contours.size(); i < contours_size; ++i)
        {
            // 輪郭から矩形領域を取得
            auto& contour = contours[i];
            cv::Rect rect = cv::boundingRect(contour);

            // 矩形領域の面積が小さい場合は誤検知として無視する
            if (rect.area() < 500)
            {
                continue;
            }

            // 最大値と最小値を保持するための構造体
            struct
            {
                float min = std::numeric_limits<float>::max();
                float max = std::numeric_limits<float>::lowest();
                void expand(float v) { min = std::min(min, v); max = std::max(max, v); }
                float width() const { return max - min; }
            } x, y, z;

            // 矩形領域を走査して解析
            for (int px_y = rect.y, px_y_end = rect.y + rect.height; px_y < px_y_end; ++px_y)
            {
                for (int px_x = rect.x, px_x_end = rect.x + rect.width; px_x < px_x_end; ++px_x)
                {
                    // 深度フレームから深度値を取得
                    //float depth = depth_frame.get_distance(px_x, px_y);

                    // メディアンフィルター画像から深度値を取得
                    float depth = depth_median.at< ushort >(px_y, px_x) * depth_scale;

                    if (depth < 0.01f)
                    {
                        continue;
                    }

                    // 最大値と最小値を更新
                    z.expand(depth);
                }
            }

            // 矩形領域を走査して解析
            for (int px_y = rect.y, px_y_end = rect.y + rect.height; px_y < px_y_end; ++px_y)
            {
                for (int px_x = rect.x, px_x_end = rect.x + rect.width; px_x < px_x_end; ++px_x)
                {
                    // 深度フレームから深度値を取得
                    //float depth = depth_frame.get_distance(px_x, px_y);

                    // メディアンフィルター画像から深度値を取得
                    float depth = depth_median.at< ushort >(px_y, px_x) * depth_scale;

                    if (depth < 0.01 || z.max - z.width() * 0.3 < depth)
                    {
                        continue;
                    }

                    // 画素の座標
                    float pixel[2] = { static_cast<float>(px_x), static_cast<float>(px_y) };

                    // ３次元空間上の座標
                    float point[3];

                    // キャリブレーションデータ
                    rs2_intrinsics intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

                    // 画素から３次元空間上の座標に変換
                    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);

                    // 最大値と最小値を更新
                    x.expand(point[0]);
                    y.expand(point[1]);

                    // 深度値に応じた色づけ
                    uchar color = static_cast <uchar>(255.0f * (1.0f - (depth - z.min) / z.width()));
                    color_mat.at<cv::Vec3b>(px_y, px_x) = cv::Vec3b::all(color);
                }
            }

            // 色定数
            const auto white = cv::Scalar(255, 255, 255);
            const auto black = cv::Scalar(0, 0, 0);

            // 輪郭を描画
            cv::drawContours(color_mat, contours, static_cast<int>(i), white);

            // 矩形領域を描画
            cv::rectangle(color_mat, rect, cv::Scalar(100, 100, 200));

            // 矩形領域の幅と高さの文字列
            cv::HersheyFonts font = cv::FONT_HERSHEY_PLAIN;
            std::ostringstream stream;
            stream << std::fixed;
            stream << std::setprecision(1);
            stream << x.width() * 100 << ", " << y.width() * 100 << ", " << z.width() * 100;

            // 文字列の大きさ計算
            int base_line = 0;
            cv::Size text_size = cv::getTextSize(stream.str(), font, 0.8f, 1, &base_line);

            // 文字列の位置
            auto center = (rect.br() + rect.tl()) * 0.5;
            center.x = center.x - text_size.width / 2;
            center.y = center.y + text_size.height + rect.height / 2 + 1;

            // 文字列描画用の背景
            cv::rectangle(color_mat, cv::Rect(cv::Point(center.x, center.y - text_size.height),
                cv::Size(text_size.width, text_size.height + base_line)),
                cv::Scalar(0, 0, 0), cv::FILLED);

            // 文字列描画
            cv::putText(color_mat, stream.str(), center, font, 0.8f, white);
        }

        // 情報を Window に表示
        imshow("color_mat", color_mat);
    }

    return EXIT_SUCCESS;
}
