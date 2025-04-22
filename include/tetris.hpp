#pragma once

#include <array>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// Tetris colors
enum class COLORS {
    YELLOW,
    PURPLE,
    GREEN,
    BLUE,
    ORANGE,
    BROWN,
    RED,
    WHITE
};

// phases
enum class PHASES {
    FIRST,
    SECOND
};

// params
struct Param {
    Param() = default;
    Param(const std::string& filename);

    void load(const std::string& filename);

    const inline static cv::Scalar red { 0, 0, 255 };
    const inline static cv::Scalar blue { 255, 0, 0 };
    const inline static cv::Scalar green { 0, 255, 0 };
    const inline static cv::Scalar white { 255, 255, 255 };
    const inline static cv::Scalar yellow { 0, 255, 255 };
    const inline static cv::Scalar purple { 128, 0, 128 };

    // base 存放不同编码值对应的方块类型以及是否需要翻转
    constexpr static std::array<std::tuple<int, COLORS, bool>, 8> base { { { 39, COLORS::PURPLE, false },
                                                                           { 57, COLORS::PURPLE, true },
                                                                           { 15, COLORS::YELLOW, false },
                                                                           { 60, COLORS::YELLOW, true },
                                                                           { 51, COLORS::BLUE, false },
                                                                           { 30, COLORS::GREEN, false },
                                                                           { 23, COLORS::BROWN, false },
                                                                           { 58, COLORS::BROWN, true } } };

    // posVector 存放不同方块类型的抓取点在codeMat中的坐标
    const static inline cv::Mat posVectorPurple { cv::Mat(cv::Vec3d(50, 150, 1), CV_64F) };
    const static inline cv::Mat posVectorPurpleUpdown { cv::Mat(cv::Vec3d(250, 50, 1), CV_64F) };
    const static inline cv::Mat posVectorYellow { cv::Mat(cv::Vec3d(250, 150, 1), CV_64F) };
    const static inline cv::Mat posVectorYellowUpdown { cv::Mat(cv::Vec3d(50, 50, 1), CV_64F) };
    const static inline cv::Mat posVectorBlue { cv::Mat(cv::Vec3d(150, 50, 1), CV_64F) };
    const static inline cv::Mat posVectorGreen { cv::Mat(cv::Vec3d(150, 50, 1), CV_64F) };
    const static inline cv::Mat posVectorOrange { cv::Mat(cv::Vec3d(50, 50, 1), CV_64F) };
    const static inline cv::Mat posVectorBrown { cv::Mat(cv::Vec3d(150, 150, 1), CV_64F) };
    const static inline cv::Mat posVectorBrownUpdown { cv::Mat(cv::Vec3d(150, 50, 1), CV_64F) };
    const static inline cv::Mat posVectorRed { cv::Mat(cv::Vec3d(150, 50, 1), CV_64F) };

    const static inline std::unordered_map<COLORS, std::string> colorTostring { { { COLORS::PURPLE, "PURPLE" },
                                                                                  { COLORS::GREEN, "GREEN" },
                                                                                  { COLORS::BLUE, "BLUE" },
                                                                                  { COLORS::ORANGE, "ORANGE" },
                                                                                  { COLORS::YELLOW, "YELLOW" },
                                                                                  { COLORS::RED, "RED" },
                                                                                  { COLORS::BROWN, "BROWN" },
                                                                                  { COLORS::WHITE, "WHITE" } } };

    inline static float threshold;
    inline static float min_Rotatedrect_Area;
    inline static float max_Rotatedrect_Area;

    inline static PHASES phase;
};

/**
 * @brief 存储方块信息
 */
struct tetris {
    tetris() = default;
    tetris(const cv::RotatedRect& rect, COLORS type);
    cv::RotatedRect rotatedrect; // 旋转矩形
    COLORS type;                 // 方块类别
    float height;                // 旋转矩形的高
    float width;                 // 旋转矩形的宽
    cv::Mat codeMat;             // 编码图
    cv::Mat toCodeMat;           // 从原图变换到编码图的变换矩阵
    double angleToHorizontal;    // codeMat旋转到水平的角度(弧度,顺时针为正)
    bool updown;                 // 是否上下翻转来达到标准状态
    int codeMatValue;            // 编码值
    cv::Point suctionPosition;   // 抓取点
};

// Tetris operating command
struct commandType {
    commandType() = default;
    commandType(int x, int y);
    int x, y;      // 放置点坐标(左上角为原点)
    tetris Tetris; // 方块
    double angle;  // 旋转到目标状态的角度(弧度,顺时针为正)
};