#pragma once

#include "tetris.hpp"

cv::Mat kmeansImageClustering(cv::Mat& img, int cluster_count);
void drawRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness = 1);
inline cv::Mat invertAffine(const cv::Mat& affineMat);

class Detector {
public:
    Detector() = default;
    bool detect(const cv::Mat& img);
    int totalTetrisCnt { 0 };                                  // 检测到的方块总数
    std::unordered_map<COLORS, std::vector<tetris>> tetrisSet; // 存储所有检测到的方块

private:
    cv::Mat m_imageRaw;    // 原图
    cv::Mat m_imageBinary; // 二值图
    cv::Mat m_image;       // 处理后图片
    void preprocess(const cv::Mat& img);
    void setCodeMat(tetris& Tetris);
    void getCodeMatValue(tetris& Tetris);
    void getSuctionPosition(tetris& Tetris);
    void findTetris();
};