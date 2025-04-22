#include "../include/tetris.hpp"

Param::Param(const std::string& filename) {
    load(filename);
}

void Param::load(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    int fsphase;
    fs["phase"] >> fsphase;
    phase = (fsphase == 1 ? PHASES::FIRST : PHASES::SECOND);
    auto fsdetect = fs["detect"];
    auto fstetris = fsdetect["tetris"];
    fstetris["threshold"] >> threshold;
    fstetris["minRotatedrectArea"] >> min_Rotatedrect_Area;
    fstetris["maxRotatedrectArea"] >> max_Rotatedrect_Area;
}

tetris::tetris(const cv::RotatedRect& rect, COLORS type):
    type(type),
    rotatedrect(rect) {
    this->height = rect.size.height;
    this->width = rect.size.width;
    this->updown = false;
}

commandType::commandType(int x, int y):
    x(x),
    y(y),
    angle(0) {
}