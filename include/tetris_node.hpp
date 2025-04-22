#pragma once

#include "detector.hpp"
#include "planner.hpp"

const std::string CONFIG_PATH = "../config.yaml";

class tetris_node {
public:
    tetris_node();
    bool getCommand(const cv::Mat& img);

private:
    Param m_param;
    Detector m_detector;
    Planner m_planner;
};