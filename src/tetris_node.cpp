#include "../include/tetris_node.hpp"

tetris_node::tetris_node():
    m_param(CONFIG_PATH) {
}

bool tetris_node::getCommand(const cv::Mat& img) {
    if (m_detector.detect(img) == false)
        return false;
    // m_planner.plan(m_detector.tetrisSet);
    // for (auto& command: m_planner.commandSet) {
    //     std::cout << "move " << Param::colorTostring.at(command.Tetris.type) << " to (" << command.x << ", " << command.y << ") with rotate angle " << command.angle << std::endl;
    // }
    return true;
}