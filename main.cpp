#include <iostream>
#include <opencv2/opencv.hpp>

#include "include/tetris_node.hpp"

const std::string image_path = "../image/6.jpg";

int main() {
    tetris_node node;
    cv::Mat img = cv::imread(image_path);
    cv::imshow("img", img);
    node.getCommand(img);
    return 0;
}