#include "../include/detector.hpp"

/**
 * @brief 对图像进行颜色聚类分割，聚类数为3
 * @param[in] img 从相机传来的一帧图像
 * @return cv::Mat
 */
cv::Mat kmeansImageClustering(cv::Mat& img, int cluster_count) {
    if (img.empty()) {
        throw std::runtime_error("Input image is empty");
    }
    if (img.channels() != 3) {
        throw std::runtime_error("Input image must have 3 channels (RGB)");
    }
    if (cluster_count <= 0) {
        throw std::runtime_error("Cluster count must be positive");
    }

    // 将图像数据转换为浮点型并重塑为二维数组 (像素数, 3)
    cv::Mat data = img.reshape(1, img.rows * img.cols); // 重塑为 (像素数, 3)
    data.convertTo(data, CV_32F);                       // 就地转换为浮点型

    // 定义K-Means参数
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 20, 0.1);
    cv::Mat labels, centers;

    // 执行K-Means聚类
    cv::kmeans(data, cluster_count, labels, criteria, 10, cv::KMEANS_RANDOM_CENTERS, centers);

    // 将聚类中心转换为uint8类型
    centers.convertTo(centers, CV_8U);

    // 根据标签替换像素值为聚类中心颜色
    cv::Mat dst(img.size(), img.type());
    for (int i = 0; i < img.rows * img.cols; i++) {
        int cluster_idx = labels.at<int>(i);
        dst.at<cv::Vec3b>(i / img.cols, i % img.cols) = centers.at<cv::Vec3b>(cluster_idx);
    }

    return dst;
}

/**
 * @brief 绘制矩形框
 * @param[in] Src
 * @param[in] rect
 * @param[in] color
 * @param[in] thickness
 */
void drawRect(cv::Mat& src, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness) {
    cv::Point2f point[4];
    rect.points(point);
    for (int i = 0; i < 4; i++)
        cv::line(src, point[i], point[(i + 1) % 4], color, thickness);
}

/**
 * @brief 计算仿射变换的逆矩阵
 * @param[in] affineMat
 * @return cv::Mat
 */
inline cv::Mat invertAffine(const cv::Mat& affineMat) {
    cv::Mat invMat;
    cv::invertAffineTransform(affineMat, invMat);
    return invMat;
}

/**
 * @brief 识别所有俄罗斯方块。如果检测成功，则返回 true，否则返回 false。
 * @param[in] img 从相机传来的一帧图像
 * @return true
 * @return false
 */
bool Detector::detect(const cv::Mat& img) {
    preprocess(img);
    findTetris();
    if (totalTetrisCnt < 35 && Param::phase == PHASES::FIRST)
        return false;
    return true;
}

/**
 * @brief 预处理，包括图像的颜色聚类分割、腐蚀与二值化
 * @param[in] img 从相机传来的一帧图像
 */
void Detector::preprocess(const cv::Mat& img) {
    m_imageRaw = img.clone();
    m_image = kmeansImageClustering(m_imageRaw, 3);
    cv::imshow("kmeans", m_image);
    cv::cvtColor(m_image, m_image, cv::COLOR_BGR2GRAY);
    cv::threshold(m_image, m_image, Param::threshold, 255, cv::THRESH_BINARY);
    // cv::threshold(m_image, m_image, 0, 255, cv::THRESH_OTSU);
    cv::bitwise_not(m_image, m_image);
    cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
    cv::erode(m_image, m_image, kernel5, cv::Point(-1, -1));
    m_imageBinary = m_image.clone();
    cv::imshow("binary", m_imageBinary);
}

/**
 * @brief 在原图中找到所有的对象
 *        并计算出对象的抓取点以及需要旋转多少度即可变为标准状态
 */
void Detector::findTetris() {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(m_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    tetris Tetris;

    for (auto& contour: contours) {
        if (contour.size() < 7)
            continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        if (rect.size.area() < Param::min_Rotatedrect_Area || rect.size.area() > Param::max_Rotatedrect_Area)
            continue;
        drawRect(m_imageRaw, rect, Param::white, 2);
        double ratio = std::max(rect.size.height, rect.size.width) / std::min(rect.size.height, rect.size.width);
        if (ratio < 1.2) {
            Tetris = std::move(tetris(rect, COLORS::ORANGE));
            setCodeMat(Tetris);
        } else if (ratio < 6 && ratio > 3.5) {
            Tetris = std::move(tetris(rect, COLORS::RED));
            setCodeMat(Tetris);
        } else if (ratio < 2 && ratio > 1.2) {
            Tetris = std::move(tetris(rect, COLORS::WHITE));
            setCodeMat(Tetris);
            getCodeMatValue(Tetris);
        } else {
            continue;
        }

        if (Tetris.type == COLORS::WHITE)
            continue;

        getSuctionPosition(Tetris);
        tetrisSet[Tetris.type].push_back(std::move(Tetris));
    }

    for (auto& [_color, _tetris]: tetrisSet) {
        std::cout << Param::colorTostring.at(_color) << " tetris number: " << _tetris.size() << std::endl;
        for (auto& t: _tetris) {
            cv::circle(m_imageRaw, t.suctionPosition, 3, cv::Scalar(255, 255, 255), -1);
            cv::putText(m_imageRaw, std::to_string(t.angleToHorizontal * 180 / M_PI), t.suctionPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            cv::putText(m_imageRaw, std::to_string(t.rotatedrect.angle), t.suctionPosition + cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            // cv::putText(m_imageRaw, Param::colorTostring.at(t.type), t.rotatedrect.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            cv::putText(m_imageRaw, std::to_string(t.updown), t.rotatedrect.center + cv::Point2f(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            // cv::putText(m_imageRaw, std::to_string(t.codeMatValue), t.rotatedrect.center + cv::Point2f(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            totalTetrisCnt++;
        }
    }
    std::cout << "total tetris number: " << totalTetrisCnt << std::endl;
    cv::imshow("detected", m_imageRaw);
    cv::waitKey(0);
}

/**
 * @brief 通过目标对象在原图中的旋转矩阵 设置目标对象的codeMat
 *        (ORANGE 200*200 RED 100*400 else 200*300)
 *        m_angleToHorizontal为目标旋转到水平的状态需要旋转的角度(弧度)
 * @param Tetris 目标对象
 */
void Detector::setCodeMat(tetris& Tetris) {
    int codeMat_h, codeMat_w;
    if (Tetris.type == COLORS::ORANGE) {
        codeMat_h = 200;
        codeMat_w = 200;
    } else if (Tetris.type == COLORS::RED) {
        codeMat_h = 100;
        codeMat_w = 400;
    } else {
        codeMat_h = 200;
        codeMat_w = 300;
    }
    cv::Point2f RawPoints[3];
    std::array<cv::Point2f, 4> _points;
    Tetris.rotatedrect.points(_points.data());
    for (int i = 0; i < 3; i++) {
        RawPoints[i] = Tetris.height < Tetris.width ? _points[i] : _points[(i + 3) % 4];
        cv::circle(m_imageRaw, RawPoints[i], 3, cv::Scalar(255, 255, 255), -1);
        cv::putText(m_imageRaw, std::to_string(i), RawPoints[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    cv::Point2f codeMatPoints[] = { cv::Point2f(0, codeMat_h), cv::Point2f(0, 0), cv::Point2f(codeMat_w, 0) };
    Tetris.toCodeMat = cv::getAffineTransform(RawPoints, codeMatPoints);
    cv::warpAffine(m_imageBinary, Tetris.codeMat, Tetris.toCodeMat, cv::Size(codeMat_w, codeMat_h));
    Tetris.angleToHorizontal = Tetris.height < Tetris.width ? (-std::abs(Tetris.rotatedrect.angle) * M_PI / 180) : ((90 - std::abs(Tetris.rotatedrect.angle)) * M_PI / 180);
}

/**
 * @brief 通过对象的codeMat对图像进行二进制编码
 * @param Tetris
 */
void Detector::getCodeMatValue(tetris& Tetris) {
    Tetris.codeMatValue = 0;
    int cnt { 5 };
    for (int y = 0; y < 200; y += 100) {
        for (int x = 0; x < 300; x += 100) {
            // 提取每格的中心处20×20的部分作为判断依据
            cv::Rect roi(x + 40, y + 40, 20, 20);
            if (cv::mean(cv::Mat(Tetris.codeMat, roi))[0] >= 200) {
                Tetris.codeMatValue |= 1 << cnt;
            }
            cnt--;
        }
    }
    std::cout << "codeMatValue: " << Tetris.codeMatValue << std::endl;
    auto iter = std::find_if(Param::base.begin(), Param::base.end(), [&Tetris](const auto& item) {
        return std::get<0>(item) == Tetris.codeMatValue;
    });
    if (iter != Param::base.end()) {
        Tetris.type = std::get<1>(*iter);
        Tetris.updown = std::get<2>(*iter);
        if (Tetris.updown)
            Tetris.angleToHorizontal += M_PI;
    } else {
        // 可能是橙色被误识别
        Tetris.type = COLORS::ORANGE;
        setCodeMat(Tetris);
        Tetris.codeMatValue = 0;
        int cnt { 3 };
        for (int y = 0; y < 200; y += 100) {
            for (int x = 0; x < 200; x += 100) {
                // 提取每格的中心处20×20的部分作为判断依据
                cv::Rect roi(x + 40, y + 40, 20, 20);
                if (cv::mean(cv::Mat(Tetris.codeMat, roi))[0] >= 200) {
                    Tetris.codeMatValue |= 1 << cnt;
                }
                cnt--;
            }
        }
        if (Tetris.codeMatValue != 15) {
            Tetris.type = COLORS::WHITE;
            std::cout << "codeMatValue calculate wrong" << std::endl;
        }
    }
}

/**
 * @brief 通过对象的 1.颜色类型  2.预设抓取点  3.从codeMat变换到image的变换矩阵 4.对象在codeMat下是否需要updown 计算出目标抓取点在原图的位置
 *
 * @param Tetris
 */
void Detector::getSuctionPosition(tetris& Tetris) {
    cv::Mat posVector { (cv::Mat_<double>(3, 1) << 0, 0, 1) };
    cv::Point2f _point;
    if (Tetris.type == COLORS::PURPLE) {
        posVector = invertAffine(Tetris.toCodeMat) * (Tetris.updown ? Param::posVectorPurpleUpdown : Param::posVectorPurple);
    } else if (Tetris.type == COLORS::YELLOW) {
        posVector = invertAffine(Tetris.toCodeMat) * (Tetris.updown ? Param::posVectorYellowUpdown : Param::posVectorYellow);

    } else if (Tetris.type == COLORS::BLUE) {
        posVector = invertAffine(Tetris.toCodeMat) * Param::posVectorBlue;
    } else if (Tetris.type == COLORS::GREEN) {
        posVector = invertAffine(Tetris.toCodeMat) * Param::posVectorGreen;
    } else if (Tetris.type == COLORS::ORANGE) {
        posVector = invertAffine(Tetris.toCodeMat) * Param::posVectorOrange;
    } else if (Tetris.type == COLORS::BROWN) {
        posVector = invertAffine(Tetris.toCodeMat) * (Tetris.updown ? Param::posVectorBrownUpdown : Param::posVectorBrown);
    } else if (Tetris.type == COLORS::RED) {
        posVector = invertAffine(Tetris.toCodeMat) * Param::posVectorRed;
    } else {
        std::cout << "wrong type" << std::endl;
    }
    Tetris.suctionPosition = cv::Point(posVector.at<double>(0, 0), posVector.at<double>(0, 1));
}