#pragma once

#include <QImage>
#include <opencv2/opencv.hpp>

inline QImage toQImage(const cv::Mat& bgr) {
    return QImage(bgr.data, bgr.cols, bgr.rows, static_cast<qsizetype>(bgr.step),
                  QImage::Format_BGR888)
        .copy();
}