#ifndef LIDAR_VISUAL_ALIGNMENT_UTILS_H
#define LIDAR_VISUAL_ALIGNMENT_UTILS_H

#include <pcl/io/pcd_io.h>

struct Plane {
  pcl::PointCloud<pcl::PointXYZI> point_cloud;
  pcl::PointXYZ point_center;
  Eigen::Vector3d normal;
  int index;
};

cv::Mat FillImage(const cv::Mat &input_img) {
  cv::Mat fill_img = input_img.clone();
  for (int y = 2; y < input_img.rows - 2; y++) {
    for (int x = 2; x < input_img.cols - 2; x++) {
      if (input_img.at<uchar>(y, x) == 0) {
        if (input_img.at<uchar>(y - 1, x) != 0) {
          fill_img.at<uchar>(y, x) = input_img.at<uchar>(y - 1, x);
        } else if (input_img.at<uchar>(y, x - 1) != 0) {
          fill_img.at<uchar>(y, x) = input_img.at<uchar>(y, x - 1);
        }
      } else {
        int left_depth = input_img.at<uchar>(y, x - 1);
        int right_depth = input_img.at<uchar>(y, x + 1);
        int up_depth = input_img.at<uchar>(y + 1, x);
        int down_depth = input_img.at<uchar>(y - 1, x);
        int current_depth = input_img.at<uchar>(y, x);
        if ((current_depth - left_depth) > 5 &&
            (current_depth - right_depth) > 5 && left_depth != 0 &&
            right_depth != 0) {
          fill_img.at<uchar>(y, x) = (left_depth + right_depth) / 2;
        } else if ((current_depth - up_depth) > 5 &&
                   (current_depth - down_depth) > 5 && up_depth != 0 &&
                   down_depth != 0) {
          fill_img.at<uchar>(y, x) = (up_depth + down_depth) / 2;
        }
      }
    }
  }
  return fill_img;
}

void MapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

#endif // LIDAR_VISUAL_ALIGNMENT_UTILS_H