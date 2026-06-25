#ifndef MATHTYPES_H
#define MATHTYPES_H

#include <eigen3/Eigen/Dense>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace quadruped_controller {

using SE3 = pinocchio::SE3Tpl<double>;
using Motion = pinocchio::MotionTpl<double>;

// ========================
// Eigen 向量类型
// ========================
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using VecInt4 = Eigen::Matrix<int, 4, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Quat = Eigen::Matrix<double, 4, 1>; // 四元数
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Vec18 = Eigen::Matrix<double, 18, 1>;
using VecX = Eigen::Matrix<double, Eigen::Dynamic, 1>;

// ========================
// Eigen 矩阵类型
// ========================
using RotMat = Eigen::Matrix<double, 3, 3>;  // 旋转矩阵
using HomoMat = Eigen::Matrix<double, 4, 4>; // 齐次变换
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat12 = Eigen::Matrix<double, 12, 12>;
using MatX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

// 3×4 矩阵，每列是一个足端位置向量
using Vec34 = Eigen::Matrix<double, 3, 4>;

// ========================
// 工具函数
// ========================
inline Vec34 vec12ToVec34(Vec12 vec12) {
  Vec34 vec34;
  for (int i = 0; i < 4; ++i)
    vec34.col(i) = vec12.segment(3 * i, 3);
  return vec34;
}

inline Vec12 vec34ToVec12(Vec34 vec34) {
  Vec12 vec12;
  for (int i = 0; i < 4; ++i) {
    vec12.segment(3 * i, 3) = vec34.col(i);
  }
  return vec12;
}

} // namespace quadruped_controller

#endif // MATHTYPES_H