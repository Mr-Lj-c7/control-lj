#ifndef EULER_ANGLES_ZXY_STRUCT_H_
#define EULER_ANGLES_ZXY_STRUCT_H_

#include <cmath>
#include "common/filters/math.h"

namespace control {
namespace common {

/**
 * @class EulerAnglesZXY
 *
 * Any orientation of a rigid body on a 3-D space can be achieved by
 * composing three rotations about the axes of an orthogonal coordinate system.
 * These rotations are said to be extrinsic if the axes are assumed to be
 * motionless, and intrinsic otherwise. Here, we use an intrinsic referential,
 * which is relative to the car's orientation.
 * Our vehicle reference frame follows NovAtel's convention:
 * Right/Forward/Up (RFU) respectively for the axes x/y/z.
 * In particular, we describe the orientation of the car by three angles:
 * 1) the pitch, in (-pi/2, pi/2), corresponds to a rotation around the x-axis;
 * 2) the roll, in [-pi, pi), corresponds to a rotation around the y-axis;
 * 3) the yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
 * The pitch is zero when the car is level and positive when the nose is up.
 * The roll is zero when the car is level and positive when the left part is up.
 * The yaw is zero when the car is facing North, and positive when facing West.
 * In turn, in the world frame, the x/y/z axes point to East/North/Up (ENU).
 * These angles represent the rotation from the world to the vehicle frames.
 *
 * @brief Implements a class of Euler angles (actually, Tait-Bryan angles),
 * with intrinsic sequence ZXY, for struct-based data.
 *
 * @param T Number type: double or float
 */
template <typename T>
class EulerAnglesZXY {
 public:
  /**
   * @brief Constructs an identity rotation.
   */
  EulerAnglesZXY() : roll_(0), pitch_(0), yaw_(0) {}

  /**
   * @brief Constructs a rotation using only yaw (i.e., around the z-axis).
   *
   * @param yaw The yaw of the car
   */
  explicit EulerAnglesZXY(T yaw) : roll_(0), pitch_(0), yaw_(yaw) {}

  /**
   * @brief Constructs a rotation using arbitrary roll, pitch, and yaw.
   *
   * @param roll The roll of the car
   * @param pitch The pitch of the car
   * @param yaw The yaw of the car
   */
  EulerAnglesZXY(T roll, T pitch, T yaw)
      : roll_(roll), pitch_(pitch), yaw_(yaw) {}

  /**
   * @brief Constructs a rotation using components of a quaternion struct.
   *
   * @param qw Quaternion w-coordinate
   * @param qx Quaternion x-coordinate
   * @param qy Quaternion y-coordinate
   * @param qz Quaternion z-coordinate
   */
  EulerAnglesZXY(T qw, T qx, T qy, T qz) {
    // 归一化四元数以确保精度
    T norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm == 0) {
      roll_ = pitch_ = yaw_ = 0;
      return;
    }
    
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
    
    // 计算 roll (绕X轴)
    T sinr_cosp = static_cast<T>(2.0) * (qw * qx + qy * qz);
    T cosr_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qx * qx + qy * qy);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);
    
    // 计算 pitch (绕Y轴)
    T sinp = static_cast<T>(2.0) * (qw * qy - qz * qx);
    if (std::abs(sinp) >= static_cast<T>(1.0)) {
      pitch_ = std::copysign(M_PI / 2.0, sinp); // 使用90度作为极限值
    } else {
      pitch_ = std::asin(sinp);
    }
    
    // 计算 yaw (绕Z轴)
    T siny_cosp = static_cast<T>(2.0) * (qw * qz + qx * qy);
    T cosy_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qy * qy + qz * qz);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  /**
   * @brief Getter for roll_
   * @return The roll of the car
   */
  T roll() const { return roll_; }

  /**
   * @brief Getter for pitch_
   * @return The pitch of the car
   */
  T pitch() const { return pitch_; }

  /**
   * @brief Getter for yaw_
   * @return The yaw of the car
   */
  T yaw() const { return yaw_; }

  /**
   * @brief Normalizes roll_, pitch_, and yaw_ to [-PI, PI).
   */
  void Normalize() {
    roll_ = NormalizeAngle(roll_);
    pitch_ = NormalizeAngle(pitch_);
    yaw_ = NormalizeAngle(yaw_);
  }

  /**
   * @brief Verifies the validity of the specified rotation.
   * @return True iff -PI/2 < pitch < PI/2
   */
  bool IsValid() {
    Normalize();
    return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
  }

 private:
  T roll_;
  T pitch_;
  T yaw_;
};

// 四元数到航向角转换函数
template <typename T>
T QuaternionToHeading(T qw, T qx, T qy, T qz) {
  EulerAnglesZXY<T> euler_angles(qw, qx, qy, qz);
  // euler_angles.yaw() is zero when the car is pointing North, but
  // the heading is zero when the car is pointing East.
  return NormalizeAngle(euler_angles.yaw() + M_PI_2);
}

using EulerAnglesZXYf = EulerAnglesZXY<float>;
using EulerAnglesZXYd = EulerAnglesZXY<double>;

}  // namespace common
}  // namespace control

#endif  // EULER_ANGLES_ZXY_STRUCT_H_