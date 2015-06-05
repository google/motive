// Copyright 2014 Google Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTIVE_MATH_ANGLE_H_
#define MOTIVE_MATH_ANGLE_H_

#include <assert.h>
#include <math.h>

#include "mathfu/constants.h"
#include "mathfu/matrix.h"
#include "mathfu/vector.h"
#include "mathfu/glsl_mappings.h"

#ifdef FPL_ANGLE_UNIT_TESTS
#include "gtest/gtest.h"
#endif  // FPL_ANGLE_UNIT_TESTS

namespace fpl {


/// Describe a conversion from `Angle` to a 3D vector.
/// Sweep from the first axis towards the second axis.
/// For example, for kAngleToVectorXY,
///     angle = 0 degrees   ==>  x-axis (1, 0, 0)
///     angle = 90 degrees  ==>  y-axis (0, 1, 0)
///     angle = 180 degrees ==>  negative x-axis (-1, 0, 0)
///     angle = 90 degrees  ==>  negative y-axis (0, -1, 0)
/// Note that we don't have to worry about handedness. No matter the handedness
/// of the coordinate system, we always sweep from one axis towards the other.
/// That is, the other axis is always at 90 degrees, never -90.
enum AngleToVectorSystem {
  kAngleToVectorXY,
  kAngleToVectorXZ,
  kAngleToVectorYZ,
  kAngleToVectorYX,
  kAngleToVectorZX,
  kAngleToVectorZY,
  kAngleToVectorCount
};

static const float kPi = static_cast<float>(M_PI);
static const float kTwoPi = static_cast<float>(2.0 * M_PI);
static const float kThreePi = static_cast<float>(3.0 * M_PI);
static const float kHalfPi = static_cast<float>(M_PI_2);
static const float kQuarterPi = static_cast<float>(0.5 * M_PI_2);
static const float kDegreesToRadians = static_cast<float>(M_PI / 180.0);
static const float kRadiansToDegrees = static_cast<float>(180.0 / M_PI);
static const float kMaxUniqueAngle = kPi;
static const float kDegreesPerCircle = 360.0f;

// The biggest floating point number > -pi.
// That is, there are no floats x s.t. -pi < x < kMinUniqueAngle.
//
// Useful for clamping to the valid angle range:
//    [kMinUniqueAngle,kMaxUniqueAngle] == (-pi, pi]
// Note: "[" indicates an inclusive bound; "(" indicates exclusive.
//
// In general, after modding, we should clamp to the valid range.
// Floating point precision errors can generate numbers outside of
// (-pi, pi], even when the math is perfect.
static const float kMinUniqueAngle = -3.1415925f;

/// @class Angle
/// @brief Represent an angle in radians, uniquely in the range (-pi, pi].
///
/// We include pi in the range, but exclude -pi, because pi and -pi
/// are equivalent mod 2pi.
///
/// Equivalence is key to this class. We want only one representation
/// of every equivalent angle. For example, 0 and 2pi are both represented
/// as 0, internally. This unique representation allows for comparison and
/// precise arithmetic.
///
/// All operators keep the angle values in the valid range.
///
/// Why use instead of Quaternions?
/// -------------------------------
/// Quaternions are great for three dimensional rotations, but for many
/// applications you only have two dimensional rotations. Instead of the
/// four floats and heavy operations required by quaternions, an angle
/// can represent the rotation in one float and reasonably light operations.
/// Angles are easier to do trigonometry on. Also, they're conceptually
/// simpler.
///
class Angle {
 public:
  Angle() : angle_(0.0f) {}

  /// Create from `angle`, which is already in the valid range (-pi,pi].
  /// If your angle is outside that range, construct the Angle with the
  /// slower FromRadians function to automatically wrap it.
  /// @param angle radians in the range (-pi,pi] -- i.e. exclusive of -pi but
  ///              inclusive of +pi.
  explicit Angle(float angle) : angle_(angle) { assert(IsValid()); }

  /// Returns the absolute value of an angle.
  Angle Abs() const { return Angle(fabs(angle_)); }

  Angle& operator=(const Angle& rhs) {
    angle_ = rhs.angle_;
    return *this;
  }

  /// Add `rhs` and ensure result is in the range (-pi,pi].
  Angle& operator+=(const Angle& rhs) {
    angle_ = ModWithinThreePi(angle_ + rhs.angle_);
    return *this;
  }

  /// Subtract `rhs` and ensure result is in the range (-pi,pi].
  Angle& operator-=(const Angle& rhs) {
    angle_ = ModWithinThreePi(angle_ - rhs.angle_);
    return *this;
  }

  /// Multiply `rhs` and ensure result is in the normalized range (-pi,pi].
  Angle& operator*=(const float rhs) {
    angle_ = WrapAngle(angle_ * rhs);
    return *this;
  }

  /// Divide `rhs` and ensure result is in the normalized range (-pi,pi].
  Angle& operator/=(const float rhs) {
    angle_ = WrapAngle(angle_ / rhs);
    return *this;
  }

  /// Negate the angle and ensure result is in the normalized range (-pi,pi].
  Angle operator-() const { return Angle(ModIfNegativePi(-angle_)); }

  /// Return the angle value in radians. Value is in the range (-pi,pi].
  float ToRadians() const { return angle_; }

  /// Return the angle value in degrees. Value is in the range (-180,180].
  float ToDegrees() const { return kRadiansToDegrees * angle_; }

  /// Returns a point on the unit circle corresponding to a sweep of `angle`
  /// across the specified vector system.
  mathfu::vec3 ToVectorSystem(const AngleToVectorSystem system) const {
    switch(system) {
      case kAngleToVectorXY: return ToXYVector();
      case kAngleToVectorXZ: return ToXZVector();
      case kAngleToVectorYZ: return ToYZVector();
      case kAngleToVectorYX: return ToYXVector();
      case kAngleToVectorZX: return ToZXVector();
      case kAngleToVectorZY: return ToZYVector();
      default: assert(false);
    }
    return mathfu::kZeros3f;
  }

  /// Returns a point on unit circle corresponding to a sweep of `angle`
  /// degrees from the x-axis towards the y-axis.
  ///   0     ==> ( 1,  0,  0)
  ///   pi/2  ==> ( 0,  1,  0)
  ///   pi    ==> (-1,  0,  0)
  ///   3pi/2 ==> ( 0, -1,  0)
  mathfu::vec3 ToXYVector() const {
    float x, y;
    ToVector(&x, &y);
    return mathfu::vec3(x, y, 0.0f);
  }

  /// Returns a point on unit circle corresponding to a sweep of `angle`
  /// degrees from the x-axis towards the z-axis.
  ///   0     ==> ( 1,  0,  0)
  ///   pi/2  ==> ( 0,  0,  1)
  ///   pi    ==> (-1,  0,  0)
  ///   3pi/2 ==> ( 0,  0, -1)
  mathfu::vec3 ToXZVector() const {
    float x, z;
    ToVector(&x, &z);
    return mathfu::vec3(x, 0.0f, z);
  }

  /// Returns a point on unit circle corresponding to a sweep of `angle`
  /// degrees from the y-axis towards the z-axis.
  ///   0     ==> (0,  1,  0)
  ///   pi/2  ==> (0,  0,  1)
  ///   pi    ==> (0, -1,  0)
  ///   3pi/2 ==> (0,  0, -1)
  mathfu::vec3 ToYZVector() const {
    float y, z;
    ToVector(&y, &z);
    return mathfu::vec3(0.0f, y, z);
  }

  /// Returns a point on unit circle corresponding to a sweep of `angle`
  /// degrees from the y-axis towards the x-axis.
  ///   0     ==> ( 0,  1,  0)
  ///   pi/2  ==> ( 1,  0,  0)
  ///   pi    ==> ( 0, -1,  0)
  ///   3pi/2 ==> (-1,  0,  0)
  mathfu::vec3 ToYXVector() const {
    float x, y;
    ToVector(&y, &x);
    return mathfu::vec3(x, y, 0.0f);
  }

  /// Returns a point on unit circle corresponding to a sweep of `angle`
  /// degrees from the z-axis towards the x-axis.
  ///   0     ==> ( 0,  0,  1)
  ///   pi/2  ==> ( 1,  0,  0)
  ///   pi    ==> ( 0,  0, -1)
  ///   3pi/2 ==> (-1,  0,  0)
  mathfu::vec3 ToZXVector() const {
    float x, z;
    ToVector(&z, &x);
    return mathfu::vec3(x, 0.0f, z);
  }

  /// Returns a point on unit circle corresponding to a sweep of `angle`
  /// degrees from the z-axis towards the y-axis.
  ///   0     ==> (0,  0,  1)
  ///   pi/2  ==> (0,  1,  0)
  ///   pi    ==> (0,  0, -1)
  ///   3pi/2 ==> (0, -1,  0)
  mathfu::vec3 ToZYVector() const {
    float y, z;
    ToVector(&z, &y);
    return mathfu::vec3(0.0f, y, z);
  }

  /// Returns a point on the unit circle corresponding to a sweep of `angle`
  /// across the specified vector system.
  mathfu::mat3 ToRotationMatrix(const AngleToVectorSystem system) const {
    switch(system) {
      case kAngleToVectorXY: return ToXYRotationMatrix();
      case kAngleToVectorXZ: return ToXZRotationMatrix();
      case kAngleToVectorYZ: return ToYZRotationMatrix();
      case kAngleToVectorYX: return ToYXRotationMatrix();
      case kAngleToVectorZX: return ToZXRotationMatrix();
      case kAngleToVectorZY: return ToZYRotationMatrix();
      default: assert(false);
    }
    return mathfu::mat3();
  }

  /// Returns a matrix that rotates about the Z axis `angle` radians.
  mathfu::mat3 ToXYRotationMatrix() const {
    float x, y;
    ToVector(&x, &y);
    return mathfu::mat3(x, y, 0.0f, -y, x, 0.0f, 0.0f, 0.0f, 1.0f);
  }

  /// Returns a matrix that rotates about the Y axis `angle` radians.
  mathfu::mat3 ToXZRotationMatrix() const {
    float x, z;
    ToVector(&x, &z);
    return mathfu::mat3(x, 0.0f, z, 0.0f, 1.0f, 0.0f, -z, 0.0f, x);
  }

  /// Returns a matrix that rotates about the X axis `angle` radians.
  mathfu::mat3 ToYZRotationMatrix() const {
    float y, z;
    ToVector(&y, &z);
    return mathfu::mat3(1.0f, 0.0f, 0.0f, 0.0f, y, z, 0.0f, -z, y);
  }

  /// Returns a matrix that rotates about the Z axis `-angle` radians.
  mathfu::mat3 ToYXRotationMatrix() const {
    return operator-().ToXYRotationMatrix();
  }

  /// Returns a matrix that rotates about the Y axis `-angle` radians.
  mathfu::mat3 ToZXRotationMatrix() const {
    return operator-().ToXZRotationMatrix();
  }

  /// Returns a matrix that rotates about the X axis `-angle` radians.
  mathfu::mat3 ToZYRotationMatrix() const {
    return operator-().ToYZRotationMatrix();
  }

  /// Check internal consistency. If class is functioning correctly, should
  /// always return true.
  bool IsValid() const { return IsAngleInRange(angle_); }

  /// Clamps the angle to the range [center - max_diff, center + max_diff].
  /// max_diff must be in the range [0~pi].
  Angle Clamp(const Angle& center, const Angle& max_diff) const;

  /// Wraps an angle to the range (-pi, pi].
  /// This function is slow because it has a division. When possible, use
  /// FromWithinThreePi instead.
  static float WrapAngle(float angle) {
    angle -= (floor(angle / kTwoPi) + 1.0f) * kTwoPi;
    if (angle <= -kPi) {
      angle += kTwoPi;
    }
    return angle;
  }

  /// Create from `angle`, in radians, which is in the range (-3pi,3pi].
  /// This function is significantly faster than WrapAngle since it avoids
  /// division. It's also more precise for the same reason. The range may seem
  /// strange at first glance; it's a consequence of the implementation. Just
  /// know that any two sums of normalized angles will still be in the range
  /// (-3pi,3pi].
  static Angle FromWithinThreePi(const float angle) {
    return Angle(ModWithinThreePi(angle));
  }

  /// Create from `radians`, which is converted to the range (-pi, pi].
  static Angle FromRadians(const float radians) {
    return Angle(WrapAngle(radians));
  }

  /// Create from `degrees`, which is converted to the range (-pi, pi].
  static Angle FromDegrees(const float degrees) {
    return FromRadians(degrees * kDegreesToRadians);
  }

  /// Returns a point on the unit circle corresponding to a sweep of `angle`
  /// across the specified vector system.
  static Angle FromVectorSystem(const mathfu::vec3& v,
                                const AngleToVectorSystem system) {
    switch(system) {
      case kAngleToVectorXY: return FromXYVector(v);
      case kAngleToVectorXZ: return FromXZVector(v);
      case kAngleToVectorYZ: return FromYZVector(v);
      case kAngleToVectorYX: return FromYXVector(v);
      case kAngleToVectorZX: return FromZXVector(v);
      case kAngleToVectorZY: return FromZYVector(v);
      default: assert(false);
    }
    return Angle(0.0f);
  }

  /// Create from the x,y coordinates of a vector, using the system
  /// described in ToXYVector().
  static Angle FromXYVector(const mathfu::vec3& v) {
    return Angle(ModIfNegativePi(atan2f(v[1], v[0])));
  }

  /// Create from the x,z coordinates of a vector, using the system
  /// described in ToXZVector().
  static Angle FromXZVector(const mathfu::vec3& v) {
    return Angle(ModIfNegativePi(atan2f(v[2], v[0])));
  }

  /// Create from the y,z coordinates of a vector, using the system
  /// described in ToYZVector().
  static Angle FromYZVector(const mathfu::vec3& v) {
    return Angle(ModIfNegativePi(atan2f(v[2], v[1])));
  }

  /// Create from the y,x coordinates of a vector, using the system
  /// described in ToYXVector().
  static Angle FromYXVector(const mathfu::vec3& v) {
    return Angle(ModIfNegativePi(atan2f(v[0], v[1])));
  }

  /// Create from the z,x coordinates of a vector, using the system
  /// described in ToZXVector().
  static Angle FromZXVector(const mathfu::vec3& v) {
    return Angle(ModIfNegativePi(atan2f(v[0], v[2])));
  }

  /// Create from the z,y coordinates of a vector, using the system
  /// described in ToZYVector().
  static Angle FromZYVector(const mathfu::vec3& v) {
    return Angle(ModIfNegativePi(atan2f(v[1], v[2])));
  }

  /// Return true if 'angle' is within the valid range (-pi,pi], that is,
  /// the range inclusive of +pi but exclusive of -pi.
  static bool IsAngleInRange(const float angle) {
    return kMinUniqueAngle <= angle && angle <= kMaxUniqueAngle;
  }

 private:
  friend bool operator==(const Angle& a, const Angle& b);
  friend bool operator<(const Angle& a, const Angle& b);
  friend bool operator<=(const Angle& a, const Angle& b);

#ifdef FPL_ANGLE_UNIT_TESTS
  FRIEND_TEST(AngleTests, ModWithinThreePi);
  FRIEND_TEST(AngleTests, ModIfNegativePi);
#endif  // FPL_ANGLE_UNIT_TESTS

  /// Convert the angle into a 2D unit vector of the form
  /// (`zero_axis`, `ninety_axis`).
  /// When angle is 0 degrees, returns (1, 0), hence the name `zero_axis`.
  /// When angle is 90 degrees, returns (0, 1), hence the name `ninety_axis`.
  /// When angle is 180 degrees, returns (-1, 0).
  /// When angle is -90 degrees, returns (0, -1).
  void ToVector(float* const zero_axis, float* const ninety_axis) const {
    // TODO OPT: Call single function that calculates both cos and sin.
    *zero_axis = cos(angle_);
    *ninety_axis = sin(angle_);
  }

  // Take 'angle' in the range (-3pi,3pi] and return an equivalent angle in
  // the range (-pi,pi]. Note that angles are equivalent if they differ by 2pi.
  static float ModWithinThreePi(const float angle) {
    assert(-kThreePi < angle && angle < kThreePi);
    // These ternary operators should be converted into select statements by
    // the compiler.
    const float above = angle < kMinUniqueAngle ? angle + kTwoPi : angle;
    const float below = above > kMaxUniqueAngle ? above - kTwoPi : above;
    assert(IsAngleInRange(below));
    return below;
  }

  static float ModIfNegativePi(const float angle) {
    // Pi negates to -pi, which is outside the range so becomes +pi again.
    return angle < kMinUniqueAngle ? kMaxUniqueAngle : angle;
  }

  float angle_;  // Angle in radians, in range (-pi, pi]
};

inline Angle operator+(Angle lhs, const Angle& rhs) {
  lhs += rhs;
  return lhs;
}

inline Angle operator-(Angle lhs, const Angle& rhs) {
  lhs -= rhs;
  return lhs;
}

inline Angle operator*(Angle lhs, float rhs) {
  lhs *= rhs;
  return lhs;
}

inline Angle operator/(Angle lhs, float rhs) {
  lhs /= rhs;
  return lhs;
}

inline bool operator==(const Angle& a, const Angle& b) {
  return a.angle_ == b.angle_;
}

inline bool operator!=(const Angle& a, const Angle& b) {
  return !operator==(a, b);
}

inline bool operator<(const Angle& a, const Angle& b) {
  return a.angle_ < b.angle_;
}

inline bool operator>=(const Angle& a, const Angle& b) {
  return !operator<(a, b);
}

inline bool operator<=(const Angle& a, const Angle& b) {
  return a.angle_ <= b.angle_;
}

inline bool operator>(const Angle& a, const Angle& b) {
  return !operator<=(a, b);
}

inline Angle Angle::Clamp(const Angle& center, const Angle& max_diff) const {
  assert(0 <= max_diff.angle_ && max_diff.angle_ <= kPi);

  // Get difference from 'center'. We know this will be a value in the range
  // (-pi, pi].
  const Angle diff = (*this) - center;

  // Clamp the difference to the valid range.
  const Angle diff_clamped(
      mathfu::Clamp(diff.angle_, -max_diff.angle_, max_diff.angle_));

  // Add the difference onto the center. Note that, if no clamping happened,
  // we're left with *this.
  return center + diff_clamped;
}

}  // namespace fpl

#endif  // MOTIVE_MATH_ANGLE_H_
