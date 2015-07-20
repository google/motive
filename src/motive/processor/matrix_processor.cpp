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

#include "mathfu/constants.h"
#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"

using mathfu::vec4;
using mathfu::mat4;
using fpl::Angle;

namespace motive {

static inline bool IsRotation(MatrixOperationType type) {
  return type <= kRotateAboutZ;
}

// Runtime structure to hold one operation and the input value of that
// operation. Kept as small as possible to conserve memory, since every
// matrix will be constructed by a series of these.
class MatrixOperation {
 public:
  MatrixOperation() {
    SetType(kInvalidMatrixOperation);
    SetAnimationType(kInvalidAnimationType);
  }

  MatrixOperation(const MatrixOperationInit& init, MotiveEngine* engine) {
    const AnimationType animation_type =
        init.init == nullptr ? kConstValueAnimation : kMotivatorAnimation;
    SetType(init.type);
    SetAnimationType(animation_type);

    switch (animation_type) {
      case kMotivatorAnimation: {
        // Manually construct the motivator in the union's memory buffer.
        Motivator1f* motivator =
            new (value_.motivator_memory) Motivator1f(*init.init, engine);

        // Initialize the state if required.
        switch (init.union_type) {
          case MatrixOperationInit::kUnionEmpty:
            break;

          case MatrixOperationInit::kUnionInitialValue:
            motivator->SetTarget(Current1f(init.initial_value));
            break;

          case MatrixOperationInit::kUnionTarget:
            motivator->SetTarget(*init.target);
            break;

          case MatrixOperationInit::kUnionSpline:
            motivator->SetSpline(*init.spline);
            break;

          default:
            assert(false);
        }
        break;
      }

      case kConstValueAnimation:
        // If this value is not driven by an motivator, it must have a constant
        // value.
        assert(init.union_type == MatrixOperationInit::kUnionInitialValue);

        // Record the const value into the union.
        value_.const_value = init.initial_value;
        break;

      default:
        assert(false);
    }
  }

  ~MatrixOperation() {
    // Manually call the Motivator destructor, since the union hides it.
    if (animation_type_ == kMotivatorAnimation) {
      Motivator().~Motivator1f();
    }
  }

  // Return the type of operation we are animating.
  MatrixOperationType Type() const {
    return static_cast<MatrixOperationType>(matrix_operation_type_);
  }

  // Return the value we are animating.
  float Value() const {
    return animation_type_ == kMotivatorAnimation ? Motivator().Value()
                                                  : value_.const_value;
  }

  // Return the child motivator if it is valid. Otherwise, return nullptr.
  Motivator1f* ValueMotivator() {
    return animation_type_ == kMotivatorAnimation ? &Motivator() : nullptr;
  }

  const Motivator1f* ValueMotivator() const {
    return animation_type_ == kMotivatorAnimation ? &Motivator() : nullptr;
  }

  void SetTarget1f(const MotiveTarget1f& t) { Motivator().SetTarget(t); }
  void SetValue1f(float value) {
    assert(animation_type_ == kConstValueAnimation &&
           (!IsRotation(Type()) || Angle::IsAngleInRange(value)));
    value_.const_value = value;
  }

 private:
  enum AnimationType {
    kInvalidAnimationType,
    kMotivatorAnimation,
    kConstValueAnimation
  };

  // Disable copies so we don't have to worry about copying the Motivator1f in
  // the union.
  MatrixOperation(const MatrixOperation& rhs);
  MatrixOperation& operator=(const MatrixOperation& rhs);

  // Motivator1f has non-trivial constructors, destructors, and copy operators,
  // so we don't use it in the union. C++11 supports these kinds of unions,
  // but not all compilers (notably, Visual Studio) have complex union support
  // yet.
  union AnimatedValue {
    uint8_t motivator_memory[sizeof(Motivator1f)];
    float const_value;
  };

  void SetType(MatrixOperationType type) {
    matrix_operation_type_ = static_cast<uint8_t>(type);
  }

  void SetAnimationType(AnimationType animation_type) {
    animation_type_ = static_cast<uint8_t>(animation_type);
  }

  Motivator1f& Motivator() {
    assert(animation_type_ == kMotivatorAnimation);
    return *reinterpret_cast<Motivator1f*>(value_.motivator_memory);
  }

  const Motivator1f& Motivator() const {
    assert(animation_type_ == kMotivatorAnimation);
    return *reinterpret_cast<const Motivator1f*>(value_.motivator_memory);
  }

  // Enum MatrixOperationType compressed to 8-bits to save memory.
  // The matrix operation that we're performing.
  uint8_t matrix_operation_type_;

  // Enum AnimationType compressed to 8-bits to save memory.
  // The union parameter in 'value_' that is currently valid.
  uint8_t animation_type_;

  // The value being animated. Union because value can come from several
  // sources. The currently valid union member is specified by animation_type_.
  AnimatedValue value_;
};

// Perform a matrix rotation about
static inline void RotateAboutAxis(const float angle, vec4* column0,
                                   vec4* column1) {
  // TODO OPT: call platform-specific function to calculate both sin and cos
  // simultaneously.
  const float s = sin(angle);
  const float c = cos(angle);
  const vec4 c0 = *column0;
  const vec4 c1 = *column1;
  *column0 = c * c0 + s * c1;
  *column1 = c * c1 - s * c0;
}

// Hold a series of matrix operations, and their resultant matrix.
//
// This class is of variable size, to keep compact and to avoid cache misses
// caused by pointer chasing. The 'ops_[]' array is actually of length
// 'num_ops_'. Each item in 'ops_' is one matrix operation.
//
class MatrixData {
  MatrixData() {}  // Use Create() to create this class.
 public:
  ~MatrixData() { Destroy(this); }

  // Execute the series of basic matrix operations in 'ops_'.
  // We break out the matrix into four column vectors to avoid matrix multiplies
  // (which are slow) in preference of operation-specific matrix math (which is
  // fast).
  mat4 CalculateResultMatrix() const {
    // Start with the identity matrix.
    vec4 c0(mathfu::kAxisX4f);  // (1, 0, 0, 0)
    vec4 c1(mathfu::kAxisY4f);  // (0, 1, 0, 0)
    vec4 c2(mathfu::kAxisZ4f);  // (0, 0, 1, 0)
    vec4 c3(mathfu::kAxisW4f);  // (0, 0, 0, 1)

    for (int i = 0; i < num_ops_; ++i) {
      const MatrixOperation& op = ops_[i];
      const float value = op.Value();

      switch (op.Type()) {
        // ( |  |  |  |)(c -s  0  0)   (c*  c*   |   |)
        // (c0 c1 c2 c3)(s  c  0  0) = (c0+ c1- c2  c3)
        // ( |  |  |  |)(0  0  1  0)   (s*  s*   |   |)
        // ( |  |  |  |)(0  0  0  1)   (c1  c0   |   |)
        case kRotateAboutX:
          RotateAboutAxis(value, &c1, &c2);
          break;

        case kRotateAboutY:
          RotateAboutAxis(value, &c2, &c0);
          break;

        case kRotateAboutZ:
          RotateAboutAxis(value, &c0, &c1);
          break;

        // ( |  |  |  |)(1  0  0 tx)   ( |  |  | tx*c0+ )
        // (c0 c1 c2 c3)(0  1  0 ty) = (c0 c1 c2 ty*c1+ )
        // ( |  |  |  |)(0  0  1 tz)   ( |  |  | tz*c2+ )
        // ( |  |  |  |)(0  0  0  1)   ( |  |  |    c3  )
        case kTranslateX:
          c3 += value * c0;
          break;

        case kTranslateY:
          c3 += value * c1;
          break;

        case kTranslateZ:
          c3 += value * c2;
          break;

        // ( |  |  |  |)(sx 0  0  0)   ( |   |   |   |)
        // (c0 c1 c2 c3)(0  sy 0  0) = (sx* sy* sz*  |)
        // ( |  |  |  |)(0  0  sz 0)   (c0  c1  c2  c3)
        // ( |  |  |  |)(0  0  0  1)   ( |   |   |   |)
        case kScaleX:
          c0 *= value;
          break;

        case kScaleY:
          c1 *= value;
          break;

        case kScaleZ:
          c2 *= value;
          break;

        case kScaleUniformly:
          c0 *= value;
          c1 *= value;
          c2 *= value;
          break;

        default:
          assert(false);
      }
    }
    return mat4(c0, c1, c2, c3);
  }

  void UpdateResultMatrix() { result_matrix_ = CalculateResultMatrix(); }

  const MatrixOperation& Op(int child_index) const {
    assert(0 <= child_index && child_index < num_ops_);
    return ops_[child_index];
  }

  MatrixOperation& Op(int child_index) {
    assert(0 <= child_index && child_index < num_ops_);
    return ops_[child_index];
  }

  const mat4& result_matrix() const { return result_matrix_; }
  int num_ops() const { return num_ops_; }

  static MatrixData* Create(const MatrixInit& init, MotiveEngine* engine) {
    // Allocate a buffer that is big enough to hold MatrixData.
    const MatrixInit::OpVector& ops = init.ops();
    const int num_ops = static_cast<int>(ops.size());
    const size_t size = SizeOfClass(num_ops);
    uint8_t* buffer = new uint8_t[size];
    MatrixData* d = new (buffer) MatrixData();

    // Explicitly call constructors on members.
    d->result_matrix_ = mat4::Identity();
    d->num_ops_ = num_ops;
    for (int i = 0; i < num_ops; ++i) {
      new (&d->ops_[i]) MatrixOperation(ops[i], engine);
    }

    return d;
  }

  static void Destroy(MatrixData* d) {
    // Explicity call destructors.
    d->result_matrix_.~mat4();
    for (int i = 0; i < d->num_ops_; ++i) {
      d->ops_[i].~MatrixOperation();
    }

    // Explicitly delete buffer the same way it was allocated.
    uint8_t* buffer = reinterpret_cast<uint8_t*>(d);
    delete[] buffer;
  }

 private:
  static size_t SizeOfClass(int num_ops) {
    return sizeof(MatrixData) + sizeof(MatrixOperation) * (num_ops - 1);
  }

  mat4 result_matrix_;
  int num_ops_;
  MatrixOperation ops_[1];
};

// See comments on MatrixInit for details on this class.
class MatrixMotiveProcessor : public MotiveProcessorMatrix4f {
 public:
  virtual ~MatrixMotiveProcessor() {
    const MotiveIndex num_indices = NumIndices();
    for (MotiveIndex index = 0; index < num_indices; ++index) {
      RemoveIndex(index);
    }
  }

  virtual void AdvanceFrame(MotiveTime /*delta_time*/) {
    Defragment();

    // Process the series of matrix operations for each index.
    const MotiveIndex num_indices = NumIndices();
    for (MotiveIndex index = 0; index < num_indices; ++index) {
      MatrixData& d = Data(index);
      d.UpdateResultMatrix();
    }
  }

  virtual MotivatorType Type() const { return MatrixInit::kType; }
  virtual int Priority() const { return 2; }

  virtual const mat4& Value(MotiveIndex index) const {
    return Data(index).result_matrix();
  }

  virtual float ChildValue1f(MotiveIndex index,
                             MotiveChildIndex child_index) const {
    return Data(index).Op(child_index).Value();
  }

  virtual void SetChildTarget1f(MotiveIndex index, MotiveChildIndex child_index,
                                const MotiveTarget1f& t) {
    Data(index).Op(child_index).SetTarget1f(t);
  }

  virtual void SetChildValue1f(MotiveIndex index, MotiveChildIndex child_index,
                               float value) {
    Data(index).Op(child_index).SetValue1f(value);
  }

 protected:
  MotiveIndex NumIndices() const {
    return static_cast<MotiveIndex>(data_.size());
  }

  virtual void InitializeIndex(const MotivatorInit& init, MotiveIndex index,
                               MotiveEngine* engine) {
    RemoveIndex(index);
    auto init_params = static_cast<const MatrixInit&>(init);
    data_[index] = MatrixData::Create(init_params, engine);
  }

  virtual void RemoveIndex(MotiveIndex index) {
    if (data_[index] != nullptr) {
      MatrixData::Destroy(data_[index]);
      data_[index] = nullptr;
    }
  }

  virtual void MoveIndex(MotiveIndex old_index, MotiveIndex new_index) {
    data_[new_index] = data_[old_index];
    data_[old_index] = nullptr;
  }

  virtual void SetNumIndices(MotiveIndex num_indices) {
    const MotiveIndex old_num_indices = NumIndices();

    // Ensure old items are deleted.
    for (MotiveIndex i = num_indices; i < old_num_indices; ++i) {
      RemoveIndex(i);
    }

    // Initialize new items to nullptr.
    data_.resize(num_indices, nullptr);
  }

  const MatrixData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return *data_[index];
  }

  MatrixData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return *data_[index];
  }

  std::vector<MatrixData*> data_;
};

MOTIVE_INSTANCE(MatrixInit, MatrixMotiveProcessor);

}  // namespace motive
