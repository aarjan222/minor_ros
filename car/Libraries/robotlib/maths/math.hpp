#ifndef MATH_H_
#define MATH_H_

#define ARM_MATH_CM4
#include "stm32f4xx_hal.h"
#include "arm_math.h"

template <typename t>
inline t map(t x, t in_min, t in_max, t out_min, t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename t> inline t map(t x, t out_min, t out_max) {
  return map(x, 0, 1, out_min, out_max);
}

template <typename t> t clamp(t x, t outmin, t outmax) {
  if (x > outmax)
    return outmax;
  else if (x < outmin)
    return outmin;
  return x;
}

template <typename t> inline t trim(t x, t a, t b, t c) {
  if (x > a && x <= b)
    return (x - a) / (b - a);
  else if (x > b && x <= c)
    return (c - x) / (c - b);
  return 0;
}

template <typename t> t invTrim(t x, t a, t b, t c) {
  t out1, out2;
  out1 = (b - a) * x + a;
  out2 = c - (c - b) * x;
  return (out1 + out2) / 2;
}

template <int n_rows, int n_cols> class Matrix {
public:
  float data[n_rows][n_cols];
  arm_matrix_instance_f32 arm_mat{n_rows, n_cols, (float*) data};

  Matrix() {}

  typedef Matrix<n_rows, n_cols> MatrixType;
  typedef Matrix<n_cols, n_rows> TransposeType;

  MatrixType &operator+=(const MatrixType &rhs) {
    MatrixType lhs = *this;
    arm_mat_add_f32(&(lhs.arm_mat), &(rhs.arm_mat), &(this->arm_mat));
    return *this;
  }

  TransposeType T() const {
    TransposeType ret;
    arm_mat_trans_f32(&(this->arm_mat), &(ret.arm_mat));
    return ret;
  }

  TransposeType inv() const {
    return inv(&this);
  }
};

template <typename MatrixType> MatrixType operator-(const MatrixType a) {
  MatrixType ret;
  arm_mat_scale_f32(&(a.arm_mat), -1.0, &(ret.arm_mat));
  return ret;
}

template <int n_rows> Matrix<n_rows, n_rows> inv(Matrix<n_rows, n_rows> a) {
  Matrix<n_rows, n_rows> ret;
  arm_mat_inverse_f64(a.arm_math, ret.arm_math);
  return ret;
}

template <typename MatrixType>
MatrixType operator+(MatrixType lhs, const MatrixType &rhs) {
  lhs += rhs;
  return lhs;
}

template <typename MatrixType>
MatrixType operator-(MatrixType lhs, const MatrixType &rhs) {
  lhs += (-rhs);
  return lhs;
}


// multiply mxn and nxp matrices
template <int m, int n, int p>
Matrix<m, p> operator*(const Matrix<m, n> lhs, const Matrix<n, p> &rhs) {
  Matrix<m, p> ret;
  arm_mat_mult_f32(&(lhs.arm_mat), &(rhs.arm_mat), &(ret.arm_mat));
  return ret;
}

template <int n>
float dot_product(Matrix<n,1> a, Matrix<n,1> b) {
  float ret;
  arm_dot_prod_f32((float*)a.data, (float*)b.data, n, &ret);
  return ret;
}

template <int n>
float vector_length(Matrix<n,1> a) {
  return sqrtf(dot_product(a, a));
}

[[maybe_unused]]
static void test() {
  Matrix<10, 1> a, b;
  Matrix <1, 10> c;
  a-b;
  a-c.T();
  dot_product(a,b);
}

#endif // MATH_H_
