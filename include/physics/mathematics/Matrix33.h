#ifndef PHYSICS_MATRIX33_H
#define PHYSICS_MATRIX33_H

#include <cassert>
#include <physics/mathematics/Vector3.h>

namespace physics {

class Matrix33 {

  private:
    /* -- Attributes -- */

    /* Matrix rows */
    Vector3 mRows[3];

  public:
    /* -- Methods -- */
    
    /* Constructor */
    Matrix33();

    /* Constructor with single parameter */
    Matrix33(float number);

    /* Constructor with multiple parameters */
    Matrix33(float a, float b, float c,
             float d, float e, float f,
             float g, float h, float i);

    /* Set components of the matrix with single parameter */
    void set(float value);

    /* Set components of the matrix with multiple parameters */
    void set(float a, float b, float c,
             float d, float e, float f,
             float g, float h, float i);

    /* Set components of the matrix to zero */
    void setZero();

    /* Set matrix to identity matrix */
    void setIdentity();

    /* Get a column of the matrix */
    Vector3 getColumn(int i) const;

    /* Get a row of the matrix */
    Vector3 getRow(int i) const;

    /* Get transpose of the matrix */
    Matrix33 getTranspose() const;

    /* Get determinant of the matrix */
    float getDeterminant() const;

    /* Get inverse of the matrix */
    Matrix33 getInverse() const;

    /* Get identity matrix */
    static Matrix33 getIdentity();

    /* Get zero matrix */
    static Matrix33 getZero();

    /* Overloaded equality operator */
    bool operator==(const Matrix33& matrix) const;

    /* Overloaded inequality operator */
    bool operator!=(const Matrix33& matrix) const;

    /* Overloaded addition operator with assignment */
    Matrix33& operator+=(const Matrix33& matrix);

    /* Overloaded subtraction operator with assignment */
    Matrix33& operator-=(const Matrix33& matrix);

    /* Overloaded multiplication operator with assignment */
    Matrix33& operator*=(float number);

    /* Overloaded value access operator */
    Vector3& operator[](int row);

    /* Overloaded value access operator */
    const Vector3& operator[](int row) const;

    /* -- Friends -- */

    friend Matrix33 operator+(const Matrix33& matrix1, const Matrix33& matrix2);
    friend Matrix33 operator-(const Matrix33& matrix1, const Matrix33& matrix2);
    friend Matrix33 operator-(const Matrix33& matrix);
    friend Matrix33 operator*(float number, const Matrix33& matrix);
    friend Matrix33 operator*(const Matrix33& matrix, float number);
    friend Vector3 operator*(const Matrix33& matrix, const Vector3& vector);
};

/* Constructor */
inline Matrix33::Matrix33() {
  set(0.0f);
}

/* Constructor with single parameter */
inline Matrix33::Matrix33(float value) {
  set(value);
}

/* Constructor with multiple parameters */
inline Matrix33::Matrix33(float a, float b, float c,
                          float d, float e, float f,
                          float g, float h, float i) {
  set(a, b, c, d, e, f, g, h, i);
}

/* Set components of the matrix with single parameter */
inline void Matrix33::set(float value) {
  mRows[0][0] = value; mRows[0][1] = value; mRows[0][2] = value;
  mRows[1][0] = value; mRows[1][1] = value; mRows[1][2] = value;
  mRows[2][0] = value; mRows[2][1] = value; mRows[2][2] = value;
}

/* Set components of the matrix with multiple parameters */
inline void Matrix33::set(float a, float b, float c,
                          float d, float e, float f,
                          float g, float h, float i) {
  mRows[0][0] = a; mRows[0][1] = b; mRows[0][2] = c;
  mRows[1][0] = d; mRows[1][1] = e; mRows[1][2] = f;
  mRows[2][0] = g; mRows[2][1] = h; mRows[2][2] = i;
}

/* Set components of the matrix to zero */
inline void Matrix33::setZero() {
  mRows[0].setZero();
  mRows[1].setZero();
  mRows[2].setZero();
}

/* Set matrix to identity matrix */
inline void Matrix33::setIdentity() {
  mRows[0][0] = 1.0f; mRows[0][1] = 0.0f; mRows[0][2] = 0.0f;
  mRows[1][0] = 0.0f; mRows[1][1] = 1.0f; mRows[1][2] = 0.0f;
  mRows[2][0] = 0.0f; mRows[2][1] = 0.0f; mRows[2][2] = 1.0f;
}

/* Get a column of the matrix */
inline Vector3 Matrix33::getColumn(int i) const {
  assert(i >= 0 && i < 3);
  return Vector3(mRows[0][i], mRows[1][i], mRows[2][i]);
}

/* Get a row of the matrix */
inline Vector3 Matrix33::getRow(int i) const {
  assert(i >= 0 && i < 3);
  return mRows[i];
}

/* Get transpose of the matrix */
inline Matrix33 Matrix33::getTranspose() const {
  return Matrix33(mRows[0][0], mRows[1][0], mRows[2][0],
                  mRows[0][1], mRows[1][1], mRows[2][1],
                  mRows[0][2], mRows[1][2], mRows[2][2]);
}

/* Get determinant of matrix */
inline float Matrix33::getDeterminant() const {
  return (mRows[0][0]*(mRows[1][1]*mRows[2][2]-mRows[2][1]*mRows[1][2]) -
          mRows[0][1]*(mRows[1][0]*mRows[2][2]-mRows[2][0]*mRows[1][2]) +
          mRows[0][2]*(mRows[1][0]*mRows[2][1]-mRows[2][0]*mRows[1][1]));
}

/* Get identity matrix */
inline Matrix33 Matrix33::getIdentity() {
  return Matrix33(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

/* Get zero matrix */
inline Matrix33 Matrix33::getZero() {
  return Matrix33(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/* Overloaded equality operator */
inline bool Matrix33::operator==(const Matrix33& matrix) const {
  return (mRows[0] == Vector3(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2]),
          mRows[1] == Vector3(matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2]),
          mRows[2] == Vector3(matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]));
}

/* Overloaded inequality operator */
inline bool Matrix33::operator!=(const Matrix33& matrix) const {
  return !(*this == matrix);
}

/* Overloaded addition operator with assignment */
inline Matrix33& Matrix33::operator+=(const Matrix33& matrix) {
  mRows[0][0] += matrix.mRows[0][0]; mRows[0][1] += matrix.mRows[0][1]; mRows[0][2] += matrix.mRows[0][2];
  mRows[1][0] += matrix.mRows[1][0]; mRows[1][1] += matrix.mRows[1][1]; mRows[1][2] += matrix.mRows[1][2];
  mRows[2][0] += matrix.mRows[2][0]; mRows[2][1] += matrix.mRows[2][1]; mRows[2][2] += matrix.mRows[2][2];
  return *this;
}

/* Overloaded subtraction operator with assignment */
inline Matrix33& Matrix33::operator-=(const Matrix33& matrix) {
  mRows[0][0] -= matrix.mRows[0][0]; mRows[0][1] -= matrix.mRows[0][1]; mRows[0][2] -= matrix.mRows[0][2];
  mRows[1][0] -= matrix.mRows[1][0]; mRows[1][1] -= matrix.mRows[1][1]; mRows[1][2] -= matrix.mRows[1][2];
  mRows[2][0] -= matrix.mRows[2][0]; mRows[2][1] -= matrix.mRows[2][1]; mRows[2][2] -= matrix.mRows[2][2];
  return *this;
}

/* Overloaded multiplication operator with assignment */
inline Matrix33& Matrix33::operator*=(float number) {
  mRows[0][0] *= number; mRows[0][1] *= number; mRows[0][2] *= number;
  mRows[1][0] *= number; mRows[1][1] *= number; mRows[1][2] *= number;
  mRows[2][0] *= number; mRows[2][1] *= number; mRows[2][2] *= number;
  return *this;
}

/* Overloaded value access operator */
inline Vector3& Matrix33::operator[](int row) {
  return mRows[row];
}

/* Overloaded value access operator */
inline const Vector3& Matrix33::operator[](int row) const {
  return mRows[row];
}

/* Overloaded operator for addition between two given matrices */
inline Matrix33 operator+(const Matrix33& matrix1, const Matrix33& matrix2) {
  return Matrix33(matrix1.mRows[0][0] + matrix2.mRows[0][0], matrix1.mRows[0][1] + matrix2.mRows[0][1], matrix1.mRows[0][2] + matrix2.mRows[0][2],
                  matrix1.mRows[1][0] + matrix2.mRows[1][0], matrix1.mRows[1][1] + matrix2.mRows[1][1], matrix1.mRows[1][2] + matrix2.mRows[1][2],
                  matrix1.mRows[2][0] + matrix2.mRows[2][0], matrix1.mRows[2][1] + matrix2.mRows[2][1], matrix1.mRows[2][2] + matrix2.mRows[2][2]);
}

/* Overloaded operator for subtraction between two given matrices */
inline Matrix33 operator-(const Matrix33& matrix1, const Matrix33& matrix2) {
  return Matrix33(matrix1.mRows[0][0] - matrix2.mRows[0][0], matrix1.mRows[0][1] - matrix2.mRows[0][1], matrix1.mRows[0][2] - matrix2.mRows[0][2],
                  matrix1.mRows[1][0] - matrix2.mRows[1][0], matrix1.mRows[1][1] - matrix2.mRows[1][1], matrix1.mRows[1][2] - matrix2.mRows[1][2],
                  matrix1.mRows[2][0] - matrix2.mRows[2][0], matrix1.mRows[2][1] - matrix2.mRows[2][1], matrix1.mRows[2][2] - matrix2.mRows[2][2]);
}

/* Overloaded operator for the negation of a given matrix */
inline Matrix33 operator-(const Matrix33& matrix) {
  return Matrix33(-matrix.mRows[0][0], -matrix.mRows[0][1], -matrix.mRows[0][2],
                  -matrix.mRows[1][0], -matrix.mRows[1][1], -matrix.mRows[1][2],
                  -matrix.mRows[2][0], -matrix.mRows[2][1], -matrix.mRows[2][2]);
}

/* Overloaded operator for multiplication of a given matrix with a given number */
inline Matrix33 operator*(float number, const Matrix33& matrix) {
  return Matrix33(matrix.mRows[0][0] * number, matrix.mRows[0][1] * number, matrix.mRows[0][2] * number,
                  matrix.mRows[1][0] * number, matrix.mRows[1][1] * number, matrix.mRows[1][2] * number,
                  matrix.mRows[2][0] * number, matrix.mRows[2][1] * number, matrix.mRows[2][2] * number);
}

/* Overloaded operator for multiplication of a given number with a given matrix */
inline Matrix33 operator*(const Matrix33& matrix, float number) {
  return number * matrix;
}

/* Overloaded operator for multiplication of a given matrix with a given vector */
inline Vector3 operator*(const Matrix33& matrix, const Vector3& vector) {
  return Vector3(matrix.mRows[0][0] * vector.x + matrix.mRows[0][1] * vector.y + matrix.mRows[0][2] * vector.z,
                 matrix.mRows[1][0] * vector.x + matrix.mRows[1][1] * vector.y + matrix.mRows[1][2] * vector.z,
                 matrix.mRows[2][0] * vector.x + matrix.mRows[2][1] * vector.y + matrix.mRows[2][2] * vector.z);
}

}

#endif