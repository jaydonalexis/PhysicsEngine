#ifndef PHYSICS_MATRIX22_H
#define PHYSICS_MATRIX22_H

#include <cassert>
#include <physics/mathematics/Vector2.h>

namespace physics {

class Matrix22 {

  private:
    /* -- Attributes -- */

    /* Matrix rows */
    Vector2 mRows[2];

  public:
    /* -- Methods -- */
    
    /* Constructor */
    Matrix22();

    /* Constructor with single parameter */
    Matrix22(float value);

    /* Constructor with multiple parameters */
    Matrix22(float a, float b,
             float c, float d);

    /* Set components of the matrix with single parameter */
    void set(float value);

    /* Set components of the matrix with multiple parameters */
    void set(float a, float b,
             float c, float d);

    /* Set components of the matrix to zero */
    void setZero();

    /* Set matrix to identity matrix */
    void setIdentity();

    /* Get a column of the matrix */
    Vector2 getColumn(int i) const;

    /* Get a row of the matrix */
    Vector2 getRow(int i) const;

    /* Get transpose of the matrix */
    Matrix22 getTranspose() const;

    /* Get determinant of the matrix */
    float getDeterminant() const;

    /* Get inverse of the matrix */
    Matrix22 getInverse() const;

    /* Get identity matrix */
    static Matrix22 getIdentity();

    /* Get zero matrix */
    static Matrix22 getZero();

    /* Overloaded equality operator */
    bool operator==(const Matrix22& matrix) const;

    /* Overloaded inequality operator */
    bool operator!=(const Matrix22& matrix) const;

    /* Overloaded addition operator with assignment */
    Matrix22& operator+=(const Matrix22& matrix);

    /* Overloaded subtraction operator with assignment */
    Matrix22& operator-=(const Matrix22& matrix);

    /* Overloaded multiplication operator with assignment */
    Matrix22& operator*=(float number);

    /* Overloaded value access operator */
    Vector2& operator[](int row);

    /* Overloaded value access operator */
    const Vector2& operator[](int row) const;

    /* -- Friends -- */

    friend Matrix22 operator+(const Matrix22& matrix1, const Matrix22& matrix2);
    friend Matrix22 operator-(const Matrix22& matrix1, const Matrix22& matrix2);
    friend Matrix22 operator-(const Matrix22& matrix);
    friend Matrix22 operator*(float number, const Matrix22& matrix);
    friend Matrix22 operator*(const Matrix22& matrix, float number);
    friend Vector2 operator*(const Matrix22& matrix, const Vector2& vector);
};

/* Constructor */
inline Matrix22::Matrix22() {
  set(0.0f);
}

/* Constructor with single parameter */
inline Matrix22::Matrix22(float value) {
  set(value);
}

/* Constructor with multiple parameters */
inline Matrix22::Matrix22(float a, float b, 
                          float c, float d) {
  set(a, b, c, d);
}

/* Set components of the matrix with single parameter */
inline void Matrix22::set(float value) {
  mRows[0][0] = value; mRows[0][1] = value;
  mRows[1][0] = value; mRows[1][1] = value;
}

/* Set components of the matrix with multiple parameters */
inline void Matrix22::set(float a, float b,
                          float c, float d) {
  mRows[0][0] = a; mRows[0][1] = b;
  mRows[1][0] = c; mRows[1][1] = d;
}

 /* Set components of the matrix to zero */
inline void Matrix22::setZero() {
  mRows[0].setZero();
  mRows[1].setZero();
}

/* Set matrix to identity matrix */
inline void Matrix22::setIdentity() {
  mRows[0][0] = 1.0f; mRows[0][1] = 0.0f;
  mRows[1][0] = 0.0f; mRows[1][1] = 1.0f;
}

/* Get a column of the matrix */
inline Vector2 Matrix22::getColumn(int i) const {
  assert(i >= 0 && i < 2);
  return Vector2(mRows[0][i], mRows[1][i]);
}

/* Get a row of the matrix */
inline Vector2 Matrix22::getRow(int i) const {
  assert(i >= 0 && i < 2);
  return mRows[i];
}

/* Get transpose of the matrix */
inline Matrix22 Matrix22::getTranspose() const {
  return Matrix22(mRows[0][0], mRows[1][0],
                  mRows[0][1], mRows[1][1]);
}

/* Get determinant of the matrix */
inline float Matrix22::getDeterminant() const {
  return mRows[0][0] * mRows[1][1] - mRows[1][0] * mRows[0][1];
}

/* Get identity matrix */
inline Matrix22 Matrix22::getIdentity() {
  return Matrix22(1.0f, 0.0f, 0.0f, 1.0f);
}

/* Get zero matrix */
inline Matrix22 Matrix22::getZero() {
  return Matrix22(0.0f, 0.0f, 0.0f, 0.0f);
}

/* Overloaded equality operator */
inline bool Matrix22::operator==(const Matrix22& matrix) const {
  return (mRows[0] == Vector2(matrix.mRows[0][0], matrix.mRows[0][1]),
          mRows[1] == Vector2(matrix.mRows[1][0], matrix.mRows[1][1]));
}

/* Overloaded inequality operator */
inline bool Matrix22::operator!=(const Matrix22& matrix) const {
    return !(*this == matrix);
}

/* Overloaded addition operator with assignment */
inline Matrix22& Matrix22::operator+=(const Matrix22& matrix) {
  mRows[0][0] += matrix.mRows[0][0]; mRows[0][1] += matrix.mRows[0][1];
  mRows[1][0] += matrix.mRows[1][0]; mRows[1][1] += matrix.mRows[1][1];
  return *this;
}

/* Overlaoded subtraction operator with assignment */
inline Matrix22& Matrix22::operator-=(const Matrix22& matrix) {
  mRows[0][0] -= matrix.mRows[0][0]; mRows[0][1] -= matrix.mRows[0][1];
  mRows[1][0] -= matrix.mRows[1][0]; mRows[1][1] -= matrix.mRows[1][1];
  return *this;
}

/* Overloaded multiplication operator with assignment */
inline Matrix22& Matrix22::operator*=(float number) {
  mRows[0][0] *= number; mRows[0][1] *= number;
  mRows[1][0] *= number; mRows[1][1] *= number;
  return *this;
}

/* Overloaded value access operator */
inline Vector2& Matrix22::operator[](int row) {
  return mRows[row];
}

/* Overloaded value access operator */
inline const Vector2& Matrix22::operator[](int row) const {
  return mRows[row];
}

/* Overloaded operator for addition between two given matrices */
inline Matrix22 operator+(const Matrix22& matrix1, const Matrix22& matrix2) {
  return Matrix22(matrix1.mRows[0][0] + matrix2.mRows[0][0], matrix1.mRows[0][1] + matrix2.mRows[0][1],
                  matrix1.mRows[1][0] + matrix2.mRows[1][0], matrix1.mRows[1][1] + matrix2.mRows[1][1]);
}

/* Overloaded operator for subtraction between two given matrices */
inline Matrix22 operator-(const Matrix22& matrix1, const Matrix22& matrix2) {
  return Matrix22(matrix1.mRows[0][0] - matrix2.mRows[0][0], matrix1.mRows[0][1] - matrix2.mRows[0][1],
                  matrix1.mRows[1][0] - matrix2.mRows[1][0], matrix1.mRows[1][1] - matrix2.mRows[1][1]);
}

/* Overloaded operator for the negation of a given matrix */
inline Matrix22 operator-(const Matrix22& matrix) {
  return Matrix22(-matrix.mRows[0][0], -matrix.mRows[0][1],
                  -matrix.mRows[1][0], -matrix.mRows[1][1]);
}

/* Overloaded operator for multiplication of a given matrix with a given number */
inline Matrix22 operator*(float number, const Matrix22& matrix) {
  return Matrix22(matrix.mRows[0][0] * number, matrix.mRows[0][1] * number,
                  matrix.mRows[1][0] * number, matrix.mRows[1][1] * number);
}

/* Overloaded operator for multiplication of a given number with a given matrix */
inline Matrix22 operator*(const Matrix22& matrix, float number) {
  return number * matrix;
}

/* Overloaded operator for multiplication of a given matrix with a given vector */
inline Vector2 operator*(const Matrix22& matrix, const Vector2& vector) {
  return Vector2(matrix.mRows[0][0] * vector.x + matrix.mRows[0][1] * vector.y,
                 matrix.mRows[1][0] * vector.x + matrix.mRows[1][1] * vector.y);
}

}

#endif