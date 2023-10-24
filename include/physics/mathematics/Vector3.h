#ifndef PHYSICS_VECTOR3_H
#define PHYSICS_VECTOR3_H

#include <cassert>
#include <cmath>
#include <algorithm>
#include <physics/mathematics/MathCommon.h>
#include <physics/Configuration.h>

namespace physics {

struct Vector3 {

  public:
    /* -- Attributes -- */

    /* x component */
    float x;

    /* y component */
    float y;

    /* z component */
    float z;

    /* -- Methods -- */

    /* Constructor */
    Vector3();

    /* Constructor with parameters */
    Vector3(float newX, float newY, float newZ);

    /* Set the x, y and z components of the vector */
    void set(float newX, float newY, float newZ);

    /* Set the vector as the zero vector */
    void setZero();

    /* Get the length of the vector */
    float length() const;

    /* Get the square of the length of the vector */
    float lengthSquare() const;

    /* Get the associated unit vector of the current vector */
    Vector3 getUnitVector() const;

    /* Query whether the current vector is a unit vector */
    bool isUnitVector() const;

    /* Query whether the component values are either NaN or Inf */
    bool isFiniteVector() const;

    /* Query whether the current vector is the zero vector */
    bool isZeroVector() const;

    /* Compute the dot product of the current vector with another given vector */
    float dot(const Vector3& vector) const;

    /* Compute the cross product of the current vector with another given vector */
    Vector3 cross(const Vector3& vector) const;

    /* Normalize the vector */
    void normalize();

    /* Overloaded equality operator */
    bool operator==(const Vector3& vector) const;

    /* Overloaded inequality operator */
    bool operator!=(const Vector3& vector) const;

    /* Overloaded addition operator with assignment */
    Vector3& operator+=(const Vector3& vector);

    /* Overloaded subtraction operator with assignment */
    Vector3& operator-=(const Vector3& vector);

    /* Overloaded multiplication operator with assignment */
    Vector3& operator*=(float number);

    /* Overloaded division operator with assignment */
    Vector3& operator/=(float number);

    /* Overloaded value access operator */
    float& operator[](int index);

    /* Overloaded value access operator */
    const float& operator[](int index) const;

    /* Overloaded less than operator */
    bool operator<(const Vector3& vector) const;

    /* -- Friends -- */

    friend Vector3 operator+(const Vector3& vector1, const Vector3& vector2);
    friend Vector3 operator-(const Vector3& vector1, const Vector3& vector2);
    friend Vector3 operator-(const Vector3& vector);
    friend Vector3 operator*(const Vector3& vector, float number);
    friend Vector3 operator*(float number, const Vector3& vector);
    friend Vector3 operator*(const Vector3& vector1, const Vector3& vector2);
    friend Vector3 operator/(const Vector3& vector, float number);
    friend Vector3 operator/(const Vector3& vector1, const Vector3& vector3);
};

/* Constructor */
inline Vector3::Vector3() : x(0), y(0), z(0) {}

/* Constructor with parameters */
inline Vector3::Vector3(float newX, float newY, float newZ) : x(newX), y(newY), z(newZ) {}

/* Set the x and y components of the vector */
inline void Vector3::set(float newX, float newY, float newZ) {
  x = newX;
  y = newY;
  z = newZ;
}

/* Set the vector as the zero vector */
inline void Vector3::setZero() {
  x = 0;
  y = 0;
  z = 0;
}

 /* Get the length of the vector */
inline float Vector3::length() const {
  return std::sqrt(x * x + y * y + z * z);
}

/* Get the square of the length of the vector */
inline float Vector3::lengthSquare() const {
  return x * x + y * y + z * z;
}

/* Get the associated unit vector of the current vector */
inline Vector3 Vector3::getUnitVector() const {
  float len = length();

  if(len < FLOAT_EPSILON) {
    return *this;
  }

  float lenInverse = 1.0f / len;
  
  return Vector3(x * lenInverse, y * lenInverse, z * lenInverse);
}

/* Query whether the current vector is a unit vector */
inline bool Vector3::isUnitVector() const {
  return approximateEqual(lengthSquare(), 1);
}

/* Query whether the component values are either NaN or Inf */
inline bool Vector3::isFiniteVector() const {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/* Query whether the current vector is the zero vector */
inline bool Vector3::isZeroVector() const {
  return approximateEqual(lengthSquare(), 0);
}

/* Compute the dot product of the current vector with another given vector */
inline float Vector3::dot(const Vector3& vector) const {
  return x * vector.x + y * vector.y + z * vector.z;
}

/* Compute the cross product of the current vector with another given vector */
inline Vector3 Vector3::cross(const Vector3& vector) const {
  return Vector3(y * vector.z - z * vector.y,
                 z * vector.x - x * vector.z,
                 x * vector.y - y * vector.x);
}

/* Normalize the vector */
inline void Vector3::normalize() {
  float len = length();

  if(len < FLOAT_EPSILON) {
    return;
  }

  x /= len;
  y /= len;
  z /= len;
}

/* Overloaded equality operator */
inline bool Vector3::operator==(const Vector3& vector) const {
  return approximateEqual(x, vector.x) && approximateEqual(y, vector.y) && approximateEqual(z, vector.z);
}

/* Overloaded inequality operator */
inline bool Vector3::operator!=(const Vector3& vector) const {
  return !(*this == vector);
}

/* Overloaded addition operator with assignment */
inline Vector3& Vector3::operator+=(const Vector3& vector) {
    x += vector.x;
    y += vector.y;
    z += vector.z;
    return *this;
}

/* Overloaded subtraction operator with assignment */
inline Vector3& Vector3::operator-=(const Vector3& vector) {
    x -= vector.x;
    y -= vector.y;
    z -= vector.z;
    return *this;
}

/* Overloaded multiplication operator with assignment */
inline Vector3& Vector3::operator*=(float number) {
    x *= number;
    y *= number;
    z *= number;
    return *this;
}

/* Overloaded division operator with assignment */
inline Vector3& Vector3::operator/=(float number) {
    assert(number > std::numeric_limits<float>::epsilon());
    x /= number;
    y /= number;
    z /= number;
    return *this;
}

/* Overloaded value access operator */
inline float& Vector3::operator[](int index) {
    return (&x)[index];
}

/* Overloaded value access operator */
inline const float& Vector3::operator[](int index) const {
    return (&x)[index];
}

/* Overloaded less than operator */
inline bool Vector3::operator<(const Vector3& vector) const {
    return approximateEqual(x, vector.x) ? approximateEqual(y, vector.y) ? z < vector.z : y < vector.y : x < vector.x;
}

/* Overloaded operator for addition between two given vectors */
inline Vector3 operator+(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z);
}

/* Overloaded operator for subtraction between two given vectors */
inline Vector3 operator-(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z);
}

/* Overloaded operator for the negation of a given vector */
inline Vector3 operator-(const Vector3& vector) {
    return Vector3(-vector.x, -vector.y, -vector.z);
}

/* Overloaded operator for multiplication of a given vector with a given number */
inline Vector3 operator*(const Vector3& vector, float number) {
    return Vector3(number * vector.x, number * vector.y, number * vector.z);
}

/* Overloaded operator for multiplication of a given number with a given vector */
inline Vector3 operator*(float number, const Vector3& vector) {
    return vector * number;
}

/* Overloaded operator for multiplication of two given vectors */
inline Vector3 operator*(const Vector3& vector1, const Vector3& vector2) {
    return Vector3(vector1.x * vector2.x, vector1.y * vector2.y, vector1.z * vector2.z);
}

/* Overloaded operator for division between two given vectors */
inline Vector3 operator/(const Vector3& vector1, const Vector3& vector2) {
    assert(vector2.x > FLOAT_EPSILON);
    assert(vector2.y > FLOAT_EPSILON);
    return Vector3(vector1.x / vector2.x, vector1.y / vector2.y, vector1.z / vector2.z);
}

/* Overloaded operator for division of a given vector by a given number */
inline Vector3 operator/(const Vector3& vector, float number) {
    assert(number > FLOAT_EPSILON);
    return Vector3(vector.x / number, vector.y / number, vector.z / number);
}

}

#endif