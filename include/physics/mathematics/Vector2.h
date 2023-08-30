#ifndef PHYSICS_VECTOR2_H
#define PHYSICS_VECTOR2_H

#include <cassert>
#include <physics/mathematics/MathCommon.h>
#include <physics/Configuration.h>

namespace physics {

struct Vector2 {
  
  public:
    /* -- Attributes -- */

    /* x component */
    float x;

    /* y component */
    float y;

    /* -- Methods -- */
    
    /* Constructor */
    Vector2();

    /* Constructor with parameters */
    Vector2(float newX, float newY);

    /* Set the x and y components of the vector */
    void set(float newX, float newY);

    /* Set the vector as the zero vector */
    void setZero();

    /* Get the length of the vector */
    float length() const;

    /* Get the square of the length of the vector */
    float lengthSquare() const;
    
    /* Get the associated unit vector of the current vector */
    Vector2 getUnitVector() const;

    /* Query whether the current vector is a unit vector */
    bool isUnitVector() const;

    /* Query whether the component values are either NaN or Inf */
    bool isFiniteVector() const;

    /* Query whether the current vector is the zero vector */
    bool isZeroVector() const;

    /* Compute the dot product of the current vector with another given vector */
    float dot(const Vector2& vector) const;

    /* Normalize the vector */
    void normalize();
    
    /* Overloaded equality operator */
    bool operator==(const Vector2& vector) const;

    /* Overloaded inequality operator */
    bool operator!=(const Vector2& vector) const;

    /* Overloaded addition operator */
    Vector2& operator+=(const Vector2& vector);

    /* Overloaded subtraction operator */
    Vector2& operator-=(const Vector2& vector);

    /* Overloaded multiplication operator */
    Vector2& operator*=(float number);

    /* Overloaded division operator */
    Vector2& operator/=(float number);

    /* Overloaded value access operator */
    float& operator[](int index);

    /* Overloaded value access operator */
    const float& operator[](int index) const;

    /* Overloaded less than operator */
    bool operator<(const Vector2& vector) const;

    /* -- Friends -- */

    friend Vector2 operator+(const Vector2& vector1, const Vector2& vector2);
    friend Vector2 operator-(const Vector2& vector1, const Vector2& vector2);
    friend Vector2 operator-(const Vector2& vector);
    friend Vector2 operator*(const Vector2& vector, float number);
    friend Vector2 operator*(float number, const Vector2& vector);
    friend Vector2 operator*(const Vector2& vector1, const Vector2& vector2);
    friend Vector2 operator/(const Vector2& vector, float number);
    friend Vector2 operator/(const Vector2& vector1, const Vector2& vector2);
};

/* Constructor */
inline Vector2::Vector2() : x(0.0f), y(0.0f) {}

/* Constructor with parameters */
inline Vector2::Vector2(float newX, float newY) : x(newX), y(newY) {}

/* Set the x and y components of the vector */
inline void Vector2::set(float newX, float newY) {
  x = newX;
  y = newY;
}

/* Set the vector as the zero vector */
inline void Vector2::setZero() {
  x = 0.0f;
  y = 0.0f;
}

 /* Get the length of the vector */
inline float Vector2::length() const {
  return std::sqrt(x * x + y * y);
}

/* Get the square of the length of the vector */
inline float Vector2::lengthSquare() const {
  return x * x + y * y;
}

/* Query whether the current vector is a unit vector */
inline bool Vector2::isUnitVector() const {
  return approximateEqual(lengthSquare(), 1.0f);
}

/* Query whether the component values are either NaN or Inf */
inline bool Vector2::isFiniteVector() const {
  return std::isfinite(x) && std::isfinite(y);
}

/* Query whether the current vector is the zero vector */
inline bool Vector2::isZeroVector() const {
  return approximateEqual(lengthSquare(), 0.0f);
}

/* Compute the dot product of the current vector with another given vector */
inline float Vector2::dot(const Vector2& vector) const {
  return (x * vector.x + y * vector.y);
}

/* Normalize the vector */
inline void Vector2::normalize() {
  float len = length();

  if(len < MACHINE_EPSILON) {
    return;
  }

  x /= len;
  y /= len;
}

/* Overloaded equality operator */
inline bool Vector2::operator==(const Vector2& vector) const {
  return (approximateEqual(x, vector.x) && approximateEqual(y, vector.y));
}

/* Overloaded inequality operator */
inline bool Vector2::operator!=(const Vector2& vector) const {
  return !(*this == vector);
}

/* Overloaded addition operator with assignment */
inline Vector2& Vector2::operator+=(const Vector2& vector) {
    x += vector.x;
    y += vector.y;
    return *this;
}

/* Overloaded subtraction operator with assignment */
inline Vector2& Vector2::operator-=(const Vector2& vector) {
    x -= vector.x;
    y -= vector.y;
    return *this;
}

/* Overloaded multiplication operator with assignment */
inline Vector2& Vector2::operator*=(float number) {
    x *= number;
    y *= number;
    return *this;
}

/* Overloaded division operator with assignment */
inline Vector2& Vector2::operator/=(float number) {
    assert(number > std::numeric_limits<float>::epsilon());
    x /= number;
    y /= number;
    return *this;
}

/* Overloaded value access operator */
inline float& Vector2::operator[](int index) {
    return (&x)[index];
}

/* Overloaded value access operator */
inline const float& Vector2::operator[](int index) const {
    return (&x)[index];
}

/* Overloaded less than operator */
inline bool Vector2::operator<(const Vector2& vector) const {
    return (x == vector.x ? y < vector.y : x < vector.x);
}

/* Overloaded operator for addition between two given vectors */
inline Vector2 operator+(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(vector1.x + vector2.x, vector1.y + vector2.y);
}

/* Overloaded operator for subtraction between two given vectors */
inline Vector2 operator-(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(vector1.x - vector2.x, vector1.y - vector2.y);
}

/* Overloaded operator for the negation of a given vector */
inline Vector2 operator-(const Vector2& vector) {
    return Vector2(-vector.x, -vector.y);
}

/* Overloaded operator for multiplication of a given vector with a given number */
inline Vector2 operator*(const Vector2& vector, float number) {
    return Vector2(number * vector.x, number * vector.y);
}

/* Overloaded operator for multiplication of a given number with a given vector */
inline Vector2 operator*(float number, const Vector2& vector) {
    return vector * number;
}

/* Overloaded operator for multiplication of two given vectors */
inline Vector2 operator*(const Vector2& vector1, const Vector2& vector2) {
    return Vector2(vector1.x * vector2.x, vector1.y * vector2.y);
}

/* Overloaded operator for division between two given vectors */
inline Vector2 operator/(const Vector2& vector1, const Vector2& vector2) {
    assert(vector2.x > MACHINE_EPSILON);
    assert(vector2.y > MACHINE_EPSILON);
    return Vector2(vector1.x / vector2.x, vector1.y / vector2.y);
}

/* Overloaded operator for division of a given vector by a given number */
inline Vector2 operator/(const Vector2& vector, float number) {
    assert(number > MACHINE_EPSILON);
    return Vector2(vector.x / number, vector.y / number);
}

/* Query whether two functions are approximately equal */
inline bool approximateEqual(const Vector2& vector1, const Vector2& vector2, float epsilon = MACHINE_EPSILON) {
  return approximateEqual(vector1.x, vector2.x, epsilon) && approximateEqual(vector1.y, vector2.y, epsilon);
}

}

#endif