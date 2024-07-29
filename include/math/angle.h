#ifndef NEKOPHYSICS_ANGLE_H
#define NEKOPHYSICS_ANGLE_H

#include "math/const.h"

#include <cmath>

namespace neko
{

class Degree;
/**
 * \brief Radian is an utility class that describes radian angles (0 to 2PI).
 * It can be easily converted to Degree with conversion constructor.
 * It can be used with trigonometric functions (Sin, Cos, Tan)
 */
class Radian
{
public:
    constexpr Radian() = default;
    constexpr explicit Radian(Scalar value) : value_(value) {}
    /**
     * \brief Conversion constructor that implicitly converts Degree to Radian
     * \param angle is the degree angle to be converted to Radian
     */
    constexpr Radian(const Degree& angle);
    [[nodiscard]] constexpr Scalar value() const { return value_; }

    constexpr Radian operator+(Radian angle) const { return Radian{ value_ + angle.value() }; }
    constexpr Radian& operator+=(Radian angle)
    {
        value_ += angle.value();
        return *this;
    }
    constexpr Radian operator-(Radian angle) const { return Radian{ value_ - angle.value() }; }
    constexpr Radian& operator-=(Radian angle)
    {
        value_ -= angle.value();
        return *this;
    }
    constexpr Radian operator*(Scalar value) const { return Radian{ value_ * value }; }
    constexpr Radian operator/(Scalar value) const { return Radian{ value_ / value }; }
    constexpr Radian operator-() const { return Radian{ -value_ }; }
private:
    Scalar value_{};
};


constexpr Radian operator*(T value, Radian angle) { return Radian{ angle.value() * value }; }

/**
 * \brief Degree is an utility class that describes degree angles (0 to 360).
 * It can be easily converted to Radian with conversion constructor.
 * It can be used with trigonometric functions (Sin, Cos, Tan)
 */
class Degree
{
public:
    constexpr Degree() = default;
    constexpr explicit Degree(Scalar value) : value_(value) {}
    /**
     * \brief Conversion constructor that implicitly converts Radian to Degree
     * \param angle is the radian angle to be converted to Degree
     */
    constexpr Degree(const Radian& angle) : value_(angle.value() / Pi() * Scalar{180} {}
    [[nodiscard]] constexpr Scalar value() const { return value_; }
    constexpr Degree operator+(Degree angle) const { return Degree{ value_ + angle.value() }; }
    constexpr Degree& operator+=(Degree angle)
    {
        value_ += angle.value();
        return *this;
    }
    constexpr Degree operator-(Degree angle) const { return Degree{ value_ - angle.value() }; }
    constexpr Degree& operator-=(Degree angle)
    {
        value_ -= angle.value();
        return *this;
    }
    constexpr Degree operator-() const { return Degree{ -value_ }; }

    constexpr Degree operator*(Scalar value) const { return Degree{ value_ * value }; }
    constexpr Degree operator/(Scalar value) const { return Degree{ value_ / value }; }
private:
    Scalar value_{};
};

constexpr Degree operator*(Scalar value, Degree angle) { return Degree{ angle.value() * value }; }


constexpr Radian::Radian(const Degree& angle)
{
    value_ = angle.value() / Scalar{180} * Pi();
}

/**
 * \brief Sin is a function that calculates the sinus of a given angle.
 * \param angle is the given angle
 * \return the result of the sinus of angle
 */
inline Scalar Sin(Radian angle)
{
    return Sin(angle.value());
}

/**
 * \brief Cos is a function that calculates the cosinus of a given angle.
 * \param angle is the given angle
 * \return the result of the cosinus of angle
 */
inline Scalar Cos(Radian angle)
{
    return Cos(angle.value());
}

/**
 * \brief Tan is a function that calculates the tangent of a given angle.
 * \param angle is the given angle
 * \return the result of the tangent of angle
 */
inline Scalar Tan(Radian angle)
{
    return Tan(angle.value());
}
}

#endif //NEKOLIB_ANGLE_H
