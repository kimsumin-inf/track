//
// Created by sumin on 22. 9. 5.
//

#ifndef SRC_TRIGONOMETRIC_H
#define SRC_TRIGONOMETRIC_H
#include <cmath>

static inline double normalize(const double& rad)
{
    return std::atan2(sin(rad), cos(rad));
}

static inline double toRadian(const double& deg)
{
    return deg * M_PI / 180;
}

static inline double toDegree(const double& rad)
{
    return rad * 180 / M_PI;
}

#endif //SRC_TRIGONOMETRIC_H
