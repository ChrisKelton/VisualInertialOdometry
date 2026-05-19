#ifndef EIGENQUATERNIONPARAMETERIZATION_H
#define EIGENQUATERNIONPARAMETERIZATION_H

#include "ceres/manifold.h"

namespace camodocal
{

// Ceres 2.x removed LocalParameterization; use the built-in EigenQuaternionManifold instead.
using EigenQuaternionParameterization = ceres::EigenQuaternionManifold;

}

#endif

