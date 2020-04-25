// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//! Type of the coordinates of a (N-D) point
using PointCoordinateType = float;

//! Type of a single scalar field value
#if defined SCALAR_TYPE_DOUBLE
using ScalarType = double;
#elif defined SCALAR_TYPE_FLOAT
using ScalarType = float;
#else
static_assert(false, "type for ScalarType has not been declared");
#endif //SCALAR_TYPE_DOUBLE
