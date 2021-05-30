// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

#include "CCTypes.h"

//system
#include <cmath>
#include <limits>

//! Pi - outside namespace because it's mostly-standard
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

//! Pi/2 - outside namespace because it's mostly-standard
#ifndef M_PI_2
constexpr double M_PI_2 = (M_PI / 2);
#endif

namespace CCCoreLib
{
	//! Square root of 3
	constexpr double SQRT_3 = 1.7320508075688772935274463415059;
	
	//! ZERO_TOLERANCE_F is used to set or compare a float variable to "close to zero".
	constexpr float ZERO_TOLERANCE_F = std::numeric_limits<float>::epsilon();
	
	//! ZERO_TOLERANCE_D is used to set or compare a double variable to "close to zero".
	//! It is defined as std::numeric_limits<float>::epsilon() because using
	//! std::numeric_limits<double>::epsilon() results in numbers that are too small for our purposes.
	constexpr double ZERO_TOLERANCE_D = static_cast<double>(ZERO_TOLERANCE_F);
	
	//! ZERO_SQUARED_TOLERANCE_D is used to set or compare a (square) value to "close to zero".
	constexpr double ZERO_SQUARED_TOLERANCE_D = ZERO_TOLERANCE_D * ZERO_TOLERANCE_D;

	//! ZERO_TOLERANCE_SCALAR is used to set or compare a ScalarType variable to "close to zero".
	constexpr ScalarType ZERO_TOLERANCE_SCALAR = std::numeric_limits<ScalarType>::epsilon();
	
	//! ZERO_TOLERANCE_POINT_COORDINATE is used to set or compare a PointCoordinateType variable to "close to zero".
	constexpr ScalarType ZERO_TOLERANCE_POINT_COORDINATE = std::numeric_limits<PointCoordinateType>::epsilon();
	
	//! '1' as a PointCoordinateType value
	/** To avoid compiler warnings about 'possible loss of data' **/
	constexpr PointCoordinateType PC_ONE = static_cast<PointCoordinateType>(1.0);
	
	//! 'NaN' as a PointCoordinateType value
	/** \warning: handle with care! **/
	constexpr PointCoordinateType PC_NAN = std::numeric_limits<PointCoordinateType>::quiet_NaN();
	
	//! NaN as a ScalarType value
	/** \warning: handle with care! **/
	constexpr ScalarType NAN_VALUE = std::numeric_limits<ScalarType>::quiet_NaN();
	
	// Point visibility states
	// By default visibility is expressed relative to the sensor point of view.
	// Warning: 'visible' value must always be the lowest!
	constexpr unsigned char POINT_VISIBLE			=	0;	//!< Point visibility state: visible
	constexpr unsigned char POINT_HIDDEN			=	1;	//!< Point visibility state: hidden (e.g. behind other points)
	constexpr unsigned char POINT_OUT_OF_RANGE		=	2;	//!< Point visibility state: out of range
	constexpr unsigned char POINT_OUT_OF_FOV		=	4;	//!< Point visibility state: out of field of view
	
	//! Chamfer distances types
	enum CHAMFER_DISTANCE_TYPE {
		CHAMFER_111		= 0,	//!< Chamfer distance <1-1-1>
		CHAMFER_345		= 1		//!< Chamfer distance <3-4-5>
	};
	
	//! Types of local models (no model, least square best fitting plan, Delaunay 2D1/2 triangulation, height function)
	enum LOCAL_MODEL_TYPES {
		NO_MODEL			= 0,	//!< No local model
		LS					= 1,	//!< Least Square best fitting plane
		TRI					= 2,	//!< 2.5D Delaunay triangulation
		QUADRIC				= 3		//!< 2.5D quadric function
	};
	
	//! Min number of points to compute local models (see CC_LOCAL_MODEL_TYPES)
	constexpr unsigned CC_LOCAL_MODEL_MIN_SIZE[] = {
		1,	//!< for single point model (i.e. no model ;)
		3,	//!< for least Square best fitting plane
		3,	//!< for Delaunay triangulation (2.5D)
		6,	//!< for Quadratic 'height' function
	};
}
