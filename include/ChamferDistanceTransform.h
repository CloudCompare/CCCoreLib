// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "Grid3D.h"
#include "MathTools.h"

namespace CCCoreLib
{

	class GenericProgressCallback;
	class NormalizedProgress;

	//! Class to compute a Chamfer distance field on a 3D grid
	/** Internally we use 'unsigned short' value to limit memory consumption.
		For computational reasons, the max computable 'distance' is 0xFAFA = 64250.
	**/
	class CC_CORE_LIB_API ChamferDistanceTransform : public Grid3D<unsigned short>, public MathTools
	{
	public:

		//! Max possible 'distance'
		/** \warning Never pass a 'constant initializer' by reference
		**/
		static const unsigned short MAX_DIST = 0xFAFA;

		//! Initializes the grid
		/** 'Zero' cells must be initialized with setValue(0).
			The grid must be explicitelty initialized prior to any action.
			\return true if the initialization succeeded
		**/
		inline bool init(const Tuple3ui& gridSize)
		{
			return Grid3D<GridElement>::init(gridSize.x, gridSize.y, gridSize.z, 1, MAX_DIST);
		}

		//! Computes the Chamfer distance on the whole grid
		/** Propagates the distances on the whole grid. The 'zeros' should
			have already been initialized before calling this method (see
			ChamferDistanceTransform::setZero).
			\param type the Chamfer distance type
			\param progressCb the client application can get some notification of the process
				progress through this callback mechanism (see GenericProgressCallback)
			\return max distance (or -1 if an error occurred)
		**/
		int propagateDistance(CHAMFER_DISTANCE_TYPE type, GenericProgressCallback* progressCb = nullptr);
	};

}
