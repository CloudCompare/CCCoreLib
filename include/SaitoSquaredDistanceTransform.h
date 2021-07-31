// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © CloudCompare Project

#pragma once

// Inspired from bil_edt.cxx (VXL) by Ricardo Fabbri (rfabbri), Brown University  (rfabbri@lems.brown.edu)

//Local
#include "Grid3D.h"
#include "MathTools.h"

namespace CCCoreLib
{
	class GenericProgressCallback;
	class NormalizedProgress;
	class GenericIndexedMesh;
	class GenericCloud;

	//! Class to compute a Squared Distance Field with the Saito algorithm on a 3D grid
	class CC_CORE_LIB_API SaitoSquaredDistanceTransform : public Grid3D<unsigned>, public MathTools
	{
	public:

		//! Default constructor
		SaitoSquaredDistanceTransform() = default;

		//! Initializes the grid
		/** The memory for the grid must be explicitelty reserved prior to any action.
			\return true if the initialization succeeded
		**/
		inline bool initGrid(const Tuple3ui& gridSize)
		{
			return Grid3D<GridElement>::init(gridSize.x, gridSize.y, gridSize.z, 0, 0); //margin = 0 (we need continuous memory)
		}

		//! Initializes the distance transform with a mesh
		inline bool initDT(	GenericIndexedMesh* mesh,
							PointCoordinateType cellLength,
							const CCVector3& gridMinCorner,
							GenericProgressCallback* progressCb = nullptr)
		{
			return intersectWith(mesh, cellLength, gridMinCorner, 1, progressCb);
		}

		//! Initializes the distance transform with a cloud
		inline bool initDT(	GenericCloud* cloud,
							PointCoordinateType cellLength,
							const CCVector3& gridMinCorner,
							GenericProgressCallback* progressCb = nullptr)
		{
			return intersectWith(cloud, cellLength, gridMinCorner, 1, progressCb);
		}

		//! Computes the exact Squared Distance Transform on the whole grid
		/** Propagates the distances on the whole grid.

			Base on the implementation by R. Fabbri (which is itslef based on
			two independent implementations by O. Cuisenaire and J. C. Torelli).

			PAPER
			T. Saito and J.I. Toriwaki, "New algorithms for Euclidean distance
			transformations of an n-dimensional digitised picture with applications",
			Pattern Recognition, 27(11), pp. 1551-1565, 1994

			\warning Output distances are squared

			\param progressCb progress callback (optional)
			\return success
		**/
		inline bool propagateDistance(GenericProgressCallback* progressCb = nullptr) { return SDT_3D(*this, progressCb); }

	protected:

		//! 1D Euclidean Distance Transform
		static bool EDT_1D(GridElement* slice, std::size_t r, std::size_t c);
		//! 2D Exact Squared Distance Transform
		static bool SDT_2D(Grid3D<GridElement>& image, std::size_t sliceIndex, const std::vector<GridElement>& sq);
		//! 3D Exact Squared Distance Transform
		static bool SDT_3D(Grid3D<GridElement>& image, GenericProgressCallback* progressCb = nullptr);
	};
}
