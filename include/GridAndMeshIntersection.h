// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "CCConst.h"
#include "Grid3D.h"

namespace CCCoreLib
{
	class GenericIndexedMesh;
	class GenericProgressCallback;
	class SaitoSquaredDistanceTransform;
	struct TriangleList;

	//! Structure to compute the intersection between a mesh and a grid (to compute fast distances)
	class CC_CORE_LIB_API GridAndMeshIntersection
	{
	public:

		//! Default constructor
		GridAndMeshIntersection();

		//! Destructor
		virtual ~GridAndMeshIntersection();

		//! Clears the structure
		void clear();

		//! Returns whether the structure is initialized or not
		inline bool isInitialized() const { return m_initialized; }

		//! Initializes the structure with a mesh
		bool computeMeshIntersection(	GenericIndexedMesh* mesh,
										const CCVector3& localMinGridBB,
										PointCoordinateType cellSize,
										GenericProgressCallback* progressCb = nullptr);

		//! Initializes the structure with a mesh
		bool initDistanceTransformWithMesh(	GenericIndexedMesh* mesh,
											const CCVector3& localMinGridBB,
											const CCVector3& minFilledBB,
											const CCVector3& maxFilledBB,
											PointCoordinateType cellSize,
											GenericProgressCallback* progressCb = nullptr);

		//! Computes the (grid) cell position that contains a given point
		inline Tuple3i computeCellPos(const CCVector3& localP) const
		{
			//DGM: if we admit that cellSize > 0, then the 'floor' operator is useless (int cast = truncation)
			assert(m_cellSize > 0);

			Tuple3i cellPos(static_cast<int>(/*floor*/(localP.x - m_localMinGridBB.x) / m_cellSize),
							static_cast<int>(/*floor*/(localP.y - m_localMinGridBB.y) / m_cellSize),
							static_cast<int>(/*floor*/(localP.z - m_localMinGridBB.z) / m_cellSize));

			return cellPos;
		}

		//! Computes the center of a given (grid) cell
		/** \param cellPos	the (grid) cell position
			\param center	the computed center
		**/
		inline void computeCellCenter(const Tuple3i& cellPos, CCVector3& localCenter) const
		{
			localCenter.x = m_localMinGridBB.x + (m_cellSize * (cellPos.x + static_cast<PointCoordinateType>(0.5)));
			localCenter.y = m_localMinGridBB.y + (m_cellSize * (cellPos.y + static_cast<PointCoordinateType>(0.5)));
			localCenter.z = m_localMinGridBB.z + (m_cellSize * (cellPos.z + static_cast<PointCoordinateType>(0.5)));
		}

		//! Returns the associated mesh (if any)
		inline const GenericIndexedMesh* mesh() const { return m_mesh; }

		//! Returns the distance transform (if any)
		inline const SaitoSquaredDistanceTransform* distanceTransform() const { return m_distanceTransform; }

		//! Returns the distance transform value (if any)
		unsigned distanceTransformValue(const Tuple3i& cellPos, bool isCellPosRelativeToGridCorner) const;

		//! Returns the virtual grid cell size
		inline PointCoordinateType cellSize() const { return m_cellSize; }

		//! Converts a global cell position to a local one
		inline Tuple3i toLocal(const Tuple3i& cellPos) const { return cellPos - m_minFillIndexes; }

		//! Returns the list of triangles intersecting a given cell
		const TriangleList* trianglesInCell(const Tuple3i& cellPos, bool isCellPosRelativeToGridCorner) const;

		//! Computes the distances between a given cell and the inner grid boundaries
		void computeSignedDistToBoundaries(const Tuple3i& cellPos, Tuple3i& distToLowerBorder, Tuple3i& distToUpperBorder) const;

	protected:

		//! Mesh
		GenericIndexedMesh* m_mesh;
		//! Distance transform
		SaitoSquaredDistanceTransform* m_distanceTransform;

		//! Virtual grid bounding-box (min corner, in the local mesh coordinate system)
		CCVector3 m_localMinGridBB;
		//! Virtual grid occupancy of the mesh (minimum indexes for each dimension)
		Tuple3i m_minFillIndexes;
		//! Virtual grid occupancy of the mesh (maximum indexes for each dimension)
		Tuple3i m_maxFillIndexes;
		//! Virtual grid cell size
		PointCoordinateType m_cellSize;

		//! Grid of TriangleList items
		/** \warning: may be smaller than the 'virtual grid' (see m_minFillIndexes and m_maxFillIndexes) **/
		Grid3D<TriangleList*> m_perCellTriangleList;

		//! Whether the structure is properly initialized
		bool m_initialized;
	};
}
