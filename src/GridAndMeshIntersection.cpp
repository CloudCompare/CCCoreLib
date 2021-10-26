// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <GridAndMeshIntersection.h>

//local
#include <PointCloud.h>
#include <ReferenceCloud.h>
#include <SaitoSquaredDistanceTransform.h>
#include <ScalarField.h>

//system
#include <algorithm>
#include <cassert>

namespace CCCoreLib
{
	//! List of triangles (indexes)
	struct TriangleList
	{
		//! Triangles indexes
		std::vector<unsigned> indexes;

		//! Adds a triangle index
		/** \return success
		**/
		inline bool push(unsigned index)
		{
			try
			{
				indexes.push_back(index);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}
			return true;
		}
	};
}

using namespace CCCoreLib;

GridAndMeshIntersection::GridAndMeshIntersection()
	: m_mesh(nullptr)
	, m_distanceTransform(nullptr)
	, m_minFillIndexes(0, 0, 0)
	, m_maxFillIndexes(0, 0, 0)
	, m_cellSize(0)
	, m_initialized(false)
{}

GridAndMeshIntersection::~GridAndMeshIntersection()
{
	clear();
}

void GridAndMeshIntersection::clear()
{
	m_initialized = false;
	m_mesh = nullptr;
	m_cellSize = 0;

	if (m_perCellTriangleList.isInitialized())
	{
		TriangleList** gridCell = m_perCellTriangleList.data();
		for (std::size_t i = 0; i < m_perCellTriangleList.totalCellCount(); ++i, ++gridCell)
		{
			if (*gridCell)
			{
				delete (*gridCell);
			}
		}
		m_perCellTriangleList.clear();
	}

	if (m_distanceTransform)
	{
		delete m_distanceTransform;
		m_distanceTransform = nullptr;
	}
}

unsigned GridAndMeshIntersection::distanceTransformValue(const Tuple3i& cellPos, bool isLocalCellPos) const
{
	if (m_distanceTransform)
	{
		Tuple3i localCellPos = isLocalCellPos ? cellPos : toLocal(cellPos);
		return m_distanceTransform->getValue(localCellPos);
	}
	else
	{
		assert(false);
		return 0;
	}
}

const CCCoreLib::TriangleList* GridAndMeshIntersection::trianglesInCell(const Tuple3i& cellPos, bool isLocalCellPos) const
{
	if (false == m_initialized)
	{
		return nullptr;
	}

	Tuple3i localCellPos = isLocalCellPos ?  cellPos : toLocal(cellPos);

	return m_perCellTriangleList.getValue(localCellPos);
}


void GridAndMeshIntersection::computeSignedDistToBoundaries(const Tuple3i& cellPos, Tuple3i& distToLowerBorder, Tuple3i& distToUpperBorder) const
{
	distToLowerBorder = cellPos - m_minFillIndexes;
	distToUpperBorder = m_maxFillIndexes - cellPos;
}

bool GridAndMeshIntersection::computeMeshIntersection(	GenericIndexedMesh* mesh,
														const CCVector3& minGridBB,
														const CCVector3& maxGridBB,
														PointCoordinateType cellSize,
														GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (!mesh || cellSize <= 0)
	{
		//invalid input
		assert(false);
		return false;
	}

	// invalidate the grid (just in case)
	clear();

	// we can now update the member variables
	m_mesh = mesh;
	m_cellSize = cellSize;
	m_minGridBB = minGridBB;
	m_maxGridBB = maxGridBB;

	//we compute the grid occupancy ... and we deduce the m_perCellTriangleList grid dimensions
	Tuple3ui gridSize;
	{
		CCVector3 meshMinBB;
		CCVector3 meshMaxBB;
		mesh->getBoundingBox(meshMinBB, meshMaxBB);

		for (unsigned char k = 0; k < 3; ++k)
		{
			m_minFillIndexes.u[k] = static_cast<int>(floor((meshMinBB.u[k] - m_minGridBB.u[k]) / m_cellSize));
			m_maxFillIndexes.u[k] = static_cast<int>(floor((meshMaxBB.u[k] - m_minGridBB.u[k]) / m_cellSize));
			gridSize.u[k] = static_cast<unsigned>(m_maxFillIndexes.u[k] - m_minFillIndexes.u[k] + 1);
			if (gridSize.u[k] == 1)
			{
				// to avoid corner cases (with flat triangles on a border of the unique cell)
				// we prefer to increase the intersection grid by one cell on each side
				m_minGridBB.u[k] -= m_cellSize;
				m_maxFillIndexes.u[k] += 2;
				gridSize.u[k] = 3;
			}
		}
	}
	CCVector3 realMinGridBB = m_minGridBB + CCVector3(m_minFillIndexes.x * m_cellSize, m_minFillIndexes.y * m_cellSize, m_minFillIndexes.z * m_cellSize);

	//we need to build the list of triangles intersecting each cell of the 3D grid
	if (!m_perCellTriangleList.init(gridSize.x, gridSize.y, gridSize.z, 0, nullptr))
	{
		// not enough memory
		clear();
		return false;
	}

	auto setIntersectTriangle = [&](const Tuple3i& cellPos, unsigned triIndex)
	{
		TriangleList*& triList = m_perCellTriangleList.getValue(cellPos);
		if (!triList)
		{
			triList = new TriangleList();
		}

		//add the triangle to the current 'intersecting triangles' list
		triList->push(triIndex);
	};

	if (!m_perCellTriangleList.intersectWith(	mesh,
												m_cellSize,
												realMinGridBB,
												setIntersectTriangle,
												progressCb)
		)
	{
		clear();
		return false;
	}

	m_initialized = true;
	return true;
}

bool GridAndMeshIntersection::initDistanceTransformWithMesh(GenericIndexedMesh* mesh,
															const CCVector3& minGridBB,
															const CCVector3& maxGridBB,
															const CCVector3& minFilledBB,
															const CCVector3& maxFilledBB,
															PointCoordinateType cellSize,
															GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (!mesh || cellSize <= 0)
	{
		//invalid input
		assert(false);
		return false;
	}

	// invalidate the grid (just in case)
	clear();

	// we can now update the member variables
	m_mesh = mesh;
	m_cellSize = cellSize;
	m_minGridBB = minGridBB;
	m_maxGridBB = maxGridBB;

	// we compute distance map dimensions
	Tuple3ui gridSize;
	{
		for (unsigned char k = 0; k < 3; ++k)
		{
			m_minFillIndexes.u[k] = static_cast<int>(floor((minFilledBB.u[k] - m_minGridBB.u[k]) / m_cellSize));
			m_maxFillIndexes.u[k] = static_cast<int>(floor((maxFilledBB.u[k] - m_minGridBB.u[k]) / m_cellSize));
			gridSize.u[k] = static_cast<unsigned>(m_maxFillIndexes.u[k] - m_minFillIndexes.u[k] + 1);
			if (gridSize.u[k] == 1)
			{
				// to avoid corner cases (with flat triangles on a border of the unique cell)
				// we prefer to increase the intersection grid by one cell on each side
				m_minGridBB.u[k] -= m_cellSize;
				m_maxFillIndexes.u[k] += 2;
				gridSize.u[k] = 3;
			}
		}
	}
	CCVector3 realMinGridBB = m_minGridBB + CCVector3(m_minFillIndexes.x * m_cellSize, m_minFillIndexes.y * m_cellSize, m_minFillIndexes.z * m_cellSize);

	// acceleration by approximating the distance
	m_distanceTransform = new SaitoSquaredDistanceTransform;
	if (!m_distanceTransform || !m_distanceTransform->initGrid(gridSize))
	{
		// not enough memory
		clear();
		return false;
	}

	auto initDistanceTransform = [&](const Tuple3i& cellPos, unsigned triIndex)
	{
		m_distanceTransform->setValue(cellPos, 1);
	};

	if (!m_distanceTransform->intersectWith(mesh,
											m_cellSize,
											realMinGridBB,
											initDistanceTransform,
											progressCb)
		)
	{
		clear();
		return false;
	}

	if (!m_distanceTransform->propagateDistance(progressCb))
	{
		// failed to apply the distance transform
		clear();
		return false;
	}

	m_initialized = true;
	return true;
}
