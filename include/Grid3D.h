// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "CCMath.h"
#include "CCMiscTools.h"
#include "GenericCloud.h"
#include "GenericIndexedMesh.h"
#include "GenericProgressCallback.h"
#include "GenericTriangle.h"

//System
#include <cassert>
#include <cstdio>
#include <vector>
#include <functional>

namespace CCCoreLib
{
	//! Simple 3D grid structure
	/** The grid data is contiguous in memory.
	**/
	template< class Type > class Grid3D
	{

	public:

		//! Cell type
		using GridElement = Type;

		//! Default constructor
		Grid3D()
			: m_innerSize      (0, 0, 0)
			, m_margin         (0)
			, m_rowSize        (0)
			, m_sliceSize      (0)
			, m_innerCellCount (0)
			, m_totalCellCount (0)
			, m_marginShift    (0)
		{}

		//! Returns the grid dimensions
		inline const Tuple3ui& size() const { return m_innerSize; }

		//! Returns whether the grid has been initialized or not
		inline bool isInitialized() const { return m_totalCellCount != 0; }

		//! Clears the grid
		/** \warning If Type is a pointer type, memory should be released first
		**/
		void clear()
		{
			m_innerSize			= Tuple3ui(0, 0, 0);
			m_margin			=  0;
			m_innerCellCount	=  0;
			m_rowSize			=  0;
			m_sliceSize			=  0;
			m_totalCellCount	=  0;
			m_marginShift		=  0;

			m_grid.clear();
		}

		//! Initializes the grid
		/** The grid must be explicitelty initialized prior to any action.
			\param di grid size along the X dimension
			\param dj grid size along the Y dimension
			\param dk grid size along the Z dimension
			\param margin grid margin
			\param defaultCellValue default cell value
			\return true if the initialization succeeded
		**/
		bool init(unsigned di, unsigned dj, unsigned dk, unsigned margin, GridElement defaultCellValue = 0)
		{
			m_innerSize			= Tuple3ui(di, dj, dk);
			m_margin			= margin;
			m_innerCellCount	= m_innerSize.x * m_innerSize.y * m_innerSize.z;
			m_rowSize			= (m_innerSize.x + 2 * m_margin);
			m_sliceSize			= (m_innerSize.y + 2 * m_margin) * m_rowSize;
			m_totalCellCount	= (m_innerSize.z + 2 * m_margin) * m_sliceSize;
			m_marginShift		= m_margin * (1 + m_rowSize + m_sliceSize);

			if (m_totalCellCount == 0)
			{
				assert(false);
				return false;
			}

			//grid initialization
			try
			{
				m_grid.resize(m_totalCellCount, defaultCellValue);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				m_totalCellCount = 0;
				return false;
			}

			return true;
		}

		//! Computes the (grid) cell position that contains a given point
		inline Tuple3i computeCellPos(const CCVector3& P, const CCVector3& gridMinCorner,PointCoordinateType cellSize) const
		{
			assert(cellSize > 0);

			//DGM: if we admit that cellLength > 0, then the 'floor' operator is useless (int cast = truncation)
			Tuple3i cellPos(static_cast<int>(/*floor*/(P.x - gridMinCorner.x) / cellSize),
							static_cast<int>(/*floor*/(P.y - gridMinCorner.y) / cellSize),
							static_cast<int>(/*floor*/(P.z - gridMinCorner.z) / cellSize));

			return cellPos;
		}

		//Internal structure used by 'intersectWith'
		struct CellToTest
		{
			//! Cell position
			Tuple3i pos;
			//! Cell size
			int cellSize;
		};

		//! Intersects this grid with a mesh
		bool intersectWith(	GenericIndexedMesh* mesh,
							PointCoordinateType cellLength,
							const CCVector3& gridMinCorner,
							GridElement intersectValue = 0,
							GenericProgressCallback* progressCb = nullptr)
		{
			auto setIntersectValue = [&](const Tuple3i& cellPos, unsigned triIndex)
			{
				this->setValue(cellPos, intersectValue);
			};

			return intersectWith(	mesh,
									cellLength,
									gridMinCorner,
									setIntersectValue,
									progressCb);
		}
		
		//! Generic function applied to a cell intersected by a triangle (used by the generic form of intersectWith)
		using genericCellTriIntersectionAction = std::function<void(const Tuple3i&, unsigned)>;

		//! Intersects this grid with a mesh (generic form)
		bool intersectWith(	GenericIndexedMesh* mesh,
							PointCoordinateType cellLength,
							const CCVector3& gridMinCorner,
							genericCellTriIntersectionAction action,
							GenericProgressCallback* progressCb = nullptr)
		{
			if (!mesh || !isInitialized())
			{
				assert(false);
				return false;
			}

			//cell dimension
			CCVector3 halfCellDimensions(cellLength / 2, cellLength / 2, cellLength / 2);

			std::vector<CellToTest> cellsToTest(1); //initial size must be > 0
			unsigned cellsToTestCount = 0;

			//number of triangles
			unsigned numberOfTriangles = mesh->size();

			//progress notification
			NormalizedProgress nProgress(progressCb, numberOfTriangles);
			if (progressCb)
			{
				if (progressCb->textCanBeEdited())
				{
					char buffer[64];
					sprintf(buffer, "Triangles: %u", numberOfTriangles);
					progressCb->setInfo(buffer);
					progressCb->setMethodTitle("Intersect Grid/Mesh");
				}
				progressCb->update(0);
				progressCb->start();
			}

			//for each triangle: look for intersecting cells
			mesh->placeIteratorAtBeginning();
			for (unsigned n = 0; n < numberOfTriangles; ++n)
			{
				//get the positions (in the grid) of each vertex
				const GenericTriangle* T = mesh->_getNextTriangle();

				//current triangle vertices
				const CCVector3* triPoints[3] = {	T->_getA(),
													T->_getB(),
													T->_getC() };

				CCVector3 AB = (*triPoints[1]) - (*triPoints[0]);
				CCVector3 BC = (*triPoints[2]) - (*triPoints[1]);
				CCVector3 CA = (*triPoints[0]) - (*triPoints[2]);

				//make sure that the triangle is not degenerate!!!
				if ( GreaterThanSquareEpsilon( AB.norm2() ) &&
					 GreaterThanSquareEpsilon( BC.norm2() ) &&
					 GreaterThanSquareEpsilon( CA.norm2() ) )
				{
					Tuple3i cellPos[3] =
					{
						computeCellPos(*(triPoints[0]), gridMinCorner, cellLength),
						computeCellPos(*(triPoints[1]), gridMinCorner, cellLength),
						computeCellPos(*(triPoints[2]), gridMinCorner, cellLength)
					};

					//compute the triangle bounding-box
					Tuple3i minPos, maxPos;
					for (int k = 0; k < 3; k++)
					{
						minPos.u[k] = std::min(cellPos[0].u[k], std::min(cellPos[1].u[k], cellPos[2].u[k]));
						maxPos.u[k] = std::max(cellPos[0].u[k], std::max(cellPos[1].u[k], cellPos[2].u[k]));
					}

					//first cell
					assert(cellsToTest.capacity() != 0);
					cellsToTestCount = 1;
					CellToTest* _currentCell = &cellsToTest[0/*cellsToTestCount-1*/];

					//compute the triangle normal
					CCVector3 N = AB.cross(BC);

					//max distance (in terms of number of cells) between the vertices
					Tuple3i delta = maxPos - minPos + Tuple3i(1, 1, 1);
					int maxSize = 0;
					{
						maxSize = std::max(delta.x, delta.y);
						maxSize = std::max(maxSize, delta.z);
					}

					//we deduce the smallest bounding cell
					static const double LOG_2 = log(2.0);
					_currentCell->cellSize = (1 << (maxSize > 1 ? static_cast<unsigned char>(ceil(log(static_cast<double>(maxSize)) / LOG_2)) : 0));

					if (_currentCell->cellSize > 1)
					{
						//center the virtual cell on the triangle
						for (int k = 0; k < 3; k++)
						{
							_currentCell->pos.u[k] = std::max(0, minPos.u[k] + (delta.u[k] - _currentCell->cellSize) / 2);
						}
					}
					else
					{
						_currentCell->pos = minPos;
					}

					//now we can (recursively) find the intersecting cells
					while (cellsToTestCount != 0)
					{
						_currentCell = &cellsToTest[--cellsToTestCount];

						//new cells may be written over the actual one
						//so we need to remember its position!
						Tuple3i currentCellPos = _currentCell->pos;

						//if we have reached the maximum subdivision level
						if (_currentCell->cellSize == 1)
						{
							//compute the (absolute) cell center
							AB = gridMinCorner + CCVector3::fromArray(currentCellPos.u) * cellLength + halfCellDimensions;

							//check that the triangle does intersect the cell (box)
							if (CCMiscTools::TriBoxOverlap(AB, halfCellDimensions, triPoints))
							{
								if (	(currentCellPos.x >= 0 && currentCellPos.x < static_cast<int>(size().x)) &&
										(currentCellPos.y >= 0 && currentCellPos.y < static_cast<int>(size().y)) &&
										(currentCellPos.z >= 0 && currentCellPos.z < static_cast<int>(size().z)))
								{
									action(currentCellPos, n);
								}
							}
						}
						else
						{
							int halfCellSize = (_currentCell->cellSize >> 1);

							//compute the position of each neighbor cell relatively to the triangle (3*3*3 = 27, including the cell itself)
							char pointsPosition[27];
							{
								char* _pointsPosition = pointsPosition;
								CCVector3 distanceToMinBorder = gridMinCorner - (*triPoints[0]);
								for (int i = 0; i < 3; ++i)
								{
									AB.x = distanceToMinBorder.x + static_cast<PointCoordinateType>(currentCellPos.x + i * halfCellSize) * cellLength;
									for (int j = 0; j < 3; ++j)
									{
										AB.y = distanceToMinBorder.y + static_cast<PointCoordinateType>(currentCellPos.y + j * halfCellSize) * cellLength;
										for (int k = 0; k < 3; ++k)
										{
											AB.z = distanceToMinBorder.z + static_cast<PointCoordinateType>(currentCellPos.z + k * halfCellSize) * cellLength;

											//determine on which side the triangle is
											*_pointsPosition++/*pointsPosition[i*9+j*3+k]*/ = (AB.dot(N) < 0 ? -1 : 1);
										}
									}
								}
							}

							//if necessary we enlarge the queue
							if (cellsToTestCount + 27 > cellsToTest.capacity())
							{
								try
								{
									cellsToTest.resize(std::max(cellsToTest.capacity() + 27, 2 * cellsToTest.capacity()));
								}
								catch (const std::bad_alloc&)
								{
									//out of memory
									return false;
								}
							}

							//the first new cell will be written over the actual one
							CellToTest* _newCell = &cellsToTest[cellsToTestCount];
							_newCell->cellSize = halfCellSize;

							//we look at the position of the 8 sub-cells relatively to the triangle
							for (int i = 0; i < 2; ++i)
							{
								_newCell->pos.x = currentCellPos.x + i * halfCellSize;
								//quick test to determine if the cube is potentially intersecting the triangle's bbox
								if (	static_cast<int>(_newCell->pos.x) + halfCellSize >= minPos.x
									&&	static_cast<int>(_newCell->pos.x) <= maxPos.x)
								{
									for (int j = 0; j < 2; ++j)
									{
										_newCell->pos.y = currentCellPos.y + j * halfCellSize;
										if (	static_cast<int>(_newCell->pos.y) + halfCellSize >= minPos.y
											&&	static_cast<int>(_newCell->pos.y) <= maxPos.y)
										{
											for (int k = 0; k < 2; ++k)
											{
												_newCell->pos.z = currentCellPos.z + k * halfCellSize;
												if (	static_cast<int>(_newCell->pos.z) + halfCellSize >= minPos.z
													&&	static_cast<int>(_newCell->pos.z) <= maxPos.z)
												{
													const char* _pointsPosition = pointsPosition + (i * 9 + j * 3 + k);
													char sum = _pointsPosition[0]  + _pointsPosition[1]
															+  _pointsPosition[3]  + _pointsPosition[4]
															+  _pointsPosition[9]  + _pointsPosition[10]
															+  _pointsPosition[12] + _pointsPosition[13];

													//if not all the vertices of this sub-cube are on the same side, then the triangle may intersect the sub-cube
													if (sum > -8 && sum < 8)
													{
														//we make newCell point on next cell in array
														cellsToTest[++cellsToTestCount] = *_newCell;
														_newCell = &cellsToTest[cellsToTestCount];
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}

				if (progressCb && !nProgress.oneStep())
				{
					return false;
				}
			}

			return true;
		}

		//! Intersects this grid with a cloud
		bool intersectWith(GenericCloud* cloud,
							PointCoordinateType cellLength,
							const CCVector3& gridMinCorner,
							GridElement intersectValue = 0,
							GenericProgressCallback* progressCb = nullptr)
		{
			if (!cloud || !isInitialized())
			{
				assert(false);
				return false;
			}

			//cell dimension
			CCVector3 halfCellDimensions(cellLength / 2, cellLength / 2, cellLength / 2);

			//number of points
			unsigned numberOfPoints = cloud->size();

			//progress notification
			NormalizedProgress nProgress(progressCb, numberOfPoints);
			if (progressCb)
			{
				if (progressCb->textCanBeEdited())
				{
					char buffer[64];
					sprintf(buffer, "Points: %u", numberOfPoints);
					progressCb->setInfo(buffer);
					progressCb->setMethodTitle("Intersect Grid/Cloud");
				}
				progressCb->update(0);
				progressCb->start();
			}

			//for each point: look for the intersecting cell
			cloud->placeIteratorAtBeginning();
			for (unsigned n = 0; n < numberOfPoints; ++n)
			{
				Tuple3i cellPos = computeCellPos(*cloud->getNextPoint(), gridMinCorner, cellLength);

				if (	(cellPos.x >= 0 && cellPos.x < static_cast<int>(size().x)) &&
						(cellPos.y >= 0 && cellPos.y < static_cast<int>(size().y)) &&
						(cellPos.z >= 0 && cellPos.z < static_cast<int>(size().z)))
				{
					setValue(cellPos, intersectValue);
				}

				if (progressCb && !nProgress.oneStep())
				{
					//cancel by user
					return false;
				}
			}

			return true;
		}

		//! Sets the value of a given cell
		/** \param i the cell coordinate along the X dimension
			\param j the cell coordinate along the Y dimension
			\param k the cell coordinate along the Z dimension
			\param value new cell value
		**/
		inline void setValue(int i, int j, int k, GridElement value)
		{
			m_grid[pos2index(i, j, k)] = value;
		}

		//! Sets the value of a given cell
		/** \param cellPos the cell position
			\param value new cell value
		**/
		inline void setValue(const Tuple3i& cellPos, GridElement value)
		{
			m_grid[pos2index(cellPos.x, cellPos.y, cellPos.z)] = value;
		}

		//! Returns the value of a given cell (const version)
		/** \param i the cell coordinate along the X dimension
			\param j the cell coordinate along the Y dimension
			\param k the cell coordinate along the Z dimension
			\return the cell value
		**/
		inline const GridElement& getValue(int i, int j, int k) const
		{
			return m_grid[pos2index(i, j, k)];
		}
		//! Returns the value of a given cell
		/** \param i the cell coordinate along the X dimension
			\param j the cell coordinate along the Y dimension
			\param k the cell coordinate along the Z dimension
			\return the cell value
		**/
		inline GridElement& getValue(int i, int j, int k)
		{
			return m_grid[pos2index(i,j,k)];
		}

		//! Returns the value of a given cell const version)
		/** \param cellPos the cell position
			\return the cell value
		**/
		const GridElement& getValue(const Tuple3i& cellPos) const
		{
			return m_grid[pos2index(cellPos.x, cellPos.y, cellPos.z)];
		}
		//! Returns the value of a given cell
		/** \param cellPos the cell position
			\return the cell value
		**/
		GridElement& getValue(const Tuple3i& cellPos)
		{
			return m_grid[pos2index(cellPos.x, cellPos.y, cellPos.z)];
		}

		//! Gives access to the internal grid data (with margin)
		inline GridElement* data() { return m_grid.data(); }
		//! Gives access to the internal grid data (with margin) (const version)
		inline const GridElement* data() const { return m_grid.data(); }

		//! Returns the number of cell count (whithout margin)
		inline unsigned innerCellCount() const { return m_innerCellCount; }
		//! Returns the total number of cell count (with margin)
		inline unsigned totalCellCount() const { return m_totalCellCount; }

	protected:

		//! Converts a 3D position to an absolute index
		inline int pos2index(int i, int j, int k) const { return i + j * static_cast<int>(m_rowSize) + k * static_cast<int>(m_sliceSize) + static_cast<int>(m_marginShift); }

		//! Grid data
		std::vector<GridElement> m_grid;

		//! Dimensions of the grid (without margin)
		Tuple3ui m_innerSize;
		//! Margin
		unsigned m_margin;
		//! 1D row size (with margin)
		unsigned m_rowSize;
		//! 2D slice size (with margin)
		unsigned m_sliceSize;
		//! 3D grid size without margin
		unsigned m_innerCellCount;
		//! 3D grid size with margin
		unsigned m_totalCellCount;
		//! First index of real data (i.e. after marin)
		unsigned m_marginShift;
	};
}
