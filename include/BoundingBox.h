#pragma once

// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

//Local
#include "SquareMatrix.h"

//System
#include <algorithm>
#include <cstdint>

namespace CCCoreLib
{
	//! Bounding box structure
	template <typename T> class BoundingBoxTpl
	{
	public:

		//! Default constructor
		BoundingBoxTpl()
			: m_bbMin(0, 0, 0)
			, m_bbMax(0, 0, 0)
			, m_valid(false)
		{}

		//! Constructor from two vectors (lower min. and upper max. corners)
		BoundingBoxTpl(const Vector3Tpl<T>& minCorner, const Vector3Tpl<T>& maxCorner, bool valid)
			: m_bbMin(minCorner)
			, m_bbMax(maxCorner)
			, m_valid(valid)
		{}

		//! Returns the 'sum' of this bounding-box and another one
		BoundingBoxTpl<T> operator + (const BoundingBoxTpl<T>& bbox) const
		{
			if (!m_valid)
				return bbox;
			if (!bbox.isValid())
				return *this;

			BoundingBoxTpl<T> tempBox;
			{
				tempBox.m_bbMin.x = std::min(m_bbMin.x, bbox.m_bbMin.x);
				tempBox.m_bbMin.y = std::min(m_bbMin.y, bbox.m_bbMin.y);
				tempBox.m_bbMin.z = std::min(m_bbMin.z, bbox.m_bbMin.z);
				tempBox.m_bbMax.x = std::max(m_bbMax.x, bbox.m_bbMax.x);
				tempBox.m_bbMax.y = std::max(m_bbMax.y, bbox.m_bbMax.y);
				tempBox.m_bbMax.z = std::max(m_bbMax.z, bbox.m_bbMax.z);
				tempBox.setValidity(true);
			}

			return tempBox;
		}

		//! In place 'sum' of this bounding-box with another one
		const BoundingBoxTpl<T>& operator += (const BoundingBoxTpl<T>& bbox)
		{
			if (bbox.isValid())
			{
				add(bbox.minCorner());
				add(bbox.maxCorner());
			}

			return *this;
		}

		//! Shifts the bounding box with a vector
		const BoundingBoxTpl<T>& operator += (const Vector3Tpl<T>& V)
		{
			if (m_valid)
			{
				m_bbMin += V;
				m_bbMax += V;
			}

			return *this;
		}

		//! Shifts the bounding box with a vector
		const BoundingBoxTpl<T>& operator -= (const Vector3Tpl<T>& V)
		{
			if (m_valid)
			{
				m_bbMin -= V;
				m_bbMax -= V;
			}

			return *this;
		}

		//! Scales the bounding box
		const BoundingBoxTpl<T>& operator *= (T scaleFactor)
		{
			if (m_valid)
			{
				m_bbMin *= scaleFactor;
				m_bbMax *= scaleFactor;
			}

			return *this;
		}

		//! Rotates the bounding box
		const BoundingBoxTpl<T>& operator *= (const SquareMatrixTpl<T>& mat)
		{
			if (m_valid)
			{
				Vector3Tpl<T> boxCorners[8];

				boxCorners[0] = m_bbMin;
				boxCorners[1] = Vector3Tpl<T>(m_bbMin.x, m_bbMin.y, m_bbMax.z);
				boxCorners[2] = Vector3Tpl<T>(m_bbMin.x, m_bbMax.y, m_bbMin.z);
				boxCorners[3] = Vector3Tpl<T>(m_bbMax.x, m_bbMin.y, m_bbMin.z);
				boxCorners[4] = m_bbMax;
				boxCorners[5] = Vector3Tpl<T>(m_bbMin.x, m_bbMax.y, m_bbMax.z);
				boxCorners[6] = Vector3Tpl<T>(m_bbMax.x, m_bbMax.y, m_bbMin.z);
				boxCorners[7] = Vector3Tpl<T>(m_bbMax.x, m_bbMin.y, m_bbMax.z);

				clear();

				for (int i = 0; i < 8; ++i)
				{
					add(mat*boxCorners[i]);
				}
			}

			return *this;
		}

		//! Resets the bounding box
		/** (0,0,0) --> (0,0,0)
		**/
		void clear()
		{
			m_bbMin = m_bbMax = Vector3Tpl<T>(0, 0, 0);
			m_valid = false;
		}

		//! 'Enlarges' the bounding box with a point
		void add(const Vector3Tpl<T>& P)
		{
			if (m_valid)
			{
				if (P.x < m_bbMin.x)
					m_bbMin.x = P.x;
				else if (P.x > m_bbMax.x)
					m_bbMax.x = P.x;

				if (P.y < m_bbMin.y)
					m_bbMin.y = P.y;
				else if (P.y > m_bbMax.y)
					m_bbMax.y = P.y;

				if (P.z < m_bbMin.z)
					m_bbMin.z = P.z;
				else if (P.z > m_bbMax.z)
					m_bbMax.z = P.z;
			}
			else
			{
				m_bbMax = m_bbMin = P;
				m_valid = true;
			}
		}

		//! Returns min corner (const)
		inline const Vector3Tpl<T>& minCorner() const { return m_bbMin; }
		//! Returns max corner (const)
		inline const Vector3Tpl<T>& maxCorner() const { return m_bbMax; }

		//! Returns min corner
		inline Vector3Tpl<T>& minCorner() { return m_bbMin; }
		//! Returns max corner
		inline Vector3Tpl<T>& maxCorner() { return m_bbMax; }

		//! Returns center
		Vector3Tpl<T> getCenter() const
		{
			return (m_bbMax + m_bbMin) * static_cast<T>(0.5);
		}

		//! Returns diagonal vector
		Vector3Tpl<T> getDiagVec() const
		{
			return (m_bbMax - m_bbMin);
		}

		//! Returns diagonal length
		inline T getDiagNorm() const
		{
			return getDiagVec().norm();
		}
		
		//! Returns diagonal length (double precision)
		double getDiagNormd() const
		{
			return getDiagVec().normd();
		}
		
		//! Returns minimal box dimension
		T getMinBoxDim() const
		{
			Vector3Tpl<T> V = getDiagVec();

			return std::min(V.x, std::min(V.y, V.z));
		}

		//! Returns maximal box dimension
		T getMaxBoxDim() const
		{
			Vector3Tpl<T> V = getDiagVec();

			return std::max(V.x, std::max(V.y, V.z));
		}

		//! Returns the bounding-box volume
		double computeVolume() const
		{
			Vector3Tpl<T> V = getDiagVec();

			return static_cast<double>(V.x) * static_cast<double>(V.y) * static_cast<double>(V.z);
		}

		//! Sets bonding box validity
		inline void setValidity(bool state)
		{
			m_valid = state;
		}

		//! Returns whether bounding box is valid or not
		inline bool isValid() const
		{
			return m_valid;
		}

		//! Computes min gap (absolute distance) between this bounding-box and another one
		/** \return min gap (>=0) or -1 if at least one of the box is not valid
		**/
		T minDistTo(const BoundingBoxTpl<T>& bbox) const
		{
			if (m_valid && bbox.isValid())
			{
				Vector3Tpl<T> d(0, 0, 0);

				for (uint8_t dim = 0; dim < 3; ++dim)
				{
					//if the boxes overlap in one dimension, the distance is zero (in this dimension)
					if (bbox.m_bbMin.u[dim] > m_bbMax.u[dim])
						d.u[dim] = bbox.m_bbMin.u[dim] - m_bbMax.u[dim];
					else if (bbox.m_bbMax.u[dim] < m_bbMin.u[dim])
						d.u[dim] = m_bbMin.u[dim] - bbox.m_bbMax.u[dim];
				}

				return d.norm();
			}
			else
			{
				return std::numeric_limits<T>::quiet_NaN();
			}
		}

		//! Returns whether a points is inside the box or not
		/** Warning: box should be valid!
		**/
		inline bool contains(const Vector3Tpl<T>& P) const
		{
			return (P.x >= m_bbMin.x && P.x <= m_bbMax.x &&
					P.y >= m_bbMin.y && P.y <= m_bbMax.y &&
					P.z >= m_bbMin.z && P.z <= m_bbMax.z);
		}

	protected:

		//! Lower min. corner
		Vector3Tpl<T> m_bbMin;
		//! Upper max. corner
		Vector3Tpl<T> m_bbMax;
		//! Validity
		bool m_valid;
	};

	//! Default bounding-box type
	using BoundingBox = BoundingBoxTpl<PointCoordinateType>;
}
