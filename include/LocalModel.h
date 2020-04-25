// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

#include "Neighbourhood.h"

namespace CCCoreLib
{
	//! Local modelization (generic interface)
	/** Local surface approximation of a point cloud
	**/
	class LocalModel
	{
	public:

		//! Factory
		/** \param type the model type
			\param subset (small) set of points from which to compute the local model
			\param center model "center"
			\param squaredRadius model max radius (squared)
		**/
		static LocalModel* New(	LOCAL_MODEL_TYPES type,
								Neighbourhood& subset,
								const CCVector3 &center,
								PointCoordinateType squaredRadius);

		//! Destructor
		virtual ~LocalModel() = default;

		//! Returns the model type
		virtual LOCAL_MODEL_TYPES getType() const = 0;

		//! Returns the model center
		inline const CCVector3& getCenter() const { return m_modelCenter; }

		//! Returns the model max radius (squared)
		inline PointCoordinateType getSquareSize() const { return m_squaredRadius; }

		//! Compute the (unsigned) distance between a 3D point and this model
		/** \param[in] P the query point
			\param[out] nearestPoint returns the nearest point (optional)
			\return the (unsigned) distance (or CCCoreLib::NAN_VALUE if the computation failed)
		**/
		virtual ScalarType computeDistanceFromModelToPoint(const CCVector3* P, CCVector3* nearestPoint = nullptr) const = 0;

	protected:

		//! Constructor
		/** \param center model "center"
			\param squaredRadius model max "radius" (squared)
		**/
		LocalModel(const CCVector3 &center, PointCoordinateType squaredRadius);

		//! Center
		CCVector3 m_modelCenter;

		//! Max radius (squared)
		PointCoordinateType m_squaredRadius;
	};
}
