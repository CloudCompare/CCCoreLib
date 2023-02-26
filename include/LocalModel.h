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
			\param center model local "center" point
			\param squaredRadius model max radius (squared)
		**/
		static LocalModel* New(	LOCAL_MODEL_TYPES type,
								Neighbourhood& subset,
								const CCVector3& localCenter,
								PointCoordinateType squaredRadius);

		//! Destructor
		virtual ~LocalModel() = default;

		//! Returns the model type
		virtual LOCAL_MODEL_TYPES getType() const = 0;

		//! Returns the model center
		inline const CCVector3& getLocalCenter() const { return m_modelLocalCenter; }

		//! Returns the model max radius (squared)
		inline PointCoordinateType getSquareSize() const { return m_squaredRadius; }

		//! Compute the (unsigned) distance between a 3D point and this model
		/** \param[in]	Plocal the query point (local)
			\param[out]	nearestPoint returns the nearest point (optional)
			\return the (unsigned) distance (or CCCoreLib::NAN_VALUE if the computation failed)
		**/
		virtual ScalarType computeDistanceFromLocalPointToModel(const CCVector3& Plocal, CCVector3* nearestPoint = nullptr) const = 0;

	protected:

		//! Constructor
		/** \param localCenter model local "center" point
			\param squaredRadius model max "radius" (squared)
		**/
		LocalModel(const CCVector3& localCenter, PointCoordinateType squaredRadius);

		//! Local center
		CCVector3 m_modelLocalCenter;

		//! Max radius (squared)
		PointCoordinateType m_squaredRadius;
	};
}
