// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

#include <functional>

//Local
#include "CCConst.h"
#include "CCGeom.h"

namespace CCCoreLib
{
	//! A generic 3D point cloud interface for data communication between library and client applications
	class CC_CORE_LIB_API GenericCloud
	{

	public:

		//! Default constructor
		GenericCloud() = default;

		//! Default destructor
		virtual ~GenericCloud() = default;

		//! Returns the number of points
		/**	Virtual method to request the cloud size
			\return the cloud size
		**/
		virtual unsigned size() const = 0;

		//! Converts a local point to a global point
		virtual CCVector3d toGlobal(const CCVector3& localPoint) const { return CCVector3d::fromArray((getLocalToGlobalTranslation() + localPoint).u); }

		//! Converts a global point to a local point
		virtual CCVector3 toLocal(const CCVector3d& globalPoint) const { return CCVector3::fromArray((globalPoint - getLocalToGlobalTranslation()).u); }

		//! Returns the cloud local bounding box
		/**	Virtual method to request the cloud local bounding box limits
			\param localBBMin lower bounding-box limits (Xmin,Ymin,Zmin)
			\param localBBMax higher bounding-box limits (Xmax,Ymax,Zmax)
		**/
		virtual void getLocalBoundingBox(CCVector3& localBBMin, CCVector3& localBBMax) = 0;

		//! Returns the cloud global bounding box
		/**	Default implementation. Can be overridden.
			\param globalBBMn lower bounding-box limits (Xmin,Ymin,Zmin)
			\param globalBBMax higher bounding-box limits (Xmax,Ymax,Zmax)
		**/
		virtual void getGlobalBoundingBox(CCVector3d& globalBBMn, CCVector3d& globalBBMax)
		{
			CCVector3 localBBMin, localBBMax;
			getLocalBoundingBox(localBBMin, localBBMax);

			globalBBMn = toGlobal(localBBMin);
			globalBBMax = toGlobal(localBBMax);
		}

		//! Returns a given (global) point visibility state (relative to a sensor for instance)
		/**	Generic method to request a point visibility (should be overridden if this functionality is required).
			The point visibility is such as defined in Daniel Girardeau-Montaut's PhD manuscript (see Chapter 2,
			section 2-3-3). In this case, a ground based laser sensor model should be used to determine it.
			This method is called before performing any point-to-cloud comparison for instance.
			\param localPoint the 3D point to test (in the local coordinate system)
			\return the point visibility (default: POINT_VISIBLE)
		**/
		virtual inline uint8_t testVisibility(const CCVector3& localPoint) const { (void)localPoint; return POINT_VISIBLE; }

		//! Sets the cloud iterator at the beginning
		/**	Virtual method to handle the cloud global iterator
		**/
		virtual void placeIteratorAtBeginning() = 0;

		//! Returns the next local point (relatively to the global iterator position)
		/**	Virtual method to handle the cloud global iterator.
			Global iterator position should be increased by one each time
			this method is called.
			Warning:
			- the returned object may not be persistent!
			- THIS METHOD MAY NOT BE COMPATIBLE WITH PARALLEL STRATEGIES
			(see the DgmOctree::executeFunctionForAllCellsAtLevel_MT and
			DgmOctree::executeFunctionForAllCellsAtStartingLevel_MT methods).
			\return pointer to the next point (or nullptr if no more)
		**/
		virtual const CCVector3* getNextLocalPoint() = 0;

		//! Returns the next global point (relatively to the global iterator position)
		/**	Virtual method to handle the cloud global iterator.
			Global iterator position should be increased by one each time
			this method is called.
			Warning:
			- THIS METHOD MAY NOT BE COMPATIBLE WITH PARALLEL STRATEGIES
			(see the DgmOctree::executeFunctionForAllCellsAtLevel_MT and
			DgmOctree::executeFunctionForAllCellsAtStartingLevel_MT methods).
			\return next point
		**/
		virtual inline const CCVector3d getNextGlobalPoint()
		{
			return toGlobal(*getNextLocalPoint());
		}

		//! Returns the translation from local to global coordinates
		virtual CCVector3d getLocalToGlobalTranslation() const = 0;

	public: // Scalar values

		//!	Enables the scalar field associated to the cloud
		/** If the scalar field structure is not yet initialized/allocated,
			this method gives the signal for its creation. Otherwise, if possible
			the structure size should be pre-reserved with the same number of
			elements as the point cloud.
			\warning If the cloud is empty, the scalar field will be empty as well.
			         The scalar field will be reserved with the same capacity as the cloud.
		**/
		virtual bool enableScalarField() = 0;

		//! Returns true if the scalar field is enabled, false otherwise
		virtual bool isScalarFieldEnabled() const = 0;

		//! Sets the ith point associated scalar value
		virtual void setPointScalarValue(unsigned pointIndex, ScalarType value) = 0;

		//! Returns the ith point associated scalar value
		virtual ScalarType getPointScalarValue(unsigned pointIndex) const = 0;

		//! Generic function to be applied to a scalar value (used by foreach)
		using GenericScalarValueAction = std::function<void(ScalarType&)>;

		//! Fast iteration mechanism on scalar values
		/**	Virtual method to apply a function to the whole scalar field
			\param action the function to apply (see GenericCloud::GenericScalarValueAction)
		**/
		virtual void forEachScalarValue(GenericScalarValueAction action) = 0;

	};

}
