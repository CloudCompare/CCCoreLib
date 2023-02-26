// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

#include <functional>

//Local
#include "CCGeom.h"
#include "GenericTriangle.h"

namespace CCCoreLib
{
	//! A generic mesh interface for data communication between library and client applications
	class CC_CORE_LIB_API GenericMesh
	{
	public:

		//! Default destructor
		virtual ~GenericMesh() = default;

		//! Returns the number of triangles
		/**	Virtual method to request the mesh size
			\return the mesh size
		**/
		virtual unsigned size() const = 0;

		//! Returns the mesh local bounding-box
		/**	Virtual method to request the mesh bounding-box limits. It is equivalent to
			the bounding-box of the cloud composed of the mesh vertexes.
			\param localBBMin lower bounding-box limits (Xmin,Ymin,Zmin)
			\param localBBMax higher bounding-box limits (Xmax,Ymax,Zmax)
		**/
		virtual void getLocalBoundingBox(CCVector3& localBBMin, CCVector3& localBBMax) = 0;

		//! Returns the mesh global bounding box
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

		//! Returns the translation from local to global coordinates
		virtual CCVector3d getLocalToGlobalTranslation() const = 0;

		//! Converts a local point to a global point
		virtual CCVector3d toGlobal(const CCVector3& localPoint) const { return CCVector3d::fromArray((getLocalToGlobalTranslation() + localPoint).u); }

		//! Converts a global point to a local point
		virtual CCVector3 toLocal(const CCVector3d& globalPoint) const { return CCVector3::fromArray((globalPoint - getLocalToGlobalTranslation()).u); }

		//! Places the mesh iterator at the beginning
		/**	Virtual method to handle the mesh global iterator
		**/
		virtual void placeIteratorAtBeginning() = 0;

		//! Returns the next (local) triangle (relative to the global iterator position)
		/**	Virtual method to handle the mesh global iterator.
			Global iterator position should be increased each time
			this method is called. The returned object can be temporary.
			\return a triangle
		**/
		virtual GenericLocalTriangle* _getNextLocalTriangle() = 0; //temporary

		//! Returns the next (global) triangle (relative to the global iterator position)
		/**	Virtual method to handle the mesh global iterator.
			Global iterator position should be increased each time
			this method is called. The returned object can be temporary.
			\return a triangle
		**/
		virtual GenericGlobalTriangle* _getNextGlobalTriangle() = 0; //temporary
	};
}
