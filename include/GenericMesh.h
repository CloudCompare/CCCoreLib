// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

#include <functional>

//Local
#include "CCGeom.h"

namespace CCCoreLib
{
	class GenericTriangle;

	//! A generic mesh interface for data communication between library and client applications
	class CC_CORE_LIB_API GenericMesh
	{
	public:

		//! Default destructor
		virtual ~GenericMesh() = default;

		//! Generic function to apply to a triangle (used by foreach)
		using genericTriangleAction = std::function<void (GenericTriangle &)>;

		//! Returns the number of triangles
		/**	Virtual method to request the mesh size
			\return the mesh size
		**/
		virtual unsigned size() const = 0;

		//! Fast iteration mechanism
		/**	Virtual method to apply a function to the whole mesh
			\param action function to apply (see GenericMesh::genericTriangleAction)
		**/
		virtual void forEach(genericTriangleAction action) = 0;

		//! Returns the mesh bounding-box
		/**	Virtual method to request the mesh bounding-box limits. It is equivalent to
			the bounding-box of the cloud composed of the mesh vertexes.
			\param bbMin lower bounding-box limits (Xmin,Ymin,Zmin)
			\param bbMax higher bounding-box limits (Xmax,Ymax,Zmax)
		**/
		virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) = 0;

		//! Places the mesh iterator at the beginning
		/**	Virtual method to handle the mesh global iterator
		**/
		virtual void placeIteratorAtBeginning() = 0;

		//! Returns the next triangle (relatively to the global iterator position)
		/**	Virtual method to handle the mesh global iterator.
			Global iterator position should be increased each time
			this method is called. The returned object can be temporary.
			\return a triangle
		**/
		virtual GenericTriangle* _getNextTriangle() = 0; //temporary
	};
}
