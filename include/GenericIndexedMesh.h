// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "GenericMesh.h"

#ifdef _MSC_VER
//To get rid of the warning about unnamed struct/union
#pragma warning( disable: 4201 )
#endif

namespace CCCoreLib
{
	//! Triangle described by the indexes of its 3 vertices
	struct VerticesIndexes
	{
		union
		{
			struct
			{
				unsigned i1, i2, i3;
			};
			unsigned i[3];
		};

		//! Constructor with specified indexes
		VerticesIndexes(unsigned _i1, unsigned _i2, unsigned _i3)
			: i1(_i1)
			, i2(_i2)
			, i3(_i3)
		{}

		//! Default constructor
		VerticesIndexes()
			: i1(0)
			, i2(0)
			, i3(0)
		{}
	};

	//! A generic mesh with index-based vertex access
	/** Implements the GenericMesh interface.
	**/
	class CC_CORE_LIB_API GenericIndexedMesh : public GenericMesh
	{
	public:

		//! Default destructor
		~GenericIndexedMesh() override = default;

		//! Returns the ith triangle
		/**	Virtual method to request a triangle with a specific index.
			The returned object can be temporary.
			\param triangleIndex of the requested triangle (between 0 and the mesh size-1)
			\return the requested triangle, or 0 if index value is not valid
		**/
		virtual GenericTriangle* _getTriangle(unsigned triangleIndex) = 0;

		//! Returns the indexes of the vertices of a given triangle
		/**	\param triangleIndex index of the triangle (between 0 and size(mesh)-1)
			\return the triangle indexes (or 0 if index value is not valid)
		**/
		virtual VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) = 0;

		//! Returns the vertices of a given triangle
		/**	\param[in] triangleIndex index of the triangle (between 0 and the size(mesh)-1)
			\param[out] A first vertex
			\param[out] B second vertex
			\param[out] C third vertex
		**/
		virtual void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const = 0;

		//! Returns the indexes of the vertices of the next triangle (relatively to the global iterator position)
		/**	\return the triangle indexes (or 0 if the global iterator is out of bounds)
		**/
		virtual VerticesIndexes* getNextTriangleVertIndexes() = 0;

		//! Returns whether normals are available
		virtual bool normalsAvailable() const { return false; }

		//! Interpolates normal(s) inside a given triangle
		/** This method should be ideally overriden by the child class if normals are supported
			\param[in]  triIndex	triangle index
			\param[in]  P			point where to interpolate (should be inside the triangle!)
			\param[out] N			interpolated normal
			\return success
		**/
		virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) { (void)triIndex; (void)P; (void)N; return false; }
	};
}

#ifdef _MSC_VER
//Restore the default warning behavior
#pragma warning( default: 4201 )
#endif
