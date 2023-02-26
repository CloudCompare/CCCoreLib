// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "BoundingBox.h"
#include "GenericIndexedMesh.h"
#include "SimpleTriangle.h"

//System
#include <vector>

namespace CCCoreLib
{
	class GenericIndexedCloud;

	//! A simple mesh structure, with index-based vertex access
	/** Implements the GenericIndexedMesh interface. This mesh is always associated
		to a (index based) point cloud that stores the mesh vertexes.
	**/
	class CC_CORE_LIB_API SimpleMesh : public GenericIndexedMesh
	{
	public: //constructors

		//! SimpleMesh Constructor
		/** \param vertices the point cloud containing the vertices
			\param linkVerticesWithMesh specifies if the vertex cloud should be deleted when the SimpleMesh object is destructed
		**/
		SimpleMesh(GenericIndexedCloud* vertices, bool linkVerticesWithMesh = false);

		//! SimpleMesh destructor
		~SimpleMesh() override;

	public: //inherited methods

		void placeIteratorAtBeginning() override;
		inline GenericLocalTriangle* _getNextLocalTriangle() override { return _getLocalTriangle(++m_globalIterator); } //temporary
		inline GenericGlobalTriangle* _getNextGlobalTriangle() override { return _getGlobalTriangle(++m_globalIterator); } //temporary
		GenericLocalTriangle* _getLocalTriangle(unsigned triangleIndex) override; //temporary
		GenericGlobalTriangle* _getGlobalTriangle(unsigned triangleIndex) override; //temporary
		inline VerticesIndexes* getNextTriangleVertIndexes() override { return getTriangleVertIndexes(m_globalIterator++); }
		inline VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override { return &(m_triIndexes[triangleIndex]); }
		unsigned size() const override { return static_cast<unsigned>(m_triIndexes.size()); }
		void getLocalBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override; // simply returns the vertices bounding-box for now
		void getGlobalBoundingBox(CCVector3d& bbMin, CCVector3d& bbMax) override; // simply returns the vertices bounding-box for now
		void getLocalTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const override;
		void getGlobalTriangleVertices(unsigned triangleIndex, CCVector3d& A, CCVector3d& B, CCVector3d& C) const override;
		CCVector3d getLocalToGlobalTranslation() const override;
		CCVector3d toGlobal(const CCVector3& localPoint) const override;
		CCVector3 toLocal(const CCVector3d& globalPoint) const override;

	public: //specific methods

		//! Returns the mesh capacity
		inline unsigned capacity() const { return static_cast<unsigned>(m_triIndexes.capacity()); }

		//! Returns the vertices
		inline const GenericIndexedCloud* vertices() const { return m_vertices; }

		//! Clears the mesh
		inline void clear() { m_triIndexes.resize(0); }

		//! Adds a triangle to the mesh
		/** Vertex indexes are expresesed relatively to the vertex cloud.
			\param i1 first vertex index
			\param i2 second vertex index
			\param i3 third vertex index
		**/
		virtual void addTriangle(unsigned i1, unsigned i2, unsigned i3);

		//! Reserves the memory to store the triangles (as 3 indexes each)
		/** \param n the number of triangles to reserve
			\return true if the method succeeds, false otherwise
		**/
		virtual bool reserve(unsigned n);

		//! Resizes the mesh database
		/** If the new number of elements is smaller than the actual size,
			the overflooding elements will be deleted.
			\param n the new number of triangles
			\return true if the method succeeds, false otherwise
		**/
		virtual bool resize(unsigned n);

		//inherited from GenericIndexedMesh
		bool normalsAvailable() const override;
		bool interpolateNormalsLocal(unsigned triIndex, const CCVector3& P, CCVector3& N) override;
		bool interpolateNormalsGlobal(unsigned triIndex, const CCVector3d& P, CCVector3& N) override;

	protected:

		//! Vertex indexes container
		using TriangleIndexesContainer = std::vector<VerticesIndexes>;
		//! Vertex indexes
		TriangleIndexesContainer m_triIndexes;

		//! Iterator on the set of triangles
		unsigned m_globalIterator;
		//! Temporary local triangle structure
		SimpleLocalTriangle m_dummyLocalTriangle;
		//! Temporary global triangle structure
		SimpleGlobalTriangle m_dummyGlobalTriangle;

		//! Associated point cloud (vertices)
		GenericIndexedCloud* m_vertices;
		//! Whether the associated cloud should be deleted when the mesh is deleted
		bool m_verticesLinked;

		//! Local bounding-box
		BoundingBox m_bbox;
	};

}
