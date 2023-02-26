// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "BoundingBox.h"
#include "DgmOctree.h"
#include "GenericIndexedCloudPersist.h"

namespace CCCoreLib
{
	//! A kind of ReferenceCloud based on the DgmOctree::NeighboursSet structure
	class CC_CORE_LIB_API DgmOctreeReferenceCloud : public GenericIndexedCloudPersist
	{
	public:

		//! Default constructor.
		/** \param associatedSet associated NeighboursSet
			\param count number of values to use (0 = all)
		**/
		DgmOctreeReferenceCloud(DgmOctree::NeighboursSet& associatedSet, unsigned count = 0);

		//**** inherited form GenericCloud ****//
		inline unsigned size() const override { return m_size; }
		void forEachScalarValue(GenericScalarValueAction action) override;
		void getLocalBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;
		CCVector3d getLocalToGlobalTranslation() const override { return { 0.0, 0.0, 0.0 }; }
		//inline uint8_t testVisibility(const CCVector3& localP) const override; //not supported
		inline void placeIteratorAtBeginning() override { m_globalIterator = 0; }
		inline const CCVector3* getNextLocalPoint() override { return (m_globalIterator < size() ? m_set[m_globalIterator++].localPoint : nullptr); }
		inline bool enableScalarField() override { return true; } //use DgmOctree::PointDescriptor::squareDistd by default
		inline bool isScalarFieldEnabled() const override { return true; } //use DgmOctree::PointDescriptor::squareDistd by default
		inline void setPointScalarValue(unsigned pointIndex, ScalarType value) override { assert(pointIndex < size()); m_set[pointIndex].squareDistd = static_cast<double>(value); }
		inline ScalarType getPointScalarValue(unsigned pointIndex) const override { assert(pointIndex < size()); return static_cast<ScalarType>(m_set[pointIndex].squareDistd); }
		//**** inherited form GenericIndexedCloud ****//
		inline const CCVector3* getLocalPoint(unsigned index) const override { assert(index < size()); return m_set[index].localPoint; }
		inline void getLocalPoint(unsigned index, CCVector3& P) const override { assert(index < size()); P = *m_set[index].localPoint; }
		//**** inherited form GenericIndexedCloudPersist ****//
		inline const CCVector3* getLocalPointPersistentPtr(unsigned index) const override { assert(index < size()); return m_set[index].localPoint; }

		//! Forwards global iterator
		inline void forwardIterator() { ++m_globalIterator; }

	protected:

		//! Computes the cloud bounding-box (internal)
		virtual void computeLocalBB();

		//! Iterator on the point references container
		unsigned m_globalIterator;

		//! Bounding-box
		BoundingBox m_localBB;

		//! Associated PointDescriptor set
		DgmOctree::NeighboursSet& m_set;

		//! Number of points
		unsigned m_size;
	};
}
