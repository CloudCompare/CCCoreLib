// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "BoundingBox.h"
#include "GenericIndexedCloudPersist.h"
#include "ScalarField.h"

//STL
#include <vector>

namespace CCCoreLib
{
	//! A storage-efficient point cloud structure that can also handle an unlimited number of scalar fields
	template<class BaseClass, typename StringType = const char*> class PointCloudTpl : public BaseClass
	{
		static_assert( std::is_base_of<GenericIndexedCloudPersist, BaseClass>::value,
		"BaseClass must be a descendant of GenericIndexedCloudPersist"
		);

	public:
		//! Default constructor
		PointCloudTpl(const CCVector3d& fromLocalToGlobal = {})
			: BaseClass()
			, m_fromLocalToGlobal(fromLocalToGlobal)
			, m_currentPointIndex(0)
			, m_currentInScalarFieldIndex(-1)
			, m_currentOutScalarFieldIndex(-1)
		{}

		//! Alternate constructor with a name and ID
		PointCloudTpl(StringType name, unsigned ID, const CCVector3d& fromLocalToGlobal = {})
			: BaseClass(name, ID)
			, m_fromLocalToGlobal(fromLocalToGlobal)
			, m_currentPointIndex(0)
			, m_currentInScalarFieldIndex(-1)
			, m_currentOutScalarFieldIndex(-1)
		{}

		//! Copy Constructor
		PointCloudTpl(const PointCloudTpl &rhs)
			: BaseClass()
			, m_points(rhs.m_points)
			, m_fromLocalToGlobal(rhs.m_fromLocalToGlobal)
			, m_bbox(rhs.m_bbox)
			, m_currentPointIndex(rhs.m_currentPointIndex)
			, m_scalarFields(rhs.m_scalarFields)
			, m_currentInScalarFieldIndex(rhs.m_currentInScalarFieldIndex)
			, m_currentOutScalarFieldIndex(rhs.m_currentOutScalarFieldIndex)
		{
			// Link all the existing scalar fields so they don't get deleted when rhs goes out of scope
			for (ScalarField* sf : m_scalarFields)
			{
				sf->link();
			}
		}

		//! Default destructor
		virtual ~PointCloudTpl()
		{
			deleteAllScalarFields();
		}

		//! Copy Assignment
		PointCloudTpl& operator=(const PointCloudTpl& rhs)
		{
			m_points = rhs.m_points;
			m_fromLocalToGlobal = rhs.m_fromLocalToGlobal;
			m_bbox = rhs.m_bbox;
			m_currentPointIndex = rhs.m_currentPointIndex;
			m_scalarFields = rhs.m_scalarFields;
			m_currentInScalarFieldIndex = rhs.m_currentInScalarFieldIndex;
			m_currentOutScalarFieldIndex = rhs.m_currentOutScalarFieldIndex;

			// Link all the existing scalar fields so they don't get deleted when rhs goes out of scope
			for (ScalarField* sf : m_scalarFields)
			{
				sf->link();
			}

			return *this;
		}

		// Inherited from GenericCloud
		inline unsigned size() const override
		{
			return static_cast<unsigned>(m_points.size());
		}

		//! Returns cloud capacity (i.e. the reserved size)
		/** \note Might be larger than the cloud size
		**/
		inline unsigned capacity() const
		{
			return static_cast<unsigned>(m_points.capacity());
		}

		// Inherited from GenericCloud
		void getLocalBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override
		{
			if (!m_bbox.isValid())
			{
				m_bbox.clear();
				for (const CCVector3& P : m_points)
				{
					m_bbox.add(P);
				}
			}

			bbMin = m_bbox.minCorner();
			bbMax = m_bbox.maxCorner();
		}

		// Inherited from GenericCloud
		void placeIteratorAtBeginning() override
		{
			m_currentPointIndex = 0;
		}

		// Inherited from GenericCloud
		CCVector3d toGlobal(const CCVector3& localPoint) const override { return m_fromLocalToGlobal + CCVector3d::fromArray(localPoint.u); }

		// Inherited from GenericCloud
		CCVector3 toLocal(const CCVector3d& globalPoint) const override { return CCVector3::fromArray((globalPoint - m_fromLocalToGlobal).u); }

		// Inherited from GenericCloud
		const CCVector3* getNextLocalPoint() override
		{
			return (m_currentPointIndex < m_points.size() ? localPoint(m_currentPointIndex++) : nullptr);
		}

		// Inherited from GenericCloud
		const CCVector3d getNextGlobalPoint() override
		{
			return (m_currentPointIndex < m_points.size() ? globalPoint(m_currentPointIndex++) : CCVector3d{});
		}

		// Inherited from GenericIndexedCloud
		inline const CCVector3* getLocalPoint(unsigned index) const override
		{
			return localPoint(index);
		}

		// Inherited from GenericIndexedCloud
		inline void getLocalPoint(unsigned index, CCVector3& P) const override
		{
			P = *localPoint(index);
		}

		// Inherited from GenericIndexedCloudPersist
		inline const CCVector3* getLocalPointPersistentPtr(unsigned index) const override
		{
			return localPoint(index);
		}

		// Inherited from GenericCloud
		bool enableScalarField() override
		{
			if (m_points.empty() && m_points.capacity() == 0)
			{
				//on must call resize or reserve on the cloud first
				return false;
			}

			ScalarField* currentInScalarField = getCurrentInScalarField();

			if (!currentInScalarField)
			{
				//if we get there, it means that either the caller has forgot to create
				//(and assign) a scalar field to the cloud, or that we are in a compatibility
				//mode with old/basic behaviour: a unique SF for everything (input/output)

				//we look for any already existing "default" scalar field
				m_currentInScalarFieldIndex = getScalarFieldIndexByName("Default");
				if (m_currentInScalarFieldIndex < 0)
				{
					//if not, we create it
					m_currentInScalarFieldIndex = addScalarField("Default");
					if (m_currentInScalarFieldIndex < 0) //Something went wrong
					{
						return false;
					}
				}

				currentInScalarField = getCurrentInScalarField();
				assert(currentInScalarField);
			}

			//if there's no output scalar field either, we set this new scalar field as output also
			if (!getCurrentOutScalarField())
			{
				m_currentOutScalarFieldIndex = m_currentInScalarFieldIndex;
			}

			if (m_points.empty())
			{
				//if the cloud is empty, with a reserved capacity, we do the same on the SF
				return currentInScalarField->reserveSafe(m_points.capacity());
			}
			else
			{
				//otherwise we resize the SF with the current number of points so that they match
				return currentInScalarField->resizeSafe(m_points.size());
			}
		}

		// Inherited from GenericCloud
		bool isScalarFieldEnabled() const override
		{
			ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
			if (!currentInScalarFieldArray)
			{
				return false;
			}

			std::size_t sfValuesCount = currentInScalarFieldArray->size();
			return (sfValuesCount != 0 && sfValuesCount >= m_points.size());
		}

		// Inherited from GenericCloud
		void setPointScalarValue(unsigned pointIndex, ScalarType value) override
		{
			assert(m_currentInScalarFieldIndex >= 0 && m_currentInScalarFieldIndex < static_cast<int>(m_scalarFields.size()));
			//slow version
			//ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
			//if (currentInScalarFieldArray)
			//	currentInScalarFieldArray->setValue(pointIndex,value);

			//fast version
			m_scalarFields[m_currentInScalarFieldIndex]->setValue(pointIndex, value);
		}

		// Inherited from GenericCloud
		ScalarType getPointScalarValue(unsigned pointIndex) const override
		{
			assert(m_currentOutScalarFieldIndex >= 0 && m_currentOutScalarFieldIndex < static_cast<int>(m_scalarFields.size()));

			return m_scalarFields[m_currentOutScalarFieldIndex]->getValue(pointIndex);
		}

		// Inherited from GenericCloud
		void forEachScalarValue(GenericCloud::GenericScalarValueAction action) override
		{
			//there's no point in calling forEachScalarValue if there's no activated scalar field!
			ScalarField* currentOutScalarFieldArray = getCurrentOutScalarField();
			if (!currentOutScalarFieldArray)
			{
				assert(false);
				return;
			}

			unsigned count = size();
			for (unsigned i = 0; i < count; ++i)
			{
				action((*currentOutScalarFieldArray)[i]);
			}
		}

		//! Adds a scalar values to the active 'in' scalar field
		/** \param value a scalar value
		**/
		void addPointScalarValue(ScalarType value)
		{
			assert(m_currentInScalarFieldIndex >= 0 && m_currentInScalarFieldIndex < static_cast<int>(m_scalarFields.size()));

			//fast version
			m_scalarFields[m_currentInScalarFieldIndex]->addElement(value);
		}

		//! Resizes the set of points as well as the scalar fields
		/** If the new size is smaller, the exceeding points will be deleted.
			If it's greater, the vector is filled with blank points
			(warning, the PointCloudTpl::addPoint method will insert points after those ones).
			\param newNumberOfPoints the new number of points
			\return true if the method succeeds, false otherwise (not enough memory)
		**/
		virtual bool resize(unsigned newNumberOfPoints)
		{
			std::size_t oldCount = m_points.size();

			//try to enlarge the vector of points (size)
			try
			{
				m_points.resize(newNumberOfPoints);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}

			//now try to enlarge the scalar fields
			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				if (!m_scalarFields[i]->resizeSafe(newNumberOfPoints))
				{
					//if something fails, we restore the (probably smaller) previous size for already resized SFs!
					for (std::size_t j = 0; j < i; ++j)
					{
						m_scalarFields[j]->resize(oldCount);
						m_scalarFields[j]->computeMinAndMax();
					}
					//we can assume that newNumberOfPoints > oldCount, so it should always be ok
					m_points.resize(oldCount);
					return false;
				}
				m_scalarFields[i]->computeMinAndMax();
			}

			return true;
		}

		//! Reserves memory for the points and scalar fields
		/** This method tries to reserve some memory to store points
			that will be inserted later (with PointCloudTpl::addPoint).
			If the new number of points is smaller than the actual one,
			nothing happens.
			\param newCapacity the new capacity
			\return true if the method succeeds, false otherwise (not enough memory)
		**/
		virtual bool reserve(unsigned newCapacity)
		{
			//try to enlarge the vector of points (capacity)
			try
			{
				m_points.reserve(newCapacity);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}

			//now try to enlarge the scalar fields
			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				if (!m_scalarFields[i]->reserveSafe(newCapacity))
				{
					return false;
				}
			}

			//double check
			return (m_points.capacity() >= newCapacity);
		}

		//! Clears the cloud
		/** Equivalent to resize(0).
		**/
		void reset()
		{
			deleteAllScalarFields();
			placeIteratorAtBeginning();
			m_points.resize(0);
			invalidateBoundingBox();
		}

		//! Sets the translation from local to global coordinates
		void setLocalToGlobalTranslation(const CCVector3d& fromLocalToGlobal)
		{
			m_fromLocalToGlobal = fromLocalToGlobal;
		}

		//! Returns the translation from local to global coordinates
		CCVector3d getLocalToGlobalTranslation() const override
		{
			return m_fromLocalToGlobal;
		}

		//! Adds a 3D local point
		/** To ensure the best efficiency, the memory must have already
			been reserved (with PointCloudTpl::reserve).
			\param P a 3D point
		**/
		void addLocalPoint(const CCVector3& P)
		{
			//NaN coordinates check
			if (	std::isnan(P.x)
				||	std::isnan(P.y)
				||	std::isnan(P.z))
			{
				//replace NaN point by (0, 0, 0)
				m_points.push_back({ 0, 0, 0 });
			}
			else
			{
				m_points.push_back(P);
			}

			m_bbox.setValidity(false);
		}

		//! Adds a 3D local point
		/** To ensure the best efficiency, the memory must have already
			been reserved (with PointCloudTpl::reserve).
			\param P a 3D point
		**/
		inline void addGlobalPoint(const CCVector3d& P)
		{
			addLocalPoint(toLocal(P));
		}

		//! Invalidates the bounding-box
		/** Forces the bounding-box to be recomputed next time 'getBoundingBox' is called.
		**/
		virtual void invalidateBoundingBox()
		{
			m_bbox.setValidity(false);
		}

	public: //Scalar fields management

		//! Returns the number of associated (and active) scalar fields
		/** \return the number of active scalar fields
		**/
		inline unsigned getNumberOfScalarFields() const { return static_cast<unsigned>(m_scalarFields.size()); }

		//! Returns a pointer to a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a ScalarField structure, or 0 if the index is invalid.
		**/
		ScalarField* getScalarField(int index) const
		{
			return (index >= 0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index] : 0);
		}

		//! Returns the name of a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a string structure (null-terminated array of characters), or 0 if the index is invalid.
		**/
		const char* getScalarFieldName(int index) const
		{
			return (index >= 0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index]->getName() : 0);
		}

		//! Returns the index of a scalar field represented by its name
		/** \param name a scalar field name
			\return an index (-1 if the scalar field couldn't be found)
		**/
		int getScalarFieldIndexByName(const char* name) const
		{
			std::size_t sfCount = m_scalarFields.size();
			for (std::size_t i = 0; i < sfCount; ++i)
			{
				//we don't accept two SF with the same name!
				if (strcmp(m_scalarFields[i]->getName(), name) == 0)
					return static_cast<int>(i);
			}

			return -1;
		}

		//! Returns the scalar field currently associated to the cloud input
		/** See PointCloud::setPointScalarValue.
			\return a pointer to the currently defined INPUT scalar field (or 0 if none)
		**/
		inline ScalarField* getCurrentInScalarField() const { return getScalarField(m_currentInScalarFieldIndex); }

		//! Returns the scalar field currently associated to the cloud output
		/** See PointCloud::getPointScalarValue.
			\return a pointer to the currently defined OUTPUT scalar field (or 0 if none)
		**/
		inline ScalarField* getCurrentOutScalarField() const { return getScalarField(m_currentOutScalarFieldIndex); }

		//! Sets the INPUT scalar field
		/** This scalar field will be used by the PointCloud::setPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline void setCurrentInScalarField(int index) { m_currentInScalarFieldIndex = index; }

		//! Returns current INPUT scalar field index (or -1 if none)
		inline int getCurrentInScalarFieldIndex() { return m_currentInScalarFieldIndex; }

		//! Sets the OUTPUT scalar field
		/** This scalar field will be used by the PointCloud::getPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline void setCurrentOutScalarField(int index) { m_currentOutScalarFieldIndex = index; }

		//! Returns current OUTPUT scalar field index (or -1 if none)
		inline int getCurrentOutScalarFieldIndex() { return m_currentOutScalarFieldIndex; }

		//! Sets both the INPUT & OUTPUT scalar field
		/** This scalar field will be used by both PointCloud::getPointScalarValue
			and PointCloud::setPointScalarValue methods.
			\param index a scalar field index
		**/
		inline void setCurrentScalarField(int index) { setCurrentInScalarField(index); setCurrentOutScalarField(index); }

		//! Creates a new scalar field and registers it
		/** Warnings:
			- the name must be unique (the method will fail if a SF with the same name already exists)
			- this method DOES resize the scalar field to match the current cloud size
			\param uniqueName scalar field name (must be unique)
			\return index of this new scalar field (or -1 if an error occurred)
		**/
		virtual int addScalarField(const char* uniqueName)
		{
			//we don't accept two SF with the same name!
			if (getScalarFieldIndexByName(uniqueName) >= 0)
			{
				return -1;
			}

			//create requested scalar field
			ScalarField* sf = new ScalarField(uniqueName);
			if (!sf || (size() && !sf->resizeSafe(m_points.size())))
			{
				//Not enough memory!
				if (sf)
					sf->release();
				return -1;
			}

			try
			{
				//we don't want 'm_scalarFields' to grow by 50% each time! (default behavior of std::vector::push_back)
				m_scalarFields.resize(m_scalarFields.size() + 1, sf);
			}
			catch (const std::bad_alloc&) //out of memory
			{
				sf->release();
				return -1;
			}

			// Link the scalar field to this cloud
			sf->link();

			return static_cast<int>(m_scalarFields.size()) - 1;
		}

		//! Renames a specific scalar field
		/** Warning: name must not be already given to another SF!
			\param index scalar field index
			\param newName new name
			\return success
		**/
		bool renameScalarField(int index, const char* newName)
		{
			if (getScalarFieldIndexByName(newName) < 0)
			{
				ScalarField* sf = getScalarField(index);
				if (sf)
				{
					sf->setName(newName);
					return true;
				}
			}
			return false;
		}

		//! Deletes a specific scalar field
		/** WARNING: this operation may modify the scalar fields order
			(especially if the deleted SF is not the last one). However
			current IN & OUT scalar fields will stay up-to-date (while
			their index may change).
			\param index index of scalar field to be deleted
		**/
		virtual void deleteScalarField(int index)
		{
			int sfCount = static_cast<int>(m_scalarFields.size());
			if (index < 0 || index >= sfCount)
				return;

			//we update SF roles if they point to the deleted scalar field
			if (index == m_currentInScalarFieldIndex)
				m_currentInScalarFieldIndex = -1;
			if (index == m_currentOutScalarFieldIndex)
				m_currentOutScalarFieldIndex = -1;

			//if the deleted SF is not the last one, we swap it with the last element
			int lastIndex = sfCount - 1; //lastIndex>=0
			if (index < lastIndex) //i.e.lastIndex>0
			{
				std::swap(m_scalarFields[index], m_scalarFields[lastIndex]);
				//don't forget to update SF roles also if they point to the last element
				if (lastIndex == m_currentInScalarFieldIndex)
					m_currentInScalarFieldIndex = index;
				if (lastIndex == m_currentOutScalarFieldIndex)
					m_currentOutScalarFieldIndex = index;
			}

			//we can always delete the last element (and the vector stays consistent)
			m_scalarFields.back()->release();
			m_scalarFields.pop_back();
		}

		//! Deletes all scalar fields associated to this cloud
		virtual void deleteAllScalarFields()
		{
			m_currentInScalarFieldIndex = m_currentOutScalarFieldIndex = -1;

			while (!m_scalarFields.empty())
			{
				m_scalarFields.back()->release();
				m_scalarFields.pop_back();
			}
		}

	protected:
		//! Swaps two points (and their associated scalar values)
		virtual void swapPoints(unsigned firstIndex, unsigned secondIndex)
		{
			if (firstIndex == secondIndex)
			{
				// nothing to do
				return;
			}

			if (firstIndex >= m_points.size() ||	secondIndex >= m_points.size())
			{
				// invalid indexes
				assert(false);
				return;
			}

			std::swap(m_points[firstIndex], m_points[secondIndex]);

			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				m_scalarFields[i]->swap(firstIndex, secondIndex);
			}
		}

		//! Non-const access to a given (local) point
		/** WARNING: index must be valid
			\param index point index
			\return pointer to point data
		**/
		inline CCVector3* localPoint(unsigned index) { assert(index < size()); return &(m_points[index]); }

		//! Const access to a given (local) point
		/** WARNING: index must be valid
			\param index point index
			\return pointer to point data
		**/
		inline const CCVector3* localPoint(unsigned index) const { assert(index < size()); return &(m_points[index]); }

		//! Const access to a given (global) point
		/** WARNING: index must be valid
			\param index point index
			\return the global point
		**/
		inline CCVector3d globalPoint(unsigned index) const { assert(index < size()); return toGlobal(m_points[index]); }

	public: // member variables

		//! 3D (local) points
		std::vector<CCVector3> m_points;

		//! Translation from local to global coordinates
		CCVector3d m_fromLocalToGlobal;

		//! Bounding-box
		BoundingBox m_bbox;

		//! 'Iterator' on the point set
		unsigned m_currentPointIndex;

		//! Associated scalar fields
		std::vector<ScalarField*> m_scalarFields;

		//! Index of the current scalar field used for input
		int m_currentInScalarFieldIndex;

		//! Index of the current scalar field used for output
		int m_currentOutScalarFieldIndex;
	};
}
