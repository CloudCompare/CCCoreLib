// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "CCConst.h"
#include "CCShareable.h"

//System
#include <vector>

namespace CCCoreLib
{
	//! A simple scalar field (to be associated to a point cloud)
	/** A monodimensional array of scalar values. It has also specific
		parameters for display purposes.

		Invalid values can be represented by CCCoreLib::NAN_VALUE.
	**/
	class ScalarField : public std::vector<ScalarType>, public CCShareable
	{
	public:

		//! Default constructor
		/** [SHAREABLE] Call 'link' when associating this structure to an object.
			\param name scalar field name
		**/
		CC_CORE_LIB_API explicit ScalarField(const char* name = nullptr);

		//! Copy constructor
		/** \param sf scalar field to copy
			\warning May throw a std::bad_alloc exception
		**/
		CC_CORE_LIB_API ScalarField(const ScalarField& sf);

		//! Sets scalar field name
		CC_CORE_LIB_API void setName(const char* name);

		//! Returns scalar field name
		inline const char* getName() const { return m_name; }

		//! Returns the specific NaN value
		static inline ScalarType NaN() { return NAN_VALUE; }

		//! Computes the mean value (and optionally the variance value) of the scalar field
		/** \param mean a field to store the mean value
			\param variance if not void, the variance will be computed and stored here
		**/
		CC_CORE_LIB_API void computeMeanAndVariance(ScalarType &mean, ScalarType* variance = nullptr) const;

		//! Determines the min and max values
		CC_CORE_LIB_API virtual void computeMinAndMax();

		//! Returns whether a scalar value is valid or not
		static inline bool ValidValue(ScalarType value) { return value == value; } //'value == value' fails for NaN values

		//! Sets the value as 'invalid' (i.e. CCCoreLib::NAN_VALUE)
		inline void flagValueAsInvalid(std::size_t index) { at(index) = NaN(); }

		//! Returns the number of valid values in this scalar field
		CC_CORE_LIB_API std::size_t countValidValues() const;

		//! Returns the minimum value
		inline ScalarType getMin() const { return m_minVal; }
		//! Returns the maximum value
		inline ScalarType getMax() const { return m_maxVal; }

		//! Fills the array with a particular value
		inline void fill(ScalarType fillValue = 0) { if (empty()) resize(capacity(), fillValue); else std::fill(begin(), end(), fillValue); }

		//! Reserves memory (no exception thrown)
		CC_CORE_LIB_API bool reserveSafe(std::size_t count);
		//! Resizes memory (no exception thrown)
		CC_CORE_LIB_API bool resizeSafe(std::size_t count, bool initNewElements = false, ScalarType valueForNewElements = 0);

		//Shortcuts (for backward compatibility)
		inline ScalarType& getValue(std::size_t index) { return at(index); }
		inline const ScalarType& getValue(std::size_t index) const { return at(index); }
		inline void setValue(std::size_t index, ScalarType value) { at(index) = value; }
		inline void addElement(ScalarType value) { emplace_back(value); }
		inline unsigned currentSize() const { return static_cast<unsigned>(size()); }
		inline void swap(std::size_t i1, std::size_t i2) { std::swap(at(i1), at(i2)); }

	protected: //methods

		//! Default destructor
		/** Call release instead.
		**/
		CC_CORE_LIB_API ~ScalarField() override = default;

	protected: //members

		//! Scalar field name
		char m_name[256];

		//! Minimum value
		ScalarType m_minVal;
		//! Maximum value
		ScalarType m_maxVal;
	};

	inline void ScalarField::computeMinAndMax()
	{
		if (!empty())
		{
			bool minMaxInitialized = false;
			for (std::size_t i = 0; i < size(); ++i)
			{
				const ScalarType& val = at(i);
				if (ValidValue(val))
				{
					if (minMaxInitialized)
					{
						if (val < m_minVal)
							m_minVal = val;
						else if (val > m_maxVal)
							m_maxVal = val;
					}
					else
					{
						//first valid value is used to init min and max
						m_minVal = m_maxVal = val;
						minMaxInitialized = true;
					}
				}
			}
		}
		else //particular case: no value
		{
			m_minVal = m_maxVal = 0;
		}
	}
}
