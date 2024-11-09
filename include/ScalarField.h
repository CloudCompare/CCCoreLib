// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "CCConst.h"
#include "CCShareable.h"

//System
#include <vector>
#include <string>

namespace CCCoreLib
{
	//! A simple scalar field (to be associated to a point cloud)
	/** A monodimensional array of scalar values. It has also specific
		parameters for display purposes.

		It is now using a base offset value of type double, and internally
		stores the values as float to limit the memory consumption.

		Invalid values can be represented by CCCoreLib::NAN_VALUE.
	**/
	class ScalarField : protected std::vector<float>, public CCShareable
	{
	public:

		//! Shortcut to the (protected) std::vector::size() method
		using std::vector<float>::size;
		//! Shortcut to the (protected) std::vector::capacity() method
		using std::vector<float>::capacity;
		//! Shortcut to the (protected) std::vector::reserve() method
		using std::vector<float>::reserve;
		//! Shortcut to the (protected) std::vector::shrink_to_fit() method
		using std::vector<float>::shrink_to_fit;
		//! Shortcut to the (protected) std::vector::empty() method
		using std::vector<float>::empty;

		//! Default constructor
		/** [SHAREABLE] Call 'link' when associating this structure to an object.
			\param name scalar field name
		**/
		CC_CORE_LIB_API explicit ScalarField(const std::string& name = std::string());

		//! Copy constructor
		/** \param sf scalar field to copy
			\warning May throw a std::bad_alloc exception
		**/
		CC_CORE_LIB_API ScalarField(const ScalarField& sf);

		//! Sets scalar field name
		CC_CORE_LIB_API void setName(const std::string& name);

		//! Returns scalar field name
		inline const std::string& getName() const { return m_name; }

		//! Returns the specific NaN value
		static inline ScalarType NaN() { return NAN_VALUE; }

		//! Returns the offset
		inline double getOffset() const { return m_offset; }

		//! Sets the offset
		/** \warning Dangerous. All values inside the vector are relative to the offset!
			\param offset the new offset value
		**/
		inline void setOffset(double offset)
		{
			m_offsetHasBeenSet = true;
			m_offset = offset;
		}

		//! Clears the scalar field
		inline void clear()
		{
			std::vector<float>::clear();
			m_offsetHasBeenSet = false;
		}

		//! Computes the mean value (and optionally the variance value) of the scalar field
		/** \param mean a field to store the mean value
			\param variance if not void, the variance will be computed and stored here
		**/
		CC_CORE_LIB_API void computeMeanAndVariance(ScalarType& mean, ScalarType* variance = nullptr) const;

		//! Determines the min and max values
		CC_CORE_LIB_API virtual void computeMinAndMax();

		//! Returns whether a scalar value is valid or not
		static inline bool ValidValue(ScalarType value) { return std::isfinite(value); }

		//! Sets the value as 'invalid' (i.e. CCCoreLib::NAN_VALUE)
		inline void flagValueAsInvalid(std::size_t index) { (*this)[index] = std::numeric_limits<float>::quiet_NaN(); }

		//! Returns the number of valid values in this scalar field
		CC_CORE_LIB_API std::size_t countValidValues() const;

		//! Returns the minimum value
		inline ScalarType getMin() const { return m_offset + m_localMinVal; }
		//! Returns the maximum value
		inline ScalarType getMax() const { return m_offset + m_localMaxVal; }

		//! Fills the array with a particular value
		inline void fill(ScalarType fillValue = 0)
		{
			float fillValueF = 0.0f;
			if (m_offsetHasBeenSet)
			{
				fillValueF = static_cast<float>(fillValue - m_offset);
			}
			else
			{
				setOffset(fillValue);
			}

			if (empty())
			{
				resize(capacity(), fillValueF);
			}
			else
			{
				std::fill(begin(), end(), fillValueF);
			}
		}

		//! Reserves memory (no exception thrown)
		CC_CORE_LIB_API bool reserveSafe(std::size_t count);
		//! Resizes memory (no exception thrown)
		CC_CORE_LIB_API bool resizeSafe(std::size_t count, bool initNewElements = false, ScalarType valueForNewElements = 0);

		//Shortcuts (for backward compatibility)
		inline ScalarType getValue(std::size_t index) const { return m_offset + (*this)[index]; }

		inline float getLocalValue(std::size_t index) const { return (*this)[index]; }
		inline void setLocalValue(std::size_t index, float value) { (*this)[index] = value; }
		inline const float* getLocalValues() const { return data(); }

		inline void setValue(std::size_t index, ScalarType value)
		{
			if (m_offsetHasBeenSet)
			{
				(*this)[index] = static_cast<float>(value - m_offset);
			}
			else
			{
				setOffset(value);
				(*this)[index] = 0.0f;
			}
		}

		inline void addElement(ScalarType value)
		{
			if (m_offsetHasBeenSet)
			{
				push_back(static_cast<float>(value - m_offset));
			}
			else
			{
				// if the offset has not been set yet, we use the first value by default
				setOffset(value);
				push_back(0.0f);
			}
		}

		inline unsigned currentSize() const { return static_cast<unsigned>(size()); }

		inline void swap(std::size_t i1, std::size_t i2) { std::swap(at(i1), at(i2)); }

	protected: //methods

		//! Default destructor
		/** Call release instead.
		**/
		CC_CORE_LIB_API ~ScalarField() override = default;

	protected: //members

		//! Scalar field name
		std::string m_name;

	private:
		//! Offset value (local to global)
		double m_offset;
		//! Whether the offset has been set or not
		bool m_offsetHasBeenSet;
		//! Minimum value (local)
		float m_localMinVal;
		//! Maximum value (local)
		float m_localMaxVal;
	};

	inline void ScalarField::computeMinAndMax()
	{
		float localMinVal = 0.0f;
		float localMaxVal = 0.0f;

		bool minMaxInitialized = false;
		for (std::size_t i = 0; i < size(); ++i)
		{
			float val = at(i);
			if (std::isfinite(val))
			{
				if (minMaxInitialized)
				{
					if (val < localMinVal)
						localMinVal = val;
					else if (val > localMaxVal)
						localMaxVal = val;
				}
				else
				{
					//first valid value is used to init min and max
					localMinVal = localMaxVal = val;
					minMaxInitialized = true;
				}
			}
		}

		if (minMaxInitialized)
		{
			m_localMinVal = localMinVal;
			m_localMaxVal = localMaxVal;
		}
		else //particular case: zero valid values
		{
			m_localMinVal = m_localMaxVal = 0.0;
		}
	}
}
