// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <ScalarField.h>

//System
#include <cassert>
#include <cstring>

using namespace CCCoreLib;

ScalarField::ScalarField(const std::string& name/*=std::string()*/)
	: m_name{ }
	, m_offset{ 0.0 }
	, m_offsetHasBeenSet{ false }
	, m_localMinVal{ 0.0f }
	, m_localMaxVal{ 0.0f }
{
	setName(name);
}

ScalarField::ScalarField(const ScalarField& sf)
	: std::vector<float>(sf)
	, m_name{ sf.m_name }
	, m_offset{ sf.m_offset }
	, m_offsetHasBeenSet{ sf.m_offsetHasBeenSet }
	, m_localMinVal{ sf.m_localMinVal }
	, m_localMaxVal{ sf.m_localMaxVal }
{
}

void ScalarField::setName(const std::string& name)
{
	if (name.empty())
	{
		m_name = "Undefined";
	}
	else
	{
		m_name = name;
	}
}

std::size_t ScalarField::countValidValues() const
{
	if (false == std::isfinite(m_offset))
	{
		// special case: if the offset is invalid, all values become invalid!
		return size();
	}

	std::size_t count = 0;

	for (std::size_t i = 0; i < size(); ++i)
	{
		const ScalarType& val = at(i);
		if (ValidValue(val))
		{
			++count;
		}
	}

	return count;
}

void ScalarField::computeMeanAndVariance(ScalarType& mean, ScalarType* variance) const
{
	double _mean = 0.0;
	double _std2 = 0.0;
	std::size_t count = 0;

	for (std::size_t i = 0; i < size(); ++i)
	{
		float val = at(i);
		if (std::isfinite(val))
		{
			_mean += val;
			_std2 += static_cast<double>(val) * val;
			++count;
		}
	}

	if (count)
	{
		_mean /= count;
		mean = _mean;

		if (variance)
		{
			_std2 = std::abs(_std2 / count - _mean*_mean);
			*variance = static_cast<ScalarType>(_std2);
		}

		mean += m_offset; // only after the standard deviation has been calculated!

	}
	else
	{
		mean = 0;
		if (variance)
		{
			*variance = 0;
		}
	}
}

bool ScalarField::reserveSafe(std::size_t count)
{
	try
	{
		reserve(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}

bool ScalarField::resizeSafe(std::size_t count, bool initNewElements/*=false*/, ScalarType valueForNewElements/*=0*/)
{
	try
	{
		if (initNewElements && count > size())
		{
			float fillValueF = 0.0f;
			if (std::isfinite(valueForNewElements))
			{
				if (m_offsetHasBeenSet)
				{
					// use the already set offset
					fillValueF = static_cast<float>(valueForNewElements - m_offset);
				}
				else // if the offset has not been set yet...
				{
					if (valueForNewElements == 0)
					{
						// special case: filling with zeros
						// (it doesn't really give an idea of what the optimal offset is)
						m_offset = 0.0;
					}
					else
					{
						// we use the first finite value as offset by default
						setOffset(valueForNewElements);
					}
				}
			}
			else
			{
				// special case: filling with NaN or +/-inf values
				fillValueF = static_cast<float>(valueForNewElements); // NaN/-inf/+inf should be maintained
			}

			resize(count, fillValueF);
		}
		else
		{
			resize(count);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}
