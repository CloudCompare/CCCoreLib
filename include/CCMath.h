// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © CloudCompare Project

#pragma once

//! \file CCMath.h
//! Methods for conversion and comparison. Note that these are intenionally not
//! templates. They are short methods and templates are overkill for these cases.

#include "CCConst.h"


namespace CCCoreLib
{
	//! Test a floating point number against epsilon (a very small number).
	/*!
	  \param x The number to test
	  \return True if the number is less than epsilon.
	*/	
	inline bool	lessThanEpsilon( float x )
	{
		return x < std::numeric_limits<float>::epsilon();
	}
	
	//! Test a floating point number against epsilon (a very small number).
	/*!
	  \param x The number to test
	  \return True if the number is less than epsilon.
	*/	
	inline bool	lessThanEpsilon( double x )
	{
		return x < std::numeric_limits<double>::epsilon();
	}
	
	//! Test a floating point number against epsilon (a very small number).
	/*!
	  \param x The number to test
	  \return True if the number is greater than epsilon.
	*/	
	inline bool	greaterThanEpsilon( float x )
	{
		return x > std::numeric_limits<float>::epsilon();
	}
	
	//! Test a floating point number against epsilon (a very small number).
	/*!
	  \param x The number to test
	  \return True if the number is greater than epsilon.
	*/	
	inline bool	greaterThanEpsilon( double x )
	{
		return x > std::numeric_limits<double>::epsilon();
	}
	
	//! Convert radians to degrees.
	/*!
	  \param radians Radians to convert
	  \return \a radians represented as degrees.
	*/	
	inline float radiansToDegrees( float radians )
	{		
		return radians * (180.0f/M_PI);
	}
	
	//! Convert radians to degrees.
	/*!
	  \param radians Radians to convert
	  \return \a radians represented as degrees.
	*/	
	inline double radiansToDegrees( double radians )
	{		
		return radians * (180.0/M_PI);
	}
	
	//! Convert degrees to radians.
	/*!
	  \param degrees Degrees to convert
	  \return \a degrees represented as radians.
	*/	
	inline float degreesToRadians( float degrees )
	{		
		return degrees * (180.0f/M_PI);
	}
	
	//! Convert degrees to radians.
	/*!
	  \param degrees Degrees to convert
	  \return \a degrees represented as radians.
	*/	
	inline double degreesToRadians( double degrees )
	{		
		return degrees * (180.0/M_PI);
	}
}
