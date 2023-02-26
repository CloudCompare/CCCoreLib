// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "CCGeom.h"
#include "CCToolbox.h"

namespace CCCoreLib
{
	//! Miscellaneous useful functions (geometrical elements handling)
	class CC_CORE_LIB_API CCMiscTools : public CCToolbox
	{
	public:

		//! Proportionally enlarges a 3D box
		/** \param dimMin the upper-left corner of the box
			\param dimMax the lower-right corner of the box
			\param coef the enlargement coefficient (1.1 <-> +10%)
		**/
		template<typename T> static void EnlargeBox(	Vector3Tpl<T>& dimMin,
														Vector3Tpl<T>& dimMax,
														double coef)
		{
			Vector3Tpl<T> dd = (dimMax - dimMin) * static_cast<T>(1.0 + coef);
			Vector3Tpl<T> md = dimMax + dimMin;

			dimMin = (md - dd) * static_cast<T>(0.5);
			dimMax = dimMin + dd;
		}

		//! Transforms a 3D box into a 3D cube
		/** The cube dimensions will be equal to the largest box dimension.
			\param dimMin the upper-left corner of the rectangle
			\param dimMax the lower-right corner of the rectangle
			\param enlargeFactor the resulting box can be automatically enlarged if this parameter is greater than 0
		**/
		template<typename T> static void MakeMinAndMaxCubical(	Vector3Tpl<T>& dimMin,
																Vector3Tpl<T>& dimMax,
																double enlargeFactor)
		{
			//get box max dimension
			T maxDD = 0;
			{
				Vector3Tpl<T> diag = dimMax - dimMin;
				maxDD = std::max(diag.x, diag.y);
				maxDD = std::max(maxDD, diag.z);
			}

			//enlarge it if necessary
			if (enlargeFactor > 0)
			{
				maxDD = static_cast<T>(maxDD * (1.0 + enlargeFactor));
			}

			//build corresponding 'square' box
			{
				Vector3Tpl<T> dd(maxDD, maxDD, maxDD);
				Vector3Tpl<T> md = dimMax + dimMin;

				dimMin = (md - dd) * static_cast<T>(0.5);
				dimMax = dimMin + dd;
			}
		}

		//! Computes base vectors for a given 3D plane
		/** Determines at least two orthogonal vectors perpendicular to a third one
			\param[in] N a non null vector
			\param[out] X the first vector (a 3 coordinates array to be updated by the algorithm)
			\param[out] Y the second vector (a 3 coordinates array to be updated by the algorithm)
		**/
		static void ComputeBaseVectors(	const CCVector3 &N,
										CCVector3& X,
										CCVector3& Y);
		//! Computes base vectors for a given 3D plane - double version
		/** Determines at least two orthogonal vectors perpendicular to a third one
			\param[in] N a non null vector
			\param[out] X the first vector (a 3 coordinates array to be updated by the algorithm)
			\param[out] Y the second vector (a 3 coordinates array to be updated by the algorithm)
		**/
		static void ComputeBaseVectors(	const CCVector3d &N,
										CCVector3d& X,
										CCVector3d& Y);

		//! Ovelap test between a 3D box and a triangle
		/** \param boxcenter the box center
		\param boxhalfsize the box half dimensions
		\param triverts the 3 triangle vertices
		\return true if the input box and triangle overlap, false otherwise
		**/
		static bool TriBoxOverlap(const CCVector3& boxcenter,
								  const CCVector3& boxhalfsize,
								  const CCVector3* triverts[3]);

		//! Ovelap test between a 3D box and a triangle (double version)
		/** \param boxcenter the box center
			\param boxhalfsize the box half dimensions
			\param triverts the 3 triangle vertices
			\return true if the input box and triangle overlap, false otherwise
		**/
		static bool TriBoxOverlapd(const CCVector3d& boxcenter,
								   const CCVector3d& boxhalfsize,
								   const CCVector3d triverts[3]);
	};
}
