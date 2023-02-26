// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "GenericTriangle.h"

namespace CCCoreLib
{
	//! A simple triangle class
	/** Implements the GenericTriangleTpl interface with references to 3D points.
		WARNING: make sure that references don't point to temporary objects!
		WARNING: not compatible with parallelization.
	**/
	template <typename Type> class SimpleRefTriangleTpl : public GenericTriangleTpl<Type>
	{
	public:

		//! Default constructor
		SimpleRefTriangleTpl()
			: A(nullptr)
			, B(nullptr)
			, C(nullptr)
		{}

		//! Constructor from 3 vertices (references to)
		/** \param _A first vertex
			\param _B second vertex
			\param _C third vertex
		**/
		SimpleRefTriangleTpl(const Vector3Tpl<Type>* _A, const Vector3Tpl<Type>* _B, const Vector3Tpl<Type>* _C)
			: A(_A)
			, B(_B)
			, C(_C)
		{}

		//inherited methods (see GenericTriangleTpl)
		inline const Vector3Tpl<Type>* _getA() const override { return A; }
		inline const Vector3Tpl<Type>* _getB() const override { return B; }
		inline const Vector3Tpl<Type>* _getC() const override { return C; }

		//! A vertex (ref)
		const Vector3Tpl<Type> *A;
		//! B vertex (ref)
		const Vector3Tpl<Type> *B;
		//! C vertex (ref)
		const Vector3Tpl<Type> *C;
	};

	//! Simple local triangle
	using SimpleLocalRefTriangle = SimpleRefTriangleTpl<PointCoordinateType>;
	//! Simple global triangle
	using SimpleGlobalRefTriangle = SimpleRefTriangleTpl<double>;

	//! A simple triangle class
	/** Implements the GenericTriangleTpl interface with a triplet of 3D points.
		Relies on direct storage for speed enhancement and parallelization!
	**/
	template <typename Type> class SimpleTriangleTpl : public GenericTriangleTpl<Type>
	{
	public:

		//! Default constructor
		SimpleTriangleTpl()
			: A(0, 0, 0)
			, B(0, 0, 0)
			, C(0, 0, 0)
		{}

		//! Constructor from 3 vertices
		/** \param _A first vertex
			\param _B second vertex
			\param _C third vertex
		**/
		SimpleTriangleTpl(const Vector3Tpl<Type>& _A, const Vector3Tpl<Type>& _B, const Vector3Tpl<Type>& _C)
			: A(_A)
			, B(_B)
			, C(_C)
		{}

		//inherited methods (see GenericTriangleTpl)
		inline const Vector3Tpl<Type>* _getA() const override { return &A; }
		inline const Vector3Tpl<Type>* _getB() const override { return &B; }
		inline const Vector3Tpl<Type>* _getC() const override { return &C; }

		//! A vertex
		Vector3Tpl<Type> A;
		//! B vertex
		Vector3Tpl<Type> B;
		//! C vertex
		Vector3Tpl<Type> C;
	};

	//! Simple local triangle
	using SimpleLocalTriangle = SimpleTriangleTpl<PointCoordinateType>;
	//! Simple global triangle
	using SimpleGlobalTriangle = SimpleTriangleTpl<double>;
}
