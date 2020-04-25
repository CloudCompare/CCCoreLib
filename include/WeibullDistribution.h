// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "GenericDistribution.h"

namespace CCCoreLib
{
	//! The Weibull statistical parametric distribution
	/** Implements the GenericDistribution interface.
	**/
	class CC_CORE_LIB_API WeibullDistribution : public GenericDistribution
	{
	public:

		//! WeibullDistribution constructor
		WeibullDistribution();

		//! WeibullDistribution constructor
		/** Distrubtion parameters can be directly set during object
			construction.
			\param a the Weibull a parameter (also known as 'k')
			\param b the Weibull b parameter (also known as 'lambda')
			\param valueShift a value shift ('zero')
		**/
		WeibullDistribution(ScalarType a, ScalarType b, ScalarType valueShift = 0);

		//! Returns the distribution parameters
		/** \param a the Weibull a parameter (also known as 'k')
			\param b the Weibull b parameter (also known as 'lambda')
			\return the parameters validity
		**/
		bool getParameters(ScalarType &a, ScalarType &b) const;

		//! Returns the normal distribution equivalent parameters
		/** \param mu a field to transmit the equivalent mean
			\param sigma2 a field to transmit the equivalent variance
			\return the parameters validity
		**/
		bool getOtherParameters(ScalarType &mu, ScalarType &sigma2) const;

		//! Returns the distribution 'mode'
		double computeMode() const;
		//! Returns the distribution 'skewness'
		double computeSkewness() const;

		//! Sets the distribution parameters
		/** \param a the Weibull a parameter (also known as 'k')
			\param b the Weibull b parameter (also known as 'lambda')
			\param valueShift a value shift ('zero')
			\return the parameters validity
		**/
		bool setParameters(ScalarType a, ScalarType b, ScalarType valueShift = 0);

		//! Sets the distribution value shift
		/** \param vs value shift
		**/
		void setValueShift(ScalarType vs);

		//! Returns the distribution value shift
		inline ScalarType getValueShift() const { return m_valueShift; }

		//inherited methods (see GenericDistribution)
		bool computeParameters(const ScalarContainer& values) override;
		double computeP(ScalarType x) const override;
		double computePfromZero(ScalarType x) const override;
		double computeP(ScalarType x1, ScalarType x2) const override;
		double computeChi2Dist(const GenericCloud* cloud, unsigned numberOfClasses, int* histo = nullptr) override;
		const char* getName() const override { return "Weibull"; }

	protected:

		//! Compute each Chi2 class limits
		/** This method is used (internally) to accelerate the Chi2 distance computation.
			\param numberOfClasses the number of classes that will be used for Chi2 distance computation
			\return success
		**/
		virtual bool setChi2ClassesPositions(unsigned numberOfClasses);

		//! Chi2 classes limits
		/** Used internally. Stores both limits for each class in a vector
			(min_class_1, max_class_1, min_class_2, max_class_2, etc.).
		**/
		std::vector<ScalarType> chi2ClassesPositions;

		//! Parameters validity
		bool parametersDefined;
		//! Weibull distribution parameter a (k)
		ScalarType m_a;
		//! Weibull distribution parameter b (lambda)
		ScalarType m_b;
		//! Weibull distribution parameter 'value shift'
		ScalarType m_valueShift;

		//! Normal distribution equivalent parameter: mean
		ScalarType m_mu;
		//! Normal distribution equivalent parameter: variance
		ScalarType m_sigma2;

		//! internal function for parameters evaluation from sample points
		/** inverseVmax can be optionally specified for overflow-safe version
		**/
		static double ComputeG(const ScalarContainer& values, double a, ScalarType valueShift, double valueRange);
		//! internal function for parameters evaluation from sample points
		static double FindGRoot(const ScalarContainer& values, ScalarType valueShift, double valueRange);
	};
}
