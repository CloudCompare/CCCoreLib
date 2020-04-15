// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#ifndef CC_POLYLINE_HEADER
#define CC_POLYLINE_HEADER

//Local
#include "ReferenceCloud.h"

namespace CCLib
{

	//! A simple polyline class
	/** The polyline is considered as a cloud of points
		(in a specific order) with a open/closed state
		information.
	**/
	class CC_CORE_LIB_API Polyline : public ReferenceCloud
	{
	public:

		//! Polyline constructor
		explicit Polyline(GenericIndexedCloudPersist* associatedCloud);

		//! Returns whether the polyline is closed or not
		inline bool isClosed() const { return m_isClosed; }

		//! Sets whether the polyline is closed or not
		inline void setClosed(bool state) { m_isClosed = state; }

		//inherited from ReferenceCloud
		void clear(bool unusedParam = true) override;

	protected:

		//! Closing state
		bool m_isClosed;
	};

}

#endif //CC_POLYLINE_HEADER
