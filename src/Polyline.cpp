// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "Polyline.h"

using namespace CCCoreLib;

Polyline::Polyline(GenericIndexedCloudPersist* associatedCloud)
	: ReferenceCloud(associatedCloud)
	, m_isClosed(false)
{
}

void Polyline::clear(bool /*unusedParam*/)
{
	ReferenceCloud::clear();
	m_isClosed = false;
}
