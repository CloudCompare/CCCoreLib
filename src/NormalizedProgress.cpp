// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "GenericProgressCallback.h"

//system
#include <mutex>
#include <cassert>

using namespace CCCoreLib;

// Use a class "wrapper" to avoid having to include <mutex> in header
class CCCoreLib::StdMutex : public std::mutex {};

NormalizedProgress::NormalizedProgress(	GenericProgressCallback* callback,
										unsigned totalSteps,
										unsigned totalPercentage/*=100*/)
	: m_percent(0)
	, m_step(1)
	, m_percentAdd(1.0f)
	, m_counter(0)
	, m_mutex(new StdMutex)
	, progressCallback(callback)
{
	scale(totalSteps, totalPercentage);
}

NormalizedProgress::~NormalizedProgress()
{
	delete m_mutex;
	m_mutex = nullptr;
}

void NormalizedProgress::scale(	unsigned totalSteps,
								unsigned totalPercentage/*=100*/,
								bool updateCurrentProgress/*=false*/)
{
	if (progressCallback)
	{
		if (totalSteps == 0 || totalPercentage == 0)
		{
			m_step = 1;
			m_percentAdd = 0;
			return;
		}

		if (totalSteps >= 2 * totalPercentage)
		{
			m_step = static_cast<unsigned>(ceil(static_cast<float>(totalSteps) / totalPercentage));
			assert(m_step != 0 && m_step < totalSteps);
			m_percentAdd = static_cast<float>(totalPercentage) / (totalSteps / m_step);
		}
		else
		{
			m_step = 1;
			m_percentAdd = static_cast<float>(totalPercentage) / totalSteps;
		}

		m_mutex->lock();
		if (updateCurrentProgress)
		{
			m_percent = (static_cast<float>(totalPercentage) / totalSteps) * m_counter;
		}
		else
		{
			m_counter = 0;
		}
		m_mutex->unlock();
	}
}

void NormalizedProgress::reset()
{
	m_mutex->lock();
	m_percent = 0;
	m_counter = 0;
	if (progressCallback)
	{
		progressCallback->update(0);
	}
	m_mutex->unlock();
}

bool NormalizedProgress::oneStep()
{
	if (!progressCallback)
	{
		return true;
	}

	m_mutex->lock();
	if ((++m_counter % m_step) == 0)
	{
		m_percent += m_percentAdd;
		progressCallback->update(m_percent);
	}
	m_mutex->unlock();

	return !progressCallback->isCancelRequested();
}

bool NormalizedProgress::steps(unsigned n)
{
	if (!progressCallback)
	{
		return true;
	}

	m_mutex->lock();
	m_counter += n;
	unsigned d1 = m_counter / m_step;
	unsigned d2 = (m_counter + n) / m_step;
	if (d2 != d1)
	{
		m_percent += static_cast<float>(d2 - d1) * m_percentAdd;
		progressCallback->update(m_percent);
	}
	m_mutex->unlock();

	return !progressCallback->isCancelRequested();
}
