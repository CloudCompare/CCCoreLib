// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//local
#include "ScalarField.h"

//STL
#include <unordered_set>

namespace CCCoreLib
{
	//! Garbage container (automatically deletes pointers when destroyed)
	template<typename C> class Garbage
	{
	public:
		//! Puts an item in the trash
		inline void add(C* item)
		{
			try
			{
				m_items.insert(item);
			}
			catch (const std::bad_alloc&)
			{
				//what can we do?!
			}
		}
		
		//! Removes an item from the trash
		/** \warning The item won't be destroyed! **/
		inline void remove(C* item)
		{
			m_items.erase(item);
		}
		
		//! To manually delete an item already in the trash
		inline void destroy(C* item)
		{
			m_items.erase(item);
			delete item;
		}
		
		//! Destructor
		/** Automatically deletes all items **/
		~Garbage()
		{
			//dispose of left over
			for (auto it = m_items.begin(); it != m_items.end(); ++it)
				delete *it;
			m_items.clear();
		}
		
		//! Items to delete
		std::unordered_set<C*> m_items;
	};
	
	//! Specialization for ScalarFields
	template <> class Garbage<ScalarField>
	{
	public:
		//! Puts an item in the trash
		inline void add(ScalarField* item)
		{
			try
			{
				m_items.insert(item);
			}
			catch (const std::bad_alloc&)
			{
				//what can we do?!
			}
		}
		
		//! Removes an item from the trash
		/** \warning The item won't be destroyed! **/
		inline void remove(ScalarField* item)
		{
			m_items.erase(item);
		}
		
		//! Manually deltes an item already in the trash
		inline void destroy(ScalarField* item)
		{
			m_items.erase(item);
			item->release();
		}
		
		//! Destructor
		/** Automatically deletes all items **/
		~Garbage()
		{
			//dispose of left over
			for (auto item : m_items)
			{
				item->release();
			}
			m_items.clear();
		}
		
		//! Items to delete
		std::unordered_set<ScalarField*> m_items;
	};
}
