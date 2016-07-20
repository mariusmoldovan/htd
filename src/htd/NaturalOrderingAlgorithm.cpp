/* 
 * File:   NaturalOrderingAlgorithm.cpp
 *
 * Author: ABSEHER Michael (abseher@dbai.tuwien.ac.at)
 * 
 * Copyright 2015-2016, Michael Abseher
 *    E-Mail: <abseher@dbai.tuwien.ac.at>
 * 
 * This file is part of htd.
 * 
 * htd is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free 
 * Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * htd is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public 
 * License for more details.

 * You should have received a copy of the GNU General Public License
 * along with htd.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HTD_HTD_NATURALORDERINGALGORITHM_CPP
#define	HTD_HTD_NATURALORDERINGALGORITHM_CPP

#include <htd/Globals.hpp>
#include <htd/NaturalOrderingAlgorithm.hpp>
#include <htd/VectorAdapter.hpp>

#include <algorithm>

htd::NaturalOrderingAlgorithm::NaturalOrderingAlgorithm(void) : htd::LibraryObject()
{
    
}
            
htd::NaturalOrderingAlgorithm::~NaturalOrderingAlgorithm()
{
    
}

htd::ConstCollection<htd::vertex_t> htd::NaturalOrderingAlgorithm::computeOrdering(const htd::IMultiHypergraph & graph) const HTD_NOEXCEPT
{
    htd::VectorAdapter<htd::vertex_t> ret(graph.vertices());

    return htd::ConstCollection<htd::id_t>::getInstance(ret);
}

void htd::NaturalOrderingAlgorithm::writeOrderingTo(const htd::IMultiHypergraph & graph, std::vector<htd::vertex_t> & target) const HTD_NOEXCEPT
{
    const htd::ConstCollection<htd::vertex_t> & vertexCollection = graph.vertices();

    std::copy(vertexCollection.begin(), vertexCollection.end(), std::back_inserter(target));
}

htd::NaturalOrderingAlgorithm * htd::NaturalOrderingAlgorithm::clone(void) const
{
    htd::NaturalOrderingAlgorithm * ret = new htd::NaturalOrderingAlgorithm();

    ret->setManagementInstance(managementInstance());

    return ret;
}

#endif /* HTD_HTD_NATURALORDERINGALGORITHM_CPP */
