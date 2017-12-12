/*
 * File:   ExternalOrderingAlgorithm.cpp
 *
 * Author: MOLDOVAN Marius (moldovan@dbai.tuwien.ac.at)
 *
 * Copyright 2017, Marius Moldovan
 *    E-Mail: <moldovan@dbai.tuwien.ac.at>
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

#ifndef HTD_HTD_EXTERNALORDERINGALGORITHM_CPP
#define HTD_HTD_EXTERNALORDERINGALGORITHM_CPP

#include <htd/Globals.hpp>
#include <htd/Helpers.hpp>
#include <htd/MinDegreeOrderingAlgorithm.hpp>
#include <htd/GraphPreprocessorFactory.hpp>
#include <htd/IGraphPreprocessor.hpp>
#include <htd/VertexOrdering.hpp>
#include <htd/PriorityQueue.hpp>

#include <algorithm>
#include <unordered_set>

/**
 *  Private implementation details of class htd::MinDegreeOrderingAlgorithm.
 */
struct htd::ExternalOrderingAlgorithm::Implementation
{
    /**
     *  Constructor for the implementation details structure.
     *
     *  @param[in] manager   The management instance to which the current object instance belongs.
     */
    Implementation(const htd::LibraryInstance * const manager) : managementInstance_(manager)
    {

    }

    virtual ~Implementation()
    {

    }

    /**
     *  The management instance to which the current object instance belongs.
     */
    const htd::LibraryInstance * managementInstance_;

    /**
     *  Compute the vertex ordering of a given graph and write it to the end of a given vector.
     *
     *  @param[in] preprocessedGraph    The input graph in preprocessed format.
     *  @param[out] target              The target vector to which the computed ordering shall be appended.
     *  @param[in] maxBagSize           The upper bound for the maximum bag size of a decomposition based on the resulting ordering.
     *
     *  @return The maximum bag size of the decomposition which is obtained via bucket elimination using the input graph and the resulting ordering.
     */
    std::size_t writeOrderingTo(const htd::IPreprocessedGraph & preprocessedGraph, std::vector<htd::vertex_t> & target, std::size_t maxBagSize) const HTD_NOEXCEPT;
};

htd::ExternalOrderingAlgorithm::ExternalOrderingAlgorithm(const htd::LibraryInstance * const manager) : implementation_(new Implementation(manager))
{

}

htd::ExternalOrderingAlgorithm::~ExternalOrderingAlgorithm()
{

}

htd::IVertexOrdering * htd::ExternalOrderingAlgorithm::computeOrdering(const htd::IMultiHypergraph & graph, htd::IVertexOrdering ordering) const HTD_NOEXCEPT
{
    return ordering;
}

const htd::LibraryInstance * htd::ExternalOrderingAlgorithm::managementInstance(void) const HTD_NOEXCEPT
{
    return implementation_->managementInstance_;
}

void htd::ExternalOrderingAlgorithm::setManagementInstance(const htd::LibraryInstance * const manager)
{
    HTD_ASSERT(manager != nullptr)

    implementation_->managementInstance_ = manager;
}

htd::ExternalOrderingAlgorithm * htd::ExternalOrderingAlgorithm::clone(void) const
{
    return new htd::ExternalOrderingAlgorithm(implementation_->managementInstance_);
}

#ifdef HTD_USE_VISUAL_STUDIO_COMPATIBILITY_MODE
htd::IOrderingAlgorithm * htd::ExternalOrderingAlgorithm::cloneOrderingAlgorithm(void) const
{
    return new htd::ExternalOrderingAlgorithm(implementation_->managementInstance_);
}

#endif

#endif /* HTD_HTD_EXTERNALORDERINGALGORITHM_CPP */
