/*
 * File:   ExternalOrderingAlgorithm.hpp
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

#ifndef EXTERNALORDERINGALGORITHM_HPP
#define EXTERNALORDERINGALGORITHM_HPP

#include <htd/Globals.hpp>
#include <htd/IOrderingAlgorithm.hpp>

#include <vector>
#include <algorithm>
#include <unordered_set>

namespace htd
{
    /**
     *  Implementation of the IOrderingAlgorithm interface based on an external elimination ordering algorithm.
     */
    class ExternalOrderingAlgorithm : public htd::IOrderingAlgorithm
    {
        public:
            /**
             *  Constructor for a new ordering algorithm of type ExtrernalOrderingAlgorithm.
             *
             *  @param[in] manager   The management instance to which the new algorithm belongs.
             */
            HTD_API ExternalOrderingAlgorithm(const htd::LibraryInstance * const manager);

            HTD_API virtual ~ExternalOrderingAlgorithm();

            HTD_API void writeOrderingTo(const htd::IMultiHypergraph & graph, std::vector<htd::vertex_t> & target) const HTD_NOEXCEPT HTD_OVERRIDE;

            HTD_API const htd::LibraryInstance * managementInstance(void) const HTD_NOEXCEPT HTD_OVERRIDE;

            HTD_API void setManagementInstance(const htd::LibraryInstance * const manager) HTD_OVERRIDE;

            HTD_API ExternalOrderingAlgorithm * clone(void) const HTD_OVERRIDE;

#ifndef HTD_USE_VISUAL_STUDIO_COMPATIBILITY_MODE

#else

#endif

        protected:
            /**
             *  Copy assignment operator for an ordering algorithm.
             *
             *  @note This operator is protected to prevent assignments to an already initialized algorithm.
             */
            ExternalOrderingAlgorithm & operator=(const ExternalOrderingAlgorithm &) { return *this; }

        private:
            struct Implementation;

            std::unique_ptr<Implementation> implementation_;
    };
}

#endif /* EXTERNALORDERINGALGORITHM_HPP */
