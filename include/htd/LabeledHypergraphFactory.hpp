/*
 * File:   LabeledHypergraphFactory.hpp
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

#ifndef HTD_HTD_LABELEDHYPERGRAPHFACTORY_HPP
#define HTD_HTD_LABELEDHYPERGRAPHFACTORY_HPP

#include <htd/Globals.hpp>
#include <htd/IMutableLabeledHypergraph.hpp>

namespace htd
{
    class LabeledHypergraphFactory
    {
        public:
            /**
             *  Destructor of the factory class.
             */
            ~LabeledHypergraphFactory();

            /**
             *  Access the singleton instance of the factory class.
             *
             *  @return The singleton instance of the factory class.
             */
            static LabeledHypergraphFactory & instance(void);

            htd::IMutableLabeledHypergraph * getLabeledHypergraph(void);

            htd::IMutableLabeledHypergraph * getLabeledHypergraph(std::size_t initialSize);

            htd::IMutableLabeledHypergraph * getLabeledHypergraph(const htd::ILabeledHypergraph & original);

            htd::IMutableLabeledHypergraph * getLabeledHypergraph(const htd::ILabeledMultiHypergraph & original);

            void setConstructionTemplate(htd::IMutableLabeledHypergraph * original);

            htd::IMutableLabeledHypergraph & accessMutableLabeledHypergraph(htd::ILabeledHypergraph & original);

            const htd::IMutableLabeledHypergraph & accessMutableLabeledHypergraph(const htd::ILabeledHypergraph & original);

        private:
            htd::IMutableLabeledHypergraph * constructionTemplate_;

            LabeledHypergraphFactory(void);

            LabeledHypergraphFactory(const LabeledHypergraphFactory & original);

            LabeledHypergraphFactory & operator=(const LabeledHypergraphFactory & original);
    };
}

#endif /* HTD_HTD_LABELEDHYPERGRAPHFACTORY_HPP */
