/*
 * File:   GraphMLFormatExporter.hpp
 *
 * Author: MOLDOVAN Marius (moldovan@dbai.tuwien.ac.at)
 *
 * Copyright 2018, Marius Moldovan
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

#ifndef GRAPHMLFORMATEXPORTER_HPP
#define GRAPHMLFORMATEXPORTER_HPP

#include <htd_io/PreprocessorDefinitions.hpp>

#include <htd_io/ITreeDecompositionExporter.hpp>

#include <iostream>

namespace htd_io
{
    /**
     *  Exporter which allows to export tree decompositions in the tree decomposition format 'graphML'.
     *
     *  (See https://github.com/mabseher/htd/FORMATS.md for information about the available output formats.)
     */
    class GraphMLFormatExporter : public htd_io::ITreeDecompositionExporter
    {
        public:
            HTD_IO_API GraphMLFormatExporter(void);

            HTD_IO_API virtual ~GraphMLFormatExporter();

            HTD_IO_API void write(const htd::ITreeDecomposition & decomposition, const htd::IMultiHypergraph & graph, std::ostream & outputStream) const HTD_OVERRIDE;

            HTD_IO_API void write(const htd::ITreeDecomposition & decomposition, const htd::NamedMultiHypergraph<std::string, std::string> & graph, std::ostream & outputStream) const HTD_OVERRIDE;
    };
}

#endif /* HTD_IO_GRAPHMLFORMATEXPORTER_HPP */

