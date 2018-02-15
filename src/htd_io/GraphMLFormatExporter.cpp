/*
 * File:   GraphMLFormatExporter.cpp
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

#ifndef HTD_IO_GRAPHMLFORMATEXPORTER_CPP
#define HTD_IO_GRAPHMLFORMATEXPORTER_CPP

#include <htd_io/GraphMLFormatExporter.hpp>

#include <sstream>
#include <unordered_map>

htd_io::GraphMLFormatExporter::GraphMLFormatExporter(void)
{

}

htd_io::GraphMLFormatExporter::~GraphMLFormatExporter()
{

}

void htd_io::GraphMLFormatExporter::write(const htd::ITreeDecomposition & decomposition, const htd::IMultiHypergraph & graph, std::ostream & outputStream) const
{
    outputStream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << "\n"
        << "<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\"" << "\n"
        << "         xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"" << "\n"
        << "         xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns "
           "http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">" << "\n"
        << "  <key id=\"bag\" for=\"node\"/>" << "\n"
        << "  <graph edgedefault=\"undirected\">" << "\n";

    std::unordered_map<htd::vertex_t, std::size_t> indices;

    if (decomposition.vertexCount() > 0)
    {
        std::size_t index = 1;

        std::stringstream tmpStream;

        for (htd::vertex_t node : decomposition.vertices())
        {
            tmpStream << "    <node id=\"n" << index << "\">" << "\n";
            tmpStream << "      <data key=\"bag\">";

            std::string separator;

            indices.emplace(node, index);

            for (htd::vertex_t vertex : decomposition.bagContent(node))
            {
                tmpStream << separator << vertex;
                separator = ", ";
            }

            tmpStream << "</data>" << "\n";
            tmpStream << "    </node>" << "\n";

            outputStream << tmpStream.rdbuf();

            tmpStream.clear();

            ++index;
        }

        const htd::ConstCollection<htd::Hyperedge> & hyperedgeCollection = decomposition.hyperedges();

        std::size_t edgeCount = decomposition.edgeCount();

        auto it = hyperedgeCollection.begin();

        for (htd::index_t index = 0; index < edgeCount; ++index)
        {
            htd::vertex_t vertex1 = indices.at((*it)[0]);
            htd::vertex_t vertex2 = indices.at((*it)[1]);

            outputStream << "    <edge source=\"n" << vertex1 << "\""
                                    " target=\"n" << vertex2 << "\"/>" << "\n";

            ++it;
        }
    }

    outputStream << "  </graph>" << std::endl
        << "</graphml>" << std::endl;
}

void htd_io::GraphMLFormatExporter::write(const htd::ITreeDecomposition & decomposition, const htd::NamedMultiHypergraph<std::string, std::string> & graph, std::ostream & outputStream) const
{
    outputStream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << "\n"
        << "<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\"" << "\n"
        << "         xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"" << "\n"
        << "         xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns "
           "http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">" << "\n"
        << "  <key id=\"bag\" for=\"node\"/>" << "\n"
        << "  <graph edgedefault=\"undirected\">" << "\n";

    std::unordered_map<htd::vertex_t, std::size_t> indices;

    if (decomposition.vertexCount() > 0)
    {
        std::size_t index = 1;

        std::stringstream tmpStream;

        for (htd::vertex_t node : decomposition.vertices())
        {
            tmpStream << "    <node id=\"n" << index << "\">" << "\n";
            tmpStream << "      <data key=\"bag\">";

            std::string separator;

            indices.emplace(node, index);

            for (htd::vertex_t vertex : decomposition.bagContent(node))
            {
                tmpStream << separator << vertex;
                separator = ", ";
            }

            tmpStream << "</data>" << "\n";
            tmpStream << "    </node>" << "\n";

            outputStream << tmpStream.rdbuf();

            tmpStream.clear();

            ++index;
        }

        const htd::ConstCollection<htd::Hyperedge> & hyperedgeCollection = decomposition.hyperedges();

        std::size_t edgeCount = decomposition.edgeCount();

        auto it = hyperedgeCollection.begin();

        for (htd::index_t index = 0; index < edgeCount; ++index)
        {
            htd::vertex_t vertex1 = indices.at((*it)[0]);
            htd::vertex_t vertex2 = indices.at((*it)[1]);

            outputStream << "    <edge source=\"n" << vertex1 << "\""
                                    " target=\"n" << vertex2 << "\"/>" << "\n";

            ++it;
        }
    }

    outputStream << "  </graph>" << std::endl
        << "</graphml>" << std::endl;}

#endif /* HTD_IO_GRAPHMLFORMATEXPORTER_CPP */
