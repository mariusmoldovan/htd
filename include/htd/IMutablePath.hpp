/* 
 * File:   IMutablePath.hpp
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

#ifndef HTD_HTD_IMUTABLEPATH_HPP
#define	HTD_HTD_IMUTABLEPATH_HPP

#include <htd/Globals.hpp>
#include <htd/IPath.hpp>

namespace htd
{
    class IMutablePath : public virtual htd::IPath
    {
        public:
            /**
             *  Destructor for an IMutablePath object.
             */
            virtual ~IMutablePath() = 0;

            /**
             *  Insert the root of the path if it does not already exist.
             *
             *  @return The ID of the root of the path.
             */
            virtual htd::vertex_t insertRoot(void) = 0;

            /**
             *  Remove a vertex from the path.
             *
             *  @param[in] vertex   The ID of the vertex which shall be removed.
             */
            virtual void removeVertex(htd::vertex_t vertex) = 0;

            /**
             *  Remove a subpath from the path.
             *
             *  @param[in] subpathRoot  The root of the subpath.
             */
            virtual void removeSubpath(htd::vertex_t subpathRoot) = 0;

            /**
             *  Remove the child of a vertex from the path.
             *
             *  @param[in] vertex   The ID of the vertex which's child shall be removed.
             *
             *  @note This operation retains the path structure by connecting the neighbors of the removed vertex in a valid way.
             */
            virtual void removeChild(htd::vertex_t vertex) = 0;

            /**
             *  Add a new child to the given vertex.
             *
             *  @param[in] vertex   The vertex which shall get a new child.
             *
             *  @return The ID of the new child.
             *
             *  @note If the given vertex already has a child, the old child will be attached to the new vertex in order to keep the path structure valid.
             */
            virtual htd::vertex_t addChild(htd::vertex_t vertex) = 0;

            /**
             *  Remove the child of a vertex from the path.
             *
             *  @param[in] vertex   The ID of the vertex which's child shall be removed.
             *  @param[in] child    The ID of the vertex which shall be removed.
             *
             *  @note This operation retains the path structure by connecting the neighbors of the removed vertex in a valid way.
             */
            virtual void removeChild(htd::vertex_t vertex, htd::vertex_t child) = 0;

            /**
             *  Add a new parent to the given vertex.
             *
             *  @param[in] vertex   The vertex which shall get a new parent.
             *
             *  @return The ID of the new parent.
             *
             *  @note If the given vertex already has a parent, the old parent will be attached to the new vertex in order to keep the path structure valid.
             */
            virtual htd::vertex_t addParent(htd::vertex_t vertex) = 0;

            /**
             *  Create a deep copy the current mutable path.
             *
             *  @return A new IMutablePath object identical to the current mutable path.
             */
            virtual IMutablePath * clone(void) const = 0;

            /**
             *  Copy assignment operator for a mutable path.
             *
             *  @param[in] original  The original path.
             */
            virtual IMutablePath & operator=(const htd::IPath & original) = 0;
    };

    inline htd::IMutablePath::~IMutablePath() { }
}

#endif /* HTD_HTD_IMUTABLEPATH_HPP */
