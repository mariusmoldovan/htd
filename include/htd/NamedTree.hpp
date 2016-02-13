/*
 * File:   NamedTree.hpp
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

#ifndef HTD_HTD_NAMEDTREE_HPP
#define	HTD_HTD_NAMEDTREE_HPP

#include <htd/Globals.hpp>
#include <htd/Helpers.hpp>
#include <htd/LabeledTreeFactory.hpp>
#include <htd/BidirectionalGraphNaming.hpp>
#include <htd/Label.hpp>
#include <htd/VectorAdapter.hpp>
#include <htd/NamedVertexHyperedge.hpp>
#include <htd/PostOrderTreeTraversal.hpp>

#include <vector>
#include <stdexcept>

namespace htd
{
    template<typename VertexNameType, typename EdgeNameType>
    class NamedTree
    {
        public:
            NamedTree(void) : base_(htd::LabeledTreeFactory::instance().getLabeledTree()), names_()
            {

            }

            NamedTree(const NamedTree<VertexNameType, EdgeNameType> & original) : base_(original.base_->clone()), names_(original.names_)
            {

            }

            ~NamedTree()
            {
                if (base_ != nullptr)
                {
                    delete base_;

                    base_ = nullptr;
                }
            }

            std::size_t vertexCount(void) const
            {
                return base_->vertexCount();
            }

            std::size_t edgeCount(void) const
            {
                return base_->edgeCount();
            }

            bool isVertexName(const VertexNameType & vertexName) const
            {
                return names_.isVertexName(vertexName);
            }

            void setVertexName(htd::vertex_t vertex, const VertexNameType & vertexName)
            {
                if (!base_->isVertex(vertex))
                {
                    throw std::logic_error("void htd::NamedTree<VertexNameType, EdgeNameType>::setVertexName(htd::vertex_t, const VertexNameType &)");
                }

                names_.setVertexName(vertex, vertexName);
            }

            bool isEdgeName(const EdgeNameType & edgeName) const
            {
                return names_.isEdgeName(edgeName);
            }

            void setEdgeName(htd::id_t edgeId, const EdgeNameType & edgeName)
            {
                if (!base_->isEdge(edgeId))
                {
                    throw std::logic_error("void htd::NamedTree<VertexNameType, EdgeNameType>::setEdgeName(htd::id_t, const EdgeNameType &)");
                }

                names_.setEdgeName(edgeId, edgeName);
            }

            const VertexNameType & vertexName(htd::vertex_t vertex) const
            {
                if (!names_.isNamedVertex(vertex))
                {
                    throw std::logic_error("const VertexNameType & htd::NamedTree<VertexNameType, EdgeNameType>::vertexName(htd::vertex_t) const");
                }

                return names_.vertexName(vertex);
            }

            const EdgeNameType & edgeName(htd::id_t edgeId) const
            {
                if (!names_.isNamedEdge(edgeId))
                {
                    throw std::logic_error("const EdgeNameType & htd::NamedTree<VertexNameType, EdgeNameType>::edgeName(htd::id_t) const");
                }

                return names_.edgeName(edgeId);
            }

            htd::vertex_t lookupVertex(const VertexNameType & vertexName) const
            {
                if (!names_.isVertexName(vertexName))
                {
                    throw std::logic_error("htd::vertex_t htd::NamedTree<VertexNameType, EdgeNameType>::lookupVertex(const VertexNameType &) const");
                }

                return names_.lookupVertex(vertexName);
            }

            htd::NamedVertexHyperedge<VertexNameType> lookupHyperedge(const EdgeNameType & edgeName) const
            {
                if (!names_.isEdgeName(edgeName))
                {
                    throw std::logic_error("htd::NamedVertexHyperedge<VertexNameType> htd::NamedTree<VertexNameType, EdgeNameType>::lookupHyperedge(const EdgeNameType &) const");
                }

                htd::id_t edgeId = names_.lookupEdge(edgeName);

                NamedVertexHyperedge<VertexNameType> ret(edgeId);

                for (htd::vertex_t vertex : base_->hyperedge(edgeId))
                {
                    ret.push_back(vertexName(vertex));
                }

                return ret;
            }

            std::size_t edgeCount(const VertexNameType & vertexName) const
            {
                return base_->edgeCount(lookupVertex(vertexName));
            }

            bool isEdge(const std::vector<VertexNameType> & elements)
            {
                return isEdge(htd::ConstCollection<VertexNameType>::getInstance(elements));
            }

            bool isEdge(const htd::ConstCollection<VertexNameType> & elements)
            {
                bool ok = true;

                for (auto it = elements.begin(); ok && it != elements.end(); it++)
                {
                    if (!isVertex(*it))
                    {
                        ok = false;
                    }
                }

                if (ok)
                {
                    ok = base_->isEdge(elements);
                }

                return ok;
            }

            htd::id_t associatedEdgeId(const EdgeNameType & edgeName) const
            {
                if (!names_.isEdgeName(edgeName))
                {
                    throw std::logic_error("htd::id_t htd::NamedTree<VertexNameType, EdgeNameType>::associatedEdgeId(const EdgeNameType &) const");
                }

                return names_.lookupEdge(edgeName);
            }

            htd::ConstCollection<htd::id_t> associatedEdgeIds(const VertexNameType & vertexName1, const VertexNameType & vertexName2) const
            {
                if (isVertexName(vertexName1) && isVertexName(vertexName2))
                {
                    return base_->associatedEdgeIds(lookupVertex(vertexName1), lookupVertex(vertexName2));
                }

                return htd::ConstCollection<htd::id_t>::getInstance(htd::VectorAdapter<htd::id_t>());
            }

            htd::ConstCollection<htd::id_t> associatedEdgeIds(std::pair<VertexNameType, VertexNameType> vertexNames) const
            {
                return associatedEdgeIds(vertexNames.first, vertexNames.second);
            }

            htd::ConstCollection<VertexNameType> vertices(void) const
            {
                htd::VectorAdapter<VertexNameType> ret;

                std::vector<VertexNameType> & container = ret.container();

                for (htd::vertex_t vertex : base_->vertices())
                {
                    container.push_back(vertexName(vertex));
                }

                return htd::ConstCollection<VertexNameType>::getInstance(ret);
            }

            bool isConnected(const VertexNameType & vertexName1, const VertexNameType & vertexName2) const
            {
                return base_->isConnected(lookupVertex(vertexName1), lookupVertex(vertexName2));
            }

            bool isNeighbor(const VertexNameType & vertexName1, const VertexNameType & vertexName2) const
            {
                return base_->isNeighbor(lookupVertex(vertexName1), lookupVertex(vertexName2));
            }
            
            std::size_t neighborCount(const VertexNameType & vertexName) const
            {
                return base_->neighborCount(lookupVertex(vertexName));
            }

            htd::ConstCollection<VertexNameType> neighbors(const VertexNameType & vertexName) const
            {
                htd::VectorAdapter<VertexNameType> ret;

                std::vector<VertexNameType> & container = ret.container();

                for (htd::vertex_t neighbor : base_->neighbors(lookupVertex(vertexName)))
                {
                    container.push_back(this->vertexName(neighbor));
                }

                return htd::ConstCollection<VertexNameType>::getInstance(ret);
            }

            const VertexNameType & neighborAtPosition(const VertexNameType & vertexName, htd::index_t index) const
            {
                return vertexName(base_->neighborAtPosition(lookupVertex(vertexName), index));
            }

            std::size_t isolatedVertexCount(void) const
            {
                return base_->isolatedVertexCount();
            }

            htd::ConstCollection<VertexNameType> isolatedVertices(void) const
            {
                htd::VectorAdapter<VertexNameType> ret;

                std::vector<VertexNameType> & container = ret.container();

                for (htd::vertex_t isolatedVertex : base_->isolatedVertices())
                {
                    container.push_back(vertexName(isolatedVertex));
                }

                return htd::ConstCollection<VertexNameType>::getInstance(ret);
            }

            const VertexNameType & isolatedVertexAtPosition(htd::index_t index) const
            {
                return vertexName(base_->isolatedVertexAtPosition(index));
            }

            bool isIsolatedVertex(const VertexNameType & vertexName) const
            {
                return base_->isIsolatedVertex(lookupVertex(vertexName));
            }

            htd::ConstCollection<NamedVertexHyperedge<VertexNameType>> hyperedges(void) const
            {
                htd::VectorAdapter<NamedVertexHyperedge<VertexNameType>> ret;

                std::vector<NamedVertexHyperedge<VertexNameType>> & container = ret.container();

                for (const htd::Hyperedge & hyperedge : base_->hyperedges())
                {
                    container.push_back(NamedVertexHyperedge<VertexNameType>(hyperedge.id()));

                    NamedVertexHyperedge<VertexNameType> & newHyperedge = container[container.size() - 1];

                    for (htd::vertex_t vertex : hyperedge)
                    {
                        newHyperedge.push_back(vertexName(vertex));
                    }
                }

                return htd::ConstCollection<NamedVertexHyperedge<VertexNameType>>::getInstance(ret);
            }

            htd::ConstCollection<NamedVertexHyperedge<VertexNameType>> hyperedges(const VertexNameType & vertexName) const
            {
                htd::VectorAdapter<NamedVertexHyperedge<VertexNameType>> ret;

                std::vector<NamedVertexHyperedge<VertexNameType>> & container = ret.container();

                for (const htd::Hyperedge & hyperedge : base_->hyperedges(lookupVertex(vertexName)))
                {
                    container.push_back(NamedVertexHyperedge<VertexNameType>(hyperedge.id()));

                    NamedVertexHyperedge<VertexNameType> & newHyperedge = container[container.size() - 1];

                    for (htd::vertex_t vertex : hyperedge)
                    {
                        newHyperedge.push_back(vertexName(vertex));
                    }
                }

                return htd::ConstCollection<NamedVertexHyperedge<VertexNameType>>::getInstance(ret);
            }

            NamedVertexHyperedge<VertexNameType> hyperedge(htd::id_t edgeId) const
            {
                NamedVertexHyperedge<VertexNameType> ret(edgeId);

                for (htd::vertex_t vertex : base_->hyperedge(edgeId))
                {
                    ret.push_back(vertexName(vertex));
                }

                return ret;
            }

            NamedVertexHyperedge<VertexNameType> hyperedgeAtPosition(htd::index_t index, const VertexNameType & vertexName) const
            {
                const htd::Hyperedge & hyperedge = base_->hyperedgeAtPosition(index, lookupVertex(vertexName));

                NamedVertexHyperedge<VertexNameType> ret(hyperedge.id());

                for (htd::vertex_t vertex : hyperedge)
                {
                    ret.push_back(vertexName(vertex));
                }

                return ret;
            }

            void removeVertex(const VertexNameType & vertexName)
            {
                if (isVertexName(vertexName))
                {
                    htd::vertex_t locatedVertex = lookupVertex(vertexName);

                    names_.removeVertexName(locatedVertex);

                    base_->removeVertex(locatedVertex);
                }
            }

            void removeSubtree(const VertexNameType & vertexName)
            {
                if (isVertexName(vertexName))
                {
                    htd::vertex_t locatedVertex = lookupVertex(vertexName);

                    htd::PostOrderTreeTraversal treeTraversal;

                    treeTraversal.traverse(*this, [&](htd::vertex_t vertex, htd::vertex_t parent, std::size_t distanceToSubtreeRoot)
                    {
                        HTD_UNUSED(parent)
                        HTD_UNUSED(distanceToSubtreeRoot)

                        names_.removeVertexName(vertex);
                    }, locatedVertex);

                    base_->removeSubtree(locatedVertex);
                }
            }

            htd::vertex_t insertRoot(const VertexNameType & vertexName)
            {
                if (base_->vertexCount() > 0)
                {
                    names_.clear();

                    base_->removeSubtree(base_->root());
                }

                htd::vertex_t ret = base_->insertRoot();

                names_.setVertexName(ret, vertexName);

                return ret;
            }

            void removeRoot(void)
            {
                if (base_->vertexCount() > 0)
                {
                    names_.clear();

                    base_->removeRoot();
                }
            }

            htd::vertex_t addChild(const VertexNameType & vertexName, const VertexNameType & childName)
            {
                if (!isVertexName(vertexName) || isVertexName(childName))
                {
                    throw std::logic_error("htd::vertex_t addChild(const VertexNameType &, const VertexNameType &)");
                }

                htd::vertex_t ret = base_->addChild(lookupVertex(vertexName));

                names_.setVertexName(ret, childName);

                return ret;
            }

            void removeChild(const VertexNameType & vertexName, const VertexNameType & childName)
            {
                if (!isVertexName(vertexName) || !isVertexName(childName))
                {
                    throw std::logic_error("void removeChild(const VertexNameType &, const VertexNameType &)");
                }

                htd::vertex_t child = lookupVertex(childName);

                base_->removeChild(lookupVertex(vertexName), child);

                names_.removeVertexName(child);
            }

            htd::vertex_t addParent(const VertexNameType & vertexName, const VertexNameType & parentName)
            {
                if (!isVertexName(vertexName) || isVertexName(parentName))
                {
                    throw std::logic_error("htd::vertex_t addParent(const VertexNameType &, const VertexNameType &)");
                }

                htd::vertex_t ret = base_->addParent(lookupVertex(vertexName));

                names_.setVertexName(ret, parentName);

                return ret;
            }

            void setParent(const VertexNameType & vertexName, const VertexNameType & newParentName)
            {
                base_->setParent(lookupVertex(vertexName), lookupVertex(newParentName));
            }

            std::size_t labelCount(void) const
            {
                return base_->labelCount();
            }

            htd::ConstCollection<std::string> labelNames(void) const
            {
                return base_->labelNames();
            }

            const std::string & labelNameAtPosition(htd::index_t index) const
            {
                return base_->labelNameAtPosition(index);
            }

            const htd::ILabel & vertexLabel(const std::string & labelName, const VertexNameType & vertexName) const
            {
                return base_->vertexLabel(labelName, lookupVertex(vertexName));
            }

            const htd::ILabel & edgeLabel(const std::string & labelName, htd::id_t edgeId) const
            {
                return base_->edgeLabel(labelName, edgeId);
            }

            const htd::ILabel & edgeLabel(const std::string & labelName, const EdgeNameType & edgeName) const
            {
                return base_->edgeLabel(labelName, names_.lookupEdge(edgeName));
            }

            void setVertexLabel(const std::string & labelName, const VertexNameType & vertexName, htd::ILabel * label)
            {
                base_->setVertexLabel(labelName, lookupVertex(vertexName), label);
            }

            void setEdgeLabel(const std::string & labelName, htd::id_t edgeId, htd::ILabel * label)
            {
                base_->setEdgeLabel(labelName, edgeId, label);
            }

            void setEdgeLabel(const std::string & labelName, const EdgeNameType & edgeName, htd::ILabel * label)
            {
                base_->setEdgeLabel(labelName, names_.lookupEdge(edgeName), label);
            }

            void removeVertexLabel(const std::string & labelName, const VertexNameType & vertexName)
            {
                base_->removeVertexLabel(labelName, lookupVertex(vertexName));
            }

            void removeEdgeLabel(const std::string & labelName, htd::id_t edgeId)
            {
                base_->removeEdgeLabel(labelName, edgeId);
            }

            void removeEdgeLabel(const std::string & labelName, const EdgeNameType & edgeName)
            {
                base_->removeEdgeLabel(labelName, names_.lookupEdge(edgeName));
            }

            void swapVertexLabels(const VertexNameType & vertexName1, const VertexNameType & vertexName2)
            {
                base_->swapVertexLabels(lookupVertex(vertexName1), lookupVertex(vertexName2));
            }

            void swapEdgeLabels(htd::id_t edgeId1, htd::id_t edgeId2)
            {
                base_->swapEdgeLabels(edgeId1, edgeId2);
            }

            void swapEdgeLabels(const EdgeNameType & edgeName1, const EdgeNameType & edgeName2)
            {
                base_->swapEdgeLabels(names_.lookupEdge(edgeName1), names_.lookupEdge(edgeName2));
            }

            void swapVertexLabel(const std::string & labelName, const VertexNameType & vertexName1, const VertexNameType & vertexName2)
            {
                base_->swapVertexLabel(labelName, lookupVertex(vertexName1), lookupVertex(vertexName2));
            }

            void swapEdgeLabel(const std::string & labelName, htd::id_t edgeId1, htd::id_t edgeId2)
            {
                base_->swapEdgeLabel(labelName, edgeId1, edgeId2);
            }

            void swapEdgeLabel(const std::string & labelName, const EdgeNameType & edgeName1, const EdgeNameType & edgeName2)
            {
                base_->swapEdgeLabel(labelName, names_.lookupEdge(edgeName1), names_.lookupEdge(edgeName2));
            }

            NamedTree<VertexNameType, EdgeNameType> * clone(void) const
            {
                return new NamedTree<VertexNameType, EdgeNameType>(*this);
            }

            const htd::ILabeledTree & internalGraph(void) const
            {
                return *base_;
            }

        private:
            htd::IMutableLabeledTree * base_;

            htd::BidirectionalGraphNaming<VertexNameType, EdgeNameType> names_;
    };
}

#endif /* HTD_HTD_NAMEDTREE_HPP */
