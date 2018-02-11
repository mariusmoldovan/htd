/*
 * File:   SeparatorBasedTreeDecompositionAlgorithm2.cpp
 *
 * Author: ABSEHER Michael (abseher@dbai.tuwien.ac.at), MOLDOVAN Marius (moldovan@dbai.tuwien.ac.at)
 *
 * Copyright 2015-2017, Michael Abseher, Marius Moldovan
 *    E-Mail: <abseher@dbai.tuwien.ac.at>, <moldovan@dbai.tuwien.ac.at>
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

#ifndef HTD_HTD_SEPARATORBASEDTREEDECOMPOSITIONALGORITHM2_CPP
#define HTD_HTD_SEPARATORBASEDTREEDECOMPOSITIONALGORITHM2_CPP

#include <htd/Globals.hpp>
#include <htd/Helpers.hpp>

#include <htd/SeparatorBasedTreeDecompositionAlgorithm2.hpp>
#include <htd/ConnectedComponentAlgorithmFactory.hpp>
#include <htd/TreeDecompositionFactory.hpp>
#include <htd/GraphLabeling.hpp>
#include <htd/ILabelingFunction.hpp>
#include <htd/IMutableTreeDecomposition.hpp>
#include <htd/GraphPreprocessorFactory.hpp>
#include <htd/IGraphPreprocessor.hpp>
#include <htd/GraphSeparatorAlgorithmFactory.hpp>
#include <htd/ITreeDecompositionManipulationOperation.hpp>
#include <htd/TrivialTreeDecompositionAlgorithm.hpp>
#include <htd/WidthReductionOperation2.hpp>

#include <algorithm>
#include <cstdarg>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 *  Private implementation details of class htd::SeparatorBasedTreeDecompositionAlgorithm2.
 */
struct htd::SeparatorBasedTreeDecompositionAlgorithm2::Implementation
{
    /**
     *  Constructor for the implementation details structure.
     *
     *  @param[in] manager  The management instance to which the current object instance belongs.
     */
    Implementation(const htd::LibraryInstance * const manager, long maxWidth) : managementInstance_(manager), maxWidth_(maxWidth), separatorAlgorithm_(manager->graphSeparatorAlgorithmFactory().createInstance()), labelingFunctions_(), postProcessingOperations_(), computeInducedEdges_(true)
    {

    }

    /**
     *  Copy constructor for the implementation details structure.
     *
     *  @param[in] original The original implementation details structure.
     */
    Implementation(const Implementation & original) : managementInstance_(original.managementInstance_), maxWidth_(original.maxWidth_), separatorAlgorithm_(original.separatorAlgorithm_->clone()), labelingFunctions_(), postProcessingOperations_(), computeInducedEdges_(original.computeInducedEdges_)
    {
        for (htd::ILabelingFunction * labelingFunction : original.labelingFunctions_)
        {
    #ifndef HTD_USE_VISUAL_STUDIO_COMPATIBILITY_MODE
            labelingFunctions_.push_back(labelingFunction->clone());
    #else
            labelingFunctions_.push_back(labelingFunction->cloneLabelingFunction());
    #endif
        }

        for (htd::ITreeDecompositionManipulationOperation * postProcessingOperation : original.postProcessingOperations_)
        {
    #ifndef HTD_USE_VISUAL_STUDIO_COMPATIBILITY_MODE
            postProcessingOperations_.push_back(postProcessingOperation->clone());
    #else
            postProcessingOperations_.push_back(postProcessingOperation->cloneTreeDecompositionManipulationOperation());
    #endif
        }
    }

    virtual ~Implementation()
    {
        delete separatorAlgorithm_;

        for (auto & labelingFunction : labelingFunctions_)
        {
            delete labelingFunction;
        }

        for (auto & postProcessingOperation : postProcessingOperations_)
        {
            delete postProcessingOperation;
        }
    }

    /**
     *  The management instance to which the current object instance belongs.
     */
    const htd::LibraryInstance * managementInstance_;

    /**
     *  Maximum bag width fow which a separator should be seeked.
     */
    long maxWidth_;

    /**
     *  The underlying graph separator algorithm.
     */
    htd::IGraphSeparatorAlgorithm * separatorAlgorithm_;

    /**
     *  The labeling functions which are applied after the decomposition was computed.
     */
    std::vector<htd::ILabelingFunction *> labelingFunctions_;

    /**
     *  The manipuation operations which are applied after the decomposition was computed.
     */
    std::vector<htd::ITreeDecompositionManipulationOperation *> postProcessingOperations_;

    /**
     *  A boolean flag indicating whether the hyperedges induced by a respective bag shall be computed.
     */
    bool computeInducedEdges_;

    /**
     *  Compute a new mutable tree decompostion of the given graph.
     *
     *  @param[in] graph                The graph which shall be decomposed.
     *  @param[in] preprocessedGraph    The input graph in preprocessed format.
     *
     *  @return A mutable tree decompostion of the given graph.
     */
    htd::IMutableTreeDecomposition * computeMutableDecomposition(const htd::IMultiHypergraph & graph, const htd::IPreprocessedGraph & preprocessedGraph) const;
};

htd::SeparatorBasedTreeDecompositionAlgorithm2::SeparatorBasedTreeDecompositionAlgorithm2(const htd::LibraryInstance * const manager, long maxWidth) : implementation_(new Implementation(manager, maxWidth))
{

}

htd::SeparatorBasedTreeDecompositionAlgorithm2::SeparatorBasedTreeDecompositionAlgorithm2(const htd::LibraryInstance * const manager, const std::vector<htd::IDecompositionManipulationOperation *> & manipulationOperations, long maxWidth)  : implementation_(new Implementation(manager, maxWidth))
{
    setManipulationOperations(manipulationOperations);
}

htd::SeparatorBasedTreeDecompositionAlgorithm2::SeparatorBasedTreeDecompositionAlgorithm2(const htd::SeparatorBasedTreeDecompositionAlgorithm2 & original)  : implementation_(new Implementation(*(original.implementation_)))
{

}

htd::SeparatorBasedTreeDecompositionAlgorithm2::~SeparatorBasedTreeDecompositionAlgorithm2()
{

}

htd::ITreeDecomposition * htd::SeparatorBasedTreeDecompositionAlgorithm2::computeDecomposition(const htd::IMultiHypergraph & graph) const
{
    return computeDecomposition(graph, std::vector<htd::IDecompositionManipulationOperation *>());
}

htd::ITreeDecomposition * htd::SeparatorBasedTreeDecompositionAlgorithm2::computeDecomposition(const htd::IMultiHypergraph & graph, const std::vector<htd::IDecompositionManipulationOperation *> & manipulationOperations) const
{
    htd::IGraphPreprocessor * preprocessor = implementation_->managementInstance_->graphPreprocessorFactory().createInstance();

    htd::IPreprocessedGraph * preprocessedGraph = preprocessor->prepare(graph);

    htd::ITreeDecomposition * ret = computeDecomposition(graph, *preprocessedGraph, manipulationOperations);

    delete preprocessedGraph;
    delete preprocessor;

    return ret;
}

htd::ITreeDecomposition * htd::SeparatorBasedTreeDecompositionAlgorithm2::computeDecomposition(const htd::IMultiHypergraph & graph, const htd::IPreprocessedGraph & preprocessedGraph) const
{
    return computeDecomposition(graph, preprocessedGraph, std::vector<htd::IDecompositionManipulationOperation *>());
}

htd::ITreeDecomposition * htd::SeparatorBasedTreeDecompositionAlgorithm2::computeDecomposition(const htd::IMultiHypergraph & graph, const htd::IPreprocessedGraph & preprocessedGraph, const std::vector<htd::IDecompositionManipulationOperation *> & manipulationOperations) const
{
    htd::IMutableTreeDecomposition * ret = implementation_->computeMutableDecomposition(graph, preprocessedGraph);

    if (ret != nullptr)
    {
        std::vector<htd::ILabelingFunction *> labelingFunctions;

        std::vector<htd::ITreeDecompositionManipulationOperation *> postProcessingOperations;

        for (htd::IDecompositionManipulationOperation * operation : manipulationOperations)
        {
            htd::ILabelingFunction * labelingFunction = dynamic_cast<htd::ILabelingFunction *>(operation);

            if (labelingFunction != nullptr)
            {
                labelingFunctions.push_back(labelingFunction);
            }

            htd::ITreeDecompositionManipulationOperation * manipulationOperation = dynamic_cast<htd::ITreeDecompositionManipulationOperation *>(operation);

            if (manipulationOperation != nullptr)
            {
                postProcessingOperations.push_back(manipulationOperation);
            }
        }

        for (const htd::ITreeDecompositionManipulationOperation * operation : implementation_->postProcessingOperations_)
        {
            operation->apply(graph, *ret);
        }

        for (htd::ITreeDecompositionManipulationOperation * operation : postProcessingOperations)
        {
            operation->apply(graph, *ret);
        }

        for (const htd::ILabelingFunction * labelingFunction : implementation_->labelingFunctions_)
        {
            for (htd::vertex_t vertex : ret->vertices())
            {
                htd::ILabelCollection * labelCollection = ret->labelings().exportVertexLabelCollection(vertex);

                htd::ILabel * newLabel = labelingFunction->computeLabel(ret->bagContent(vertex), *labelCollection);

                delete labelCollection;

                ret->setVertexLabel(labelingFunction->name(), vertex, newLabel);
            }
        }

        for (htd::ILabelingFunction * labelingFunction : labelingFunctions)
        {
            for (htd::vertex_t vertex : ret->vertices())
            {
                htd::ILabelCollection * labelCollection = ret->labelings().exportVertexLabelCollection(vertex);

                htd::ILabel * newLabel = labelingFunction->computeLabel(ret->bagContent(vertex), *labelCollection);

                delete labelCollection;

                ret->setVertexLabel(labelingFunction->name(), vertex, newLabel);
            }
        }
    }

    for (htd::IDecompositionManipulationOperation * operation : manipulationOperations)
    {
        delete operation;
    }

    return ret;
}

htd::ITreeDecomposition * htd::SeparatorBasedTreeDecompositionAlgorithm2::computeDecomposition(const htd::IMultiHypergraph & graph, int manipulationOperationCount, ...) const
{
    va_list arguments;

    va_start(arguments, manipulationOperationCount);

    std::vector<htd::IDecompositionManipulationOperation *> manipulationOperations;

    for (int manipulationOperationIndex = 0; manipulationOperationIndex < manipulationOperationCount; manipulationOperationIndex++)
    {
        manipulationOperations.push_back(va_arg(arguments, htd::IDecompositionManipulationOperation *));
    }

    va_end(arguments);

    return computeDecomposition(graph, manipulationOperations);
}

htd::ITreeDecomposition * htd::SeparatorBasedTreeDecompositionAlgorithm2::computeDecomposition(const htd::IMultiHypergraph & graph, const htd::IPreprocessedGraph & preprocessedGraph, int manipulationOperationCount, ...) const
{
    va_list arguments;

    va_start(arguments, manipulationOperationCount);

    std::vector<htd::IDecompositionManipulationOperation *> manipulationOperations;

    for (int manipulationOperationIndex = 0; manipulationOperationIndex < manipulationOperationCount; manipulationOperationIndex++)
    {
        manipulationOperations.push_back(va_arg(arguments, htd::IDecompositionManipulationOperation *));
    }

    va_end(arguments);

    return computeDecomposition(graph, preprocessedGraph, manipulationOperations);
}

void htd::SeparatorBasedTreeDecompositionAlgorithm2::setGraphSeparatorAlgorithm(htd::IGraphSeparatorAlgorithm * algorithm)
{
    HTD_ASSERT(algorithm != nullptr)

    delete implementation_->separatorAlgorithm_;

    implementation_->separatorAlgorithm_ = algorithm;
}

void htd::SeparatorBasedTreeDecompositionAlgorithm2::setManipulationOperations(const std::vector<htd::IDecompositionManipulationOperation *> & manipulationOperations)
{
    for (auto & labelingFunction : implementation_->labelingFunctions_)
    {
        delete labelingFunction;
    }

    for (auto & postProcessingOperation : implementation_->postProcessingOperations_)
    {
        delete postProcessingOperation;
    }

    implementation_->labelingFunctions_.clear();

    implementation_->postProcessingOperations_.clear();

    addManipulationOperations(manipulationOperations);
}

void htd::SeparatorBasedTreeDecompositionAlgorithm2::addManipulationOperation(htd::IDecompositionManipulationOperation * manipulationOperation)
{
    bool assigned = false;

    htd::ILabelingFunction * labelingFunction = dynamic_cast<htd::ILabelingFunction *>(manipulationOperation);

    if (labelingFunction != nullptr)
    {
        implementation_->labelingFunctions_.emplace_back(labelingFunction);

        assigned = true;
    }

    htd::ITreeDecompositionManipulationOperation * newManipulationOperation = dynamic_cast<htd::ITreeDecompositionManipulationOperation *>(manipulationOperation);

    if (newManipulationOperation != nullptr)
    {
        implementation_->postProcessingOperations_.emplace_back(newManipulationOperation);

        assigned = true;
    }

    if (!assigned)
    {
        delete manipulationOperation;
    }
}

void htd::SeparatorBasedTreeDecompositionAlgorithm2::addManipulationOperations(const std::vector<htd::IDecompositionManipulationOperation *> & manipulationOperations)
{
    for (htd::IDecompositionManipulationOperation * operation : manipulationOperations)
    {
        addManipulationOperation(operation);
    }
}

bool htd::SeparatorBasedTreeDecompositionAlgorithm2::isSafelyInterruptible(void) const
{
    return false;
}

const htd::LibraryInstance * htd::SeparatorBasedTreeDecompositionAlgorithm2::managementInstance(void) const HTD_NOEXCEPT
{
    return implementation_->managementInstance_;
}

void htd::SeparatorBasedTreeDecompositionAlgorithm2::setManagementInstance(const htd::LibraryInstance * const manager)
{
    HTD_ASSERT(manager != nullptr)

    implementation_->managementInstance_ = manager;
}

bool htd::SeparatorBasedTreeDecompositionAlgorithm2::isComputeInducedEdgesEnabled(void) const
{
    return implementation_->computeInducedEdges_;
}

void htd::SeparatorBasedTreeDecompositionAlgorithm2::setComputeInducedEdgesEnabled(bool computeInducedEdgesEnabled)
{
    implementation_->computeInducedEdges_ = computeInducedEdgesEnabled;
}

htd::SeparatorBasedTreeDecompositionAlgorithm2 * htd::SeparatorBasedTreeDecompositionAlgorithm2::clone(void) const
{
    return new htd::SeparatorBasedTreeDecompositionAlgorithm2(*this);
}

htd::IMutableTreeDecomposition * htd::SeparatorBasedTreeDecompositionAlgorithm2::Implementation::computeMutableDecomposition(const htd::IMultiHypergraph & graph, const htd::IPreprocessedGraph & preprocessedGraph) const
{
    TrivialTreeDecompositionAlgorithm baseAlgorithm(managementInstance_);

    htd::IMutableTreeDecomposition * ret = dynamic_cast<htd::IMutableTreeDecomposition *>(baseAlgorithm.computeDecomposition(graph, preprocessedGraph));

    if (!managementInstance_->isTerminated())
    {
        HTD_ASSERT(ret != nullptr)

        WidthReductionOperation2 operation(managementInstance_, maxWidth_);

        operation.setGraphSeparatorAlgorithm(separatorAlgorithm_->clone());

        htd::ITreeDecomposition & decomposition = *ret;

        operation.apply(graph, managementInstance_->treeDecompositionFactory().accessMutableInstance(decomposition));
    }
    else
    {
        delete ret;

        ret = nullptr;
    }

    return ret;
}

#endif /* HTD_HTD_SEPARATORBASEDTREEDECOMPOSITIONALGORITHM2_CPP */
