// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2020
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt
// https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Version: 4.0.2019.08.13

#pragma once

#include <algorithm>
#include <array>

namespace gte
{
    template <int N, bool Ordered>
    class FeatureKey
    {
    protected:
        // Abstract base class with V[] uninitialized.  The derived classes
        // must set the V[] values accordingly.
        //
        // An ordered feature key has V[0] = min(V[]) with
        // (V[0],V[1],...,V[N-1]) a permutation of N inputs with an even
        // number of transpositions.
        //
        // An unordered feature key has V[0] < V[1] < ... < V[N-1].
        //
        // Note that the word 'order' is about the geometry of the feature, not
        // the comparison order for any sorting.
        FeatureKey() = default;

    public:
        bool operator==(FeatureKey const& key) const
        {
            return V == key.V;
        }

        bool operator!=(FeatureKey const& key) const
        {
            return V != key.V;
        }

        bool operator<(FeatureKey const& key) const
        {
            return V < key.V;
        }

        bool operator<=(FeatureKey const& key) const
        {
            return V <= key.V;
        }

        bool operator>(FeatureKey const& key) const
        {
            return V > key.V;
        }

        bool operator>=(FeatureKey const& key) const
        {
            return V >= key.V;
        }

        std::array<int, N> V;
    };
}
