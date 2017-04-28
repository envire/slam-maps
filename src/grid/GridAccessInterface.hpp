//
// Copyright (c) 2015-2017, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// Copyright (c) 2015-2017, University of Bremen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#pragma once

#include "AccessIterator.hpp"
#include "Index.hpp"

namespace maps { namespace grid
{
    template <typename CellBaseT>
    class GridAccessInterface
    {
    public:

        typedef AccessIterator<CellBaseT> iterator;
        typedef ConstAccessIterator<CellBaseT> const_iterator;

        GridAccessInterface() {}

        virtual ~GridAccessInterface() {}

        virtual const CellBaseT &getDefaultValue() const = 0;

        virtual iterator begin() = 0;

        virtual iterator end() = 0;

        virtual const_iterator begin() const = 0;

        virtual const_iterator end() const = 0;

        virtual void resize(Vector2ui new_number_cells) = 0;

        virtual void moveBy(Index idx) = 0;

        virtual const CellBaseT& at(Index idx) const = 0;

        virtual CellBaseT& at(Index idx) = 0;

        virtual const Vector2ui &getNumCells() const = 0;

        virtual void clear() = 0;
    };
}}
