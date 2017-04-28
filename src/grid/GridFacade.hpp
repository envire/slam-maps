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

#include "GridAccessInterface.hpp"

namespace maps { namespace grid
{
    template <typename CellBaseT>
    class GridFacade
    {
        GridAccessInterface<CellBaseT> *impl;
    public:
        
        typedef AccessIterator<CellBaseT> iterator;
        typedef ConstAccessIterator<CellBaseT> const_iterator;
        
        GridFacade(GridAccessInterface<CellBaseT> *impl) : impl(impl)
        {
        }

        const CellBaseT &getDefaultValue() const
        {
            return impl->getDefaultValue();
        }
        
        iterator begin()
        {
            return impl->begin();
        }

        iterator end()
        {
            return impl->end();
        }

        const_iterator begin() const
        {
            return impl->begin();
        }

        const_iterator end() const
        {
            return impl->end();
        }

        void resize(const Vector2ui &new_number_cells)
        {
            impl->resize(new_number_cells);
        };
        
        void moveBy(const Index &idx)
        {
            impl->moveBy(idx);
        }

        const CellBaseT& at(const Index &idx) const
        {
            return impl->at(idx);
        }

        CellBaseT& at(const Index &idx)
        {
            return impl->at(idx);
        }

        const CellBaseT& at(size_t x, size_t y) const
        {
            return impl->at(Index(x,y));
        }

        CellBaseT& at(size_t x, size_t y)
        {
            return impl->at(Index(x,y));
        }
        
        const Index &getNumCells() const
        {
            return impl->getNumCells();
        };
        
        void clear()
        {
            impl->clear();
        };
    };
    
}}
