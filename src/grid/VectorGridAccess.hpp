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
    template <typename CellT, typename CellBaseT>
    class VectorGridAccess : public GridAccessInterface<CellBaseT>
    {
        VectorGrid<CellT> *grid;
    public:
        VectorGridAccess(VectorGrid<CellT> *grid) : grid(grid)
        {
        }

        virtual ~VectorGridAccess()
        {
        }

        virtual const CellBaseT &getDefaultValue() const
        {
            return grid->getDefaultValue();
        }

        virtual typename GridAccessInterface<CellBaseT>::iterator begin()
        {
            return AccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->begin());
        }

        virtual typename GridAccessInterface<CellBaseT>::iterator end()
        {
            return AccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->end());
        }

        virtual typename GridAccessInterface<CellBaseT>::const_iterator begin() const
        {
            return ConstAccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->begin());
        }

        virtual typename GridAccessInterface<CellBaseT>::const_iterator end() const
        {
            return ConstAccessIteratorImpl<CellT, CellBaseT, std::vector<CellT> >(grid->end());
        }

        virtual void resize(Vector2ui new_number_cells)
        {
            grid->resize(new_number_cells);
        };

        virtual void moveBy(Index idx)
        {
            grid->moveBy(idx);
        }

        virtual const CellBaseT& at(Index idx) const
        {
            return static_cast<const CellBaseT &>(grid->at(idx));
        }

        virtual CellBaseT& at(Index idx)
        {
            return static_cast<CellBaseT &>(grid->at(idx));
        }

        virtual const CellBaseT& at(size_t x, size_t y) const
        {
            return static_cast<const CellBaseT &>(grid->at(x, y));
        }

        virtual CellBaseT& at(size_t x, size_t y)
        {
            return static_cast<CellBaseT &>(grid->at(x, y));
        }

        virtual const Vector2ui &getNumCells() const
        {
            return grid->getNumCells();
        };

        virtual void clear()
        {
            grid->clear();
        };
    };
}}
