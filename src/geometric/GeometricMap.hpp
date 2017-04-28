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
#ifndef __MAPS_GEOMETRICMAP_HPP_
#define __MAPS_GEOMETRICMAP_HPP_

#pragma once

/** Maps classes **/
#include <maps/LocalMap.hpp>

/** Std vector **/
#include <vector>
#include <unordered_map>
#include <typeindex>

/** Boost serialization **/
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>

namespace maps { namespace geometric
{
    /**@brief GeometricMap class IEEE 1873 standard
     * This map is a Geometric structure for a vector metric (Cartesian) map
     * This map store collections of geometric entities (e.g. points, lines)
     */
    template <typename T>
    class GeometricMap: public LocalMap
    {

    public:
        using ElementVector = std::vector<T>;

    private:
        /** Elements is the Geometric Map.
         * **/
        ElementVector elements;

    public:

        typedef typename ElementVector::iterator iterator;
        typedef typename ElementVector::const_iterator const_iterator;

        GeometricMap()
            : LocalMap(maps::LocalMapType::GEOMETRIC_MAP)
        {
        }

        GeometricMap(const GeometricMap& other)
            : LocalMap(other),
            elements (other.elements)
        {
        }

        /** @brief default destructor
         */
        ~GeometricMap()
        {
        }

    public:

        inline iterator begin() { return this->elements.begin(); }

        inline iterator end() { return this->elements.end(); }

        template <typename... Ts>
        void resize(Ts&&... args)
        {
            this->elements.resize(std::forward<Ts>(args)...);
        }

        size_t capacity()
        {
            return this->elements.capacity();
        }

        void push_back(const T &value)
        {
            return this->elements.push_back(value);
        }

        void pop_back()
        {
            return this->elements.pop_back();
        }

        template <typename... Ts>
        void insert(Ts&&... args)
        {
            this->elements.insert(std::forward<Ts>(args)...);
        }

        size_t getNumElements() const
        {
            return this->elements.size();
        }

        T& at(size_t n)
        {
            return this->elements.at(n);
        }

        const T& at(size_t n) const
        {
            return this->elements.at(n);
        }

        T& operator[](std::size_t idx)
        {
            return this->elements[idx];
        }

        const T& operator[](std::size_t idx) const
        {
            return this->elements[idx];
        }

        template <typename... Ts>
        iterator erase(Ts&&... args)
        {
            return this->elements.erase(std::forward<Ts>(args)...);
        }

    protected:
        /** Grants access to boost serialization */
        friend class boost::serialization::access;

        /** Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
             ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(::maps::LocalMap);
             ar & BOOST_SERIALIZATION_NVP(elements);
        }
    };
}}

#endif /* __MAPS_GEOMETRICMAP_HPP_ */
