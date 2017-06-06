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

#include <boost/container/flat_set.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/version.hpp>
#include <boost_serialization/DynamicSizeSerialization.hpp>

namespace maps { namespace grid
{
struct MLSConfig;

template <class _Tp>
struct myCmp : public std::binary_function<_Tp, _Tp, bool>
{
    bool
    operator()(const _Tp& __x, const _Tp& __y) const
    { return *__x < *__y; }
};
    
template <class S>
class LevelList : public boost::container::flat_set<S>
{
    typedef boost::container::flat_set<S> Base;
public:
    LevelList()
    {
    };
#if BOOST_VERSION < 105500
    // Custom copy constructor to work around boost bug:
    // https://svn.boost.org/trac/boost/ticket/9166
    // Note: This is fixed in version 1.55
    LevelList(const LevelList& other) {
        if(other.size() > 0)
            *this = other;
    }
#endif

    template<class S2>
    LevelList(const LevelList<S2>& other) : Base(other.begin(), other.end())
    { }

protected:
            /** Grants access to boost serialization */
    friend class boost::serialization::access;

    /** Serializes the members of this class*/
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    template<class Archive>
    void load(Archive &ar, const unsigned int version)
    {
        uint64_t count;
        loadSizeValue(ar, count);

        Base::clear();
        Base::reserve(count);
        for(size_t i=0; i<count; ++i)
        {
            S obj;
            ar >> obj;
            Base::insert(Base::end(), std::move(obj));
        }
    }
    template<class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        uint64_t size = (uint64_t)Base::size();
        saveSizeValue(ar, size);

        for(typename Base::const_iterator it = Base::begin(); it!= Base::end(); ++it)
        {
            ar << *it;
        }
    }
};

template <class S>
class LevelList<S *> : public boost::container::flat_set<S *, myCmp<S *>>
{
    typedef boost::container::flat_set<S *, myCmp<S *> > Base;
public:
    LevelList()
    {
    };
#if BOOST_VERSION < 105500
    // Custom copy constructor to work around boost bug:
    // https://svn.boost.org/trac/boost/ticket/9166
    // Note: This is fixed in version 1.55
    LevelList(const LevelList& other) {
        if(other.size() > 0)
            *this = other;
    }
#endif

    /** Serializes the members of this class*/
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    template<class Archive>
    void load(Archive &ar, const unsigned int version)
    {
        uint64_t count;
        loadSizeValue(ar, count);

        Base::clear();
        Base::reserve(count);
        for(size_t i=0; i<count; ++i)
        {
            S *obj;
            ar >> obj;
            Base::insert(Base::end(), obj);
        }
    }
    template<class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        uint64_t size = (uint64_t)Base::size();
        saveSizeValue(ar, size);

        for(typename Base::const_iterator it = Base::begin(); it!= Base::end(); ++it)
        {
            ar << *it;
        }
    }

    template<class S2>
    LevelList(const LevelList<S2>& other) : boost::container::flat_set<S *, myCmp<S *>>(other.begin(), other.end())
    { 
    }
    
    
};

template <class T>
class LevelListAccess
{
public:

    
    virtual ConstAccessIterator<T> begin() = 0;
    virtual ConstAccessIterator<T> end() = 0;
    
    LevelListAccess()
    {
    }
    virtual ~LevelListAccess() {}
    
};

template <class S, class SBase = S>
class DerivableLevelList : public DerivableLevelList<SBase, SBase>, public LevelList<S>
{
protected:
    virtual ConstAccessIterator<SBase> getBegin()
    {
        return ConstAccessIteratorImpl<S, SBase, LevelList<S> >(LevelList<S>::begin());
    };
    
    virtual ConstAccessIterator<SBase> getEnd()
    {
        return ConstAccessIteratorImpl<S, SBase, LevelList<S> >(LevelList<S>::end());
    };
    
    virtual size_t getSize() const
    {
        return LevelList<S>::size();
    };
public:
    using LevelList<S>::begin;
    using LevelList<S>::end;
    using LevelList<S>::find;
    using LevelList<S>::size;
    
    DerivableLevelList()
    {
    };

    virtual ~DerivableLevelList()
    {
    };
    
};

template <class S>
class DerivableLevelList<S, S>
{
protected:
    virtual ConstAccessIterator<S> getBegin() = 0;
    virtual ConstAccessIterator<S> getEnd() = 0;
    virtual size_t getSize() const = 0;
    
public:
    
    typedef ConstAccessIterator<S> iterator;
    
    DerivableLevelList()
    {
    };
    virtual ~DerivableLevelList() { }
    
    iterator begin()
    {
        return getBegin();
    };
    iterator end()
    {
        return getEnd();
    };

    size_t size() const
    {
        return getSize();
    };
    
};





template <class T, class TBase>
class LevelListAccessImpl : public LevelListAccess<TBase>
{
    LevelList<T> *list;
public:
    
    virtual ConstAccessIterator<TBase> begin()
    {
        return ConstAccessIteratorImpl<T, TBase, LevelList<T> >(list->begin());
    };
    virtual ConstAccessIterator<TBase> end()
    {
        return ConstAccessIteratorImpl<T, TBase, LevelList<T> >(list->end());
    };
    
    LevelListAccessImpl(LevelList<T> *list) : list(list)
    {
    };
    
    virtual ~LevelListAccessImpl() {}
};

}} // namespace maps
