#pragma once

#include <set>
#include <boost/pool/object_pool.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/interprocess/allocators/private_node_allocator.hpp>
#include "AccessIterator.hpp"

namespace envire 
{
namespace maps 
{
template <class _Tp>
struct myCmp : public std::binary_function<_Tp, _Tp, bool>
{
    bool
    operator()(const _Tp& __x, const _Tp& __y) const
    { return *__x < *__y; }
};
    
template <class S>
class LevelList : public std::multiset<S>
{
public:
    LevelList()
    {
    };    
};

template <class S>
class LevelList<S *> : public std::multiset<S *, myCmp<S *>>
{
public:
    LevelList()
    {
    };
    
    
};

template <class T>
class LevelListAccess
{
public:

    
    virtual ConstAccessIterator<T> begin() = 0;
    virtual ConstAccessIterator<T> end() = 0;
    
    LevelListAccess()
    {
    };
    
};

template <class S, class SBase = S>
class DerivableLevelList : public DerivableLevelList<SBase, SBase>, public std::multiset<S>
{
protected:
    virtual ConstAccessIterator<SBase> getBegin()
    {
        return ConstAccessIteratorImpl<S, SBase, std::multiset<S> >(std::multiset<S>::begin());
    };
    
    virtual ConstAccessIterator<SBase> getEnd()
    {
        return ConstAccessIteratorImpl<S, SBase, std::multiset<S> >(std::multiset<S>::end());
    };
    
    virtual size_t getSize() const
    {
        return std::multiset<S>::size();
    };
public:
    using std::multiset<S>::begin;
    using std::multiset<S>::end;

    using std::multiset<S>::find;
    
    using std::multiset<S>::size;
    
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
    
};

}
}
