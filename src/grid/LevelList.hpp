#pragma once

#include "AccessIterator.hpp"

#include <boost/container/flat_set.hpp>

namespace maps 
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
public:
    LevelList()
    {
    };    

    void update(const S&, const MLSConfig&);
};

template <class S>
class LevelList<S *> : public boost::container::flat_set<S *, myCmp<S *>>
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
    }
    virtual ~LevelListAccess() {}
    
};

template <class S, class SBase = S>
class DerivableLevelList : public DerivableLevelList<SBase, SBase>, public boost::container::flat_set<S>
{
protected:
    virtual ConstAccessIterator<SBase> getBegin()
    {
        return ConstAccessIteratorImpl<S, SBase, boost::container::flat_set<S> >(boost::container::flat_set<S>::begin());
    };
    
    virtual ConstAccessIterator<SBase> getEnd()
    {
        return ConstAccessIteratorImpl<S, SBase, boost::container::flat_set<S> >(boost::container::flat_set<S>::end());
    };
    
    virtual size_t getSize() const
    {
        return boost::container::flat_set<S>::size();
    };
public:
    using boost::container::flat_set<S>::begin;
    using boost::container::flat_set<S>::end;

    using boost::container::flat_set<S>::find;
    
    using boost::container::flat_set<S>::size;
    
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
    
};

} // namespace maps
