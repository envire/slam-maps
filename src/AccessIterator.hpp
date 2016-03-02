#pragma once

#include <stdexcept>
#include <iostream>

namespace envire 
{
namespace maps 
{


template <class T, class A = std::allocator<T> >
class AccessIterator { 
protected:
    AccessIterator *impl;
public:
    typedef typename A::difference_type difference_type;
    typedef typename A::value_type value_type;
    typedef typename A::reference reference;
    typedef typename A::pointer pointer;
    typedef std::forward_iterator_tag iterator_category;

    AccessIterator(AccessIterator *impl) : impl(impl) {};
    AccessIterator(const AccessIterator &it)  : impl(it.impl) {};
    virtual ~AccessIterator() {};

    virtual AccessIterator& operator=(const AccessIterator &it) 
    {
        impl = it.impl;
        return impl->operator=(it);
    };
    
    virtual bool operator==(const AccessIterator &it)
    {
        return impl->operator==(it);
    };

    virtual bool operator!=(const AccessIterator &it)
    {
        return impl->operator!=(it);
    };

    virtual AccessIterator& operator++()
    {
        return impl->operator++();        
    };

    virtual AccessIterator operator++(int i)
    {
        return impl->operator++(i);
    };

    virtual reference operator*() const
    {
        return impl->operator*();
    };

    virtual pointer operator->() const
    {
        return impl->operator->();
    };
};

template <class T, class A = std::allocator<T> >
class ConstAccessIterator { 
protected:
    ConstAccessIterator *impl;
public:
    typedef typename A::difference_type difference_type;
    typedef const typename A::value_type value_type;
    typedef typename A::const_reference reference;
    typedef typename A::const_pointer pointer;
    typedef std::forward_iterator_tag iterator_category;

    ConstAccessIterator(ConstAccessIterator *impl) : impl(impl) {
        std::cout << "Construction" << std::endl;
    };
    ConstAccessIterator(const ConstAccessIterator &it)  : impl(it.impl){
        std::cout << "Copy by reference impl p is " << it.impl << " own " << impl << std::endl;
    };
    virtual ~ConstAccessIterator() {};

    virtual ConstAccessIterator& operator=(const ConstAccessIterator &it) 
    {
        std::cout << "Copy by equal" << std::endl;
        impl = it.impl;
        return impl->operator=(it);
    };
    
    virtual bool operator==(const ConstAccessIterator &it)
    {
        return impl->operator==(it);
    };
    
    virtual bool operator!=(const ConstAccessIterator &it)
    {
        return impl->operator!=(it);
    };

    virtual ConstAccessIterator& operator++()
    {
        return impl->operator++();
    };

    virtual ConstAccessIterator operator++(int i)
    {
        return impl->operator++(i);
    };

    virtual reference operator*() const
    {
        return impl->operator*();
    };
    
    virtual pointer operator->() const
    {
        return impl->operator->();
    };
};

template <class T, class TBase, class Iterator, class OwnBase>
class AccessIteratorImplBase : public OwnBase
{ 
    protected:
        typedef OwnBase baseIt;
        Iterator it;
        AccessIteratorImplBase() : OwnBase(nullptr)
        {
        };
    public:
        AccessIteratorImplBase(Iterator it) : OwnBase(new AccessIteratorImplBase())
        {
            static_cast<AccessIteratorImplBase *>(OwnBase::impl)->it = it;
        };
        AccessIteratorImplBase(const AccessIteratorImplBase &it): OwnBase(it), it(it.it) {
            std::cout << "Copy by Reference ImplBase" << std::endl;
        };
        virtual ~AccessIteratorImplBase() {};

        virtual baseIt& operator=(const baseIt &other)
        {
            it = static_cast<const AccessIteratorImplBase &>(other).it;
            return *this;
        };
        
        virtual bool operator==(const baseIt &other) const
        {
            return it == static_cast<const AccessIteratorImplBase &>(other).it;
        };
        
        virtual bool operator!=(const baseIt &other) const
        {
            return it != static_cast<const AccessIteratorImplBase &>(other).it;
        };

        virtual baseIt& operator++()
        {
            it++;
            return *this;
        };
        
        virtual baseIt operator++(int) {
            ++it;
            return *this;
        }
        
        virtual typename baseIt::reference operator*() const
        {
            return *static_cast<typename OwnBase::value_type *>(&(*it));
        };
        
        virtual typename baseIt::pointer operator->() const
        {
            return static_cast<typename OwnBase::value_type *>(it.operator->());
        };
};

template <class T, class TBase, class Container>
class AccessIteratorImpl : public AccessIteratorImplBase<T, TBase, typename Container::iterator, AccessIterator<TBase> >
{
public:
    AccessIteratorImpl(typename Container::iterator it) : 
        AccessIteratorImplBase<T, TBase, typename Container::iterator, AccessIterator<TBase> >(it)
    {
    };
};

template <class T, class TBase, class Container>
class ConstAccessIteratorImpl : public AccessIteratorImplBase<T, TBase, typename Container::const_iterator, ConstAccessIterator<TBase> >
{
public:
    ConstAccessIteratorImpl(typename Container::const_iterator it) : 
        AccessIteratorImplBase<T, TBase, typename Container::const_iterator, ConstAccessIterator<TBase> >(it)
    {
    };
    
};

}
}