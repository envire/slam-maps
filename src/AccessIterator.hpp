#pragma once

#include <stdexcept>

namespace envire 
{
namespace maps 
{


template <class T, class A = std::allocator<T> >
class AccessIterator { 
public:
    typedef typename A::difference_type difference_type;
    typedef typename A::value_type value_type;
    typedef typename A::reference reference;
    typedef typename A::pointer pointer;
    typedef std::forward_iterator_tag iterator_category;

    AccessIterator() {};
    AccessIterator(const AccessIterator&) {};
    virtual ~AccessIterator() {};

    virtual AccessIterator& operator=(const AccessIterator&) 
    {
        throw std::runtime_error("Full virtual method called");
    };
    
    virtual bool operator==(const AccessIterator&)
    {
        throw std::runtime_error("Full virtual method called");
    };
    virtual bool operator!=(const AccessIterator&)
    {
        throw std::runtime_error("Full virtual method called");
    };

    virtual AccessIterator& operator++()
    {
        throw std::runtime_error("Full virtual method called");
    };

    virtual reference operator*() const
    {
        throw std::runtime_error("Full virtual method called");
    };
    virtual pointer operator->() const
    {
        throw std::runtime_error("Full virtual method called");
    };
};

template <class T, class A = std::allocator<T> >
class ConstAccessIterator { 
public:
    typedef typename A::difference_type difference_type;
    typedef const typename A::value_type value_type;
    typedef typename A::const_reference reference;
    typedef typename A::const_pointer pointer;
    typedef std::forward_iterator_tag iterator_category;

    ConstAccessIterator() {};
    ConstAccessIterator(const ConstAccessIterator&) {};
    virtual ~ConstAccessIterator() {};

    virtual ConstAccessIterator& operator=(const ConstAccessIterator&) 
    {
        throw std::runtime_error("Full virtual method called");
    };
    
    virtual bool operator==(const ConstAccessIterator&)
    {
        throw std::runtime_error("Full virtual method called");
    };
    virtual bool operator!=(const ConstAccessIterator&)
    {
        throw std::runtime_error("Full virtual method called");
    };

    virtual ConstAccessIterator& operator++()
    {
        throw std::runtime_error("Full virtual method called");
    };

    virtual reference operator*() const
    {
        throw std::runtime_error("Full virtual method called");
    };
    virtual pointer operator->() const
    {
        throw std::runtime_error("Full virtual method called");
    };
};

template <class T, class TBase, class Container, class OwnBase>
class AccessIteratorImplBase : public OwnBase
{ 
    protected:
        typedef OwnBase baseIt;
        typename Container::iterator it;
    public:
        AccessIteratorImplBase(typename Container::iterator it) : it(it)
        {
        };
        AccessIteratorImplBase(const AccessIteratorImplBase&) {};
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
class AccessIteratorImpl : public AccessIteratorImplBase<T, TBase, Container, AccessIterator<TBase> >
{
public:
    AccessIteratorImpl(typename Container::iterator it) : 
        AccessIteratorImplBase<T, TBase, Container, AccessIterator<TBase> >(it)
    {
    };
};

template <class T, class TBase, class Container>
class ConstAccessIteratorImpl : public AccessIteratorImplBase<T, TBase, Container, ConstAccessIterator<TBase> >
{
public:
    ConstAccessIteratorImpl(typename Container::const_iterator it) : 
        AccessIteratorImplBase<T, TBase, Container, ConstAccessIterator<TBase> >(it)
    {
    };
    
};

}
}