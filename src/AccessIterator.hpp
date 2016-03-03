#pragma once

#include <stdexcept>
#include <iostream>

namespace envire 
{
namespace maps 
{

template <class T>
class AccessIteratorInterface { 
public:
    typedef std::ptrdiff_t difference_type;
    typedef T value_type;
    typedef T& reference;
    typedef T* pointer;
    typedef std::forward_iterator_tag iterator_category;

    AccessIteratorInterface(){};
    virtual ~AccessIteratorInterface() {};

    virtual AccessIteratorInterface *getNewInstace() const = 0;
    
    virtual AccessIteratorInterface& operator=(const AccessIteratorInterface &it) = 0;
    
    virtual bool operator==(const AccessIteratorInterface &it) const = 0;

    virtual bool operator!=(const AccessIteratorInterface &it) const = 0;

    virtual void operator++() = 0;

    virtual void operator++(int i) = 0;

    virtual T &operator*() const = 0;

    virtual T *operator->() const = 0;
};


template <class T>
class AccessIterator { 
protected:
    AccessIteratorInterface<T> *impl;
public:
    typedef std::ptrdiff_t difference_type;
    typedef T value_type;
    typedef T& reference;
    typedef T* pointer;
    typedef std::forward_iterator_tag iterator_category;

    AccessIterator(const AccessIteratorInterface<T> &impl) : impl(impl.getNewInstace()) {};
    
    AccessIterator(const AccessIterator &it)  : impl(it.impl->getNewInstace()) {};
    ~AccessIterator() {
        delete impl;
    };

    AccessIterator& operator=(const AccessIterator &it) 
    {
        *impl = it.*impl;
        return *this;
    };
    
    bool operator==(const AccessIterator &it)
    {
        return impl->operator==(it);
    };

    bool operator!=(const AccessIterator &it)
    {
        return impl->operator!=(it);
    };

    AccessIterator& operator++()
    {
        impl->operator++();
        return *this;
    };

    AccessIterator operator++(int i)
    {
        impl->operator++(i);
        return *this;
    };

    reference operator*() const
    {
        return impl->operator*();
    };

    pointer operator->() const
    {
        return impl->operator->();
    };
};

template <class T>
class ConstAccessIteratorInterface { 
public:
    typedef std::ptrdiff_t difference_type;
    typedef const T value_type;
    typedef const T & reference;
    typedef const T * pointer;
    typedef std::forward_iterator_tag iterator_category;

    ConstAccessIteratorInterface() {
    };
    
    ConstAccessIteratorInterface(const ConstAccessIteratorInterface &it) {
    };
    
    virtual ~ConstAccessIteratorInterface() {};

    virtual ConstAccessIteratorInterface *getNewInstace() const = 0;

    virtual ConstAccessIteratorInterface& operator=(const ConstAccessIteratorInterface &it) = 0;
    
    virtual bool operator==(const ConstAccessIteratorInterface &it) const = 0;
    
    virtual bool operator!=(const ConstAccessIteratorInterface &it) const = 0;

    virtual void operator++() = 0;

    virtual void operator++(int i) = 0;

    virtual reference operator*() const = 0;
    
    virtual pointer operator->() const = 0;
};

template <class T>
class ConstAccessIterator { 
protected:
    ConstAccessIteratorInterface<T> *impl;
public:
    typedef std::ptrdiff_t difference_type;
    typedef const T value_type;
    typedef const T & reference;
    typedef const T * pointer;
    typedef std::forward_iterator_tag iterator_category;

    ConstAccessIterator(const ConstAccessIteratorInterface<T> &impl) : impl(impl.getNewInstace()) {
    };
    
    ConstAccessIterator(const ConstAccessIterator &it)  : impl(it.impl->getNewInstace()){
    };
    
    ~ConstAccessIterator() {
        delete impl;
    };

    ConstAccessIterator& operator=(const ConstAccessIterator &it) 
    {
        *impl = *it.impl;
        return *this;
    };
    
    bool operator==(const ConstAccessIterator &it)
    {
        return impl->operator==(it);
    };
    
    bool operator!=(const ConstAccessIterator &it)
    {
        return impl->operator!=(it);
    };

    ConstAccessIterator& operator++()
    {
        impl->operator++();
        return *this;
    };

    ConstAccessIterator operator++(int i)
    {
        impl->operator++(i);
        return *this;
    };

    reference operator*() const
    {
        return impl->operator*();
    };
    
    pointer operator->() const
    {
        return impl->operator->();
    };
};

template <class T, class TBase, class Iterator, class Interface>
class AccessIteratorInterfaceImplBase : public Interface
{ 
    protected:
        typedef Interface baseIt;
        Iterator it;
    public:
        AccessIteratorInterfaceImplBase(const Iterator &it) : it(it)
        {
        };
        
        AccessIteratorInterfaceImplBase(const AccessIteratorInterfaceImplBase &it): it(it.it) {
        };
        
        virtual ~AccessIteratorInterfaceImplBase() {};

        virtual Interface *getNewInstace() const
        {
            return new AccessIteratorInterfaceImplBase<T, TBase, Iterator, Interface>(it);
        };
        
        virtual baseIt& operator=(const baseIt &other)
        {
            it = static_cast<const AccessIteratorInterfaceImplBase &>(other).it;
            return *this;
        };
        
        virtual bool operator==(const baseIt &other) const
        {
            return it == static_cast<const AccessIteratorInterfaceImplBase &>(other).it;
        };
        
        virtual bool operator!=(const baseIt &other) const
        {
            return it != static_cast<const AccessIteratorInterfaceImplBase &>(other).it;
        };

        virtual void operator++()
        {
            it++;
        };
        
        virtual void operator++(int) {
            ++it;
        }
        
        virtual typename baseIt::reference operator*() const
        {
            return *static_cast<typename Interface::value_type *>(&(*it));
        };
        
        virtual typename baseIt::pointer operator->() const
        {
            return static_cast<typename Interface::value_type *>(it.operator->());
        };
};

template <class T, class TBase, class Container>
class AccessIteratorImpl : public AccessIterator<TBase>
{
public:
    AccessIteratorImpl(const typename Container::iterator &it) : AccessIterator<TBase>(AccessIteratorInterfaceImplBase<T, TBase, typename Container::iterator, AccessIteratorInterface<TBase> >(it))
    {
    };
};

template <class T, class TBase, class Container>
class ConstAccessIteratorImpl : public ConstAccessIterator<TBase>
{
public:
    ConstAccessIteratorImpl(const typename Container::const_iterator &it) : ConstAccessIterator<TBase>(AccessIteratorInterfaceImplBase<T, TBase, typename Container::const_iterator, ConstAccessIteratorInterface<TBase> >(it))
    {
    };
};

}
}