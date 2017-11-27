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

#include <stdexcept>
#include <iostream>

namespace maps { namespace grid
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
        *impl = *(it.impl);
        return *this;
    };
    
    bool operator==(const AccessIterator &it)
    {
        return impl->operator==(*(it.impl));
    };

    bool operator!=(const AccessIterator &it)
    {
        return impl->operator!=(*(it.impl));
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

}}
