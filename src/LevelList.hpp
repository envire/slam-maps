#pragma once

#include <set>
#include <boost/pool/object_pool.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/interprocess/allocators/private_node_allocator.hpp>
namespace envire 
{
namespace maps 
{

template <class S>
class LevelList : public std::multiset<S>
{
public:
    LevelList()
    {
    };
    
};

template <class _Tp>
struct myCmp : public std::binary_function<_Tp, _Tp, bool>
{
    bool
    operator()(const _Tp& __x, const _Tp& __y) const
    { return *__x < *__y; }
};

template <class S>
class LevelList<S *> : public std::multiset<S *, myCmp<S *>>
{
public:
    LevelList()
    {
    };
    
    
};


}
}
