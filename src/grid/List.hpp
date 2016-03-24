#ifndef __MAPS_LIST_HPP__
#define __MAPS_LIST_HPP__

#include <boost/multi_array.hpp>
#include <boost/intrusive/list.hpp>
#include <boost/pool/object_pool.hpp>

namespace maps 
{
    template <typename T>
    class List {

    public:
        typedef boost::intrusive::constant_time_size<false> constant_time_size;

        typedef boost::intrusive::list<T, constant_time_size> Holder;
        typedef typename Holder::iterator iterator;
        typedef typename Holder::const_iterator const_iterator;

    public:
        List() 
            : mem_pool(new boost::object_pool<T>())
        {
        }

        List(const List<T>& other) 
        {
            mem_pool = new boost::object_pool<T>();     
            // use the assignment operator              
            this->operator=( other );               
        }

        ~List()
        {       
            if (mem_pool)
                delete mem_pool;
        }

        List& operator=( const List<T>& other )
        {
            clear();

            for (const_iterator it = other.begin(); it != other.end(); ++it)
            {
                insertTail(*it);
            }           
            return *this;               
        }

        // Q: what is head and what is tail?
        void insertTail(const T& value)
        {
            T* value_t = mem_pool->construct(value);            
            holder.push_back(*value_t);
        }

        void insertHead(const T& value)
        {
            T* value_t = mem_pool->construct(value);
            holder.push_front(*value_t);
        }

        //! inserts \c value before position \c it
        iterator insert(iterator it, const T& value)
        {
            T* value_t = mem_pool->construct(value);
            return holder.insert(it, *value_t);
        }

        iterator begin()
        {
            return holder.begin();
        } 

        const_iterator begin() const
        {
            return holder.begin();
        }   

        iterator end()
        {
            return holder.end();
        }

        const_iterator end() const
        {
            return holder.end();
        }   

        iterator erase(iterator it)
        {
            T* value = &*it;
            ++it;
            if (mem_pool->is_from(value) == false) 
                throw std::runtime_error("Wrong iterator.");

            mem_pool->destroy(value);

            return it;
        }

        void clear()
        {
            // since the we use auto_unlink_mode hook,
            // by deleting mem_pool all object will be deleted
            // therefore all object will be automatically unlinked from the
            // intrusive list
            if (mem_pool)
                delete mem_pool;
            mem_pool = new boost::object_pool<T>();
        }

        size_t size() 
        {
            return holder.size();
        }

    protected:
        Holder holder;

        boost::object_pool<T>* mem_pool;
        
    };

}

#endif // __MAPS_LIST_HPP__