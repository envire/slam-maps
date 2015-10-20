#ifndef __ENVIRE_MAPS_LIST_HPP__
#define __ENVIRE_MAPS_LIST_HPP__

#include <boost/multi_array.hpp>
#include <boost/intrusive/list.hpp>
#include <boost/pool/object_pool.hpp>

namespace envire 
{
	namespace maps 
	{
		template <typename T>
		class List {

		public:
			typedef boost::intrusive::constant_time_size<false> constant_time_size;

			typedef boost::intrusive::list<T, constant_time_size> Cell;
			typedef typename Cell::iterator CellItr;
			typedef typename Cell::const_iterator CellItrConst;

		protected:
			/*//The disposer object function
			struct delete_disposer
			{
			   void operator()(T *delete_this)
			   {  
			   	//std::cout << "delete: " << delete_this << std::endl;
			   	delete delete_this;  
			   }			   }

			};	

			struct cloner 
			{
				T* operator()(const T &clone_this)
				{	return new T(clone_this); }
			};*/

		public:
			List() 
				: cell(new Cell()),
				mem_pool(new boost::object_pool<T>())
			{
				//std::cout << "List: constructor " << cell << std::endl;
			}

			List(const List<T>& other) 
			{
				cell = new Cell();	
				mem_pool = new boost::object_pool<T>();		
				// use the assignment operator				
				this->operator=( other );				
			}

			~List()
			{
				//std::cout << "List: desctructor " << cell << std::endl;			
				if (mem_pool)
					delete mem_pool;
				if (cell)				
					delete cell;				
			}

			List& operator=( const List<T>& other )
			{
				clear();

				for (CellItrConst it = other.begin(); it != other.end(); ++it)
				{
					insertTail(*it);
				}			
				return *this;				
			}

			Cell* getPtr() const
			{
				return cell;
			}

			// Q: what is head and what is tail?
			void insertTail(const T& value)
			{
				T* value_t = mem_pool->construct(value);			
				cell->push_back(*value_t);
			}

			void insertHead(const T& value)
			{
				T* value_t = mem_pool->construct(value);
				cell->insert(cell->begin(), *value_t);		
			}

			CellItr begin()
			{
				return cell->begin();
			} 

		    CellItrConst begin() const
		    {
				return cell->begin();
		    }	

			CellItr end()
			{
				return cell->end();
			}

		    CellItrConst end() const
		    {
				return cell->end();
		    }	

		    CellItr erase(CellItr iterator)
		    {
		    	T* value = &*iterator;
		    	if (mem_pool->is_from(value) == false) 
		    		throw std::runtime_error("Wrong iterator.");

		    	mem_pool->destroy(value);

		    	return iterator++;
		    }

			void clear()
			{
				// since the we use auto_unlink_mode hook,
				// by deleting mem_pool all object will be deleted
				// therefore all object will be automaticly unlinked from the 
				// intrusive list
				if (mem_pool)
					delete mem_pool;
				mem_pool = new boost::object_pool<T>();	
			}

			size_t size() 
			{
				return cell->size();
			}

		protected:
		    Cell* cell;

		    boost::object_pool<T>* mem_pool;
			
		};

	}
}

#endif // __ENVIRE_MAPS_LIST_HPP__