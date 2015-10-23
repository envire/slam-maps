#ifndef __ENVIRE_MAPS_GRIDMAP_HPP__
#define __ENVIRE_MAPS_GRIDMAP_HPP__

#include <map>
#include <iostream>
#include <string>

#include <boost/multi_array.hpp>

#include "Grid.hpp"

namespace envire
{
	namespace maps
	{
		class GridMap : public GridBase {
		public:
			GridMap() : GridBase() {}

			/**
			 * @brief creat an empty grid of specific size and resolution
			 * @details Create an empty grid of specific size and resolution. 
			 * No layer will be created in this case. Call <createLayer>"()"
			 * to add layers to the grid.
			 * 
			 * @param config [description]
			 */
			// initialize the structure of grid
			// no layers will be created
			// call createLayer to add some layers to grid
			GridMap(GridConfig config) 
			    : GridBase(config){}

			~GridMap() 
			{
				removeAllGrids();
			}

			/**
			 * @brief create the layer with the specific key and default value
			 * @details create new layer with the specific key and initialize 
			 * the grid of this layer with the default value. this function can be failed
			 * in case if the layer with the same key already exists
			 * 
			 * The first layer will be set as basis layer
			 * 
			 * @param key the unique identification key of the layer
			 * @param default_value the default initialisation value for the layer
			 * @return true if the layer could be created successfully, otherwise false
			 */
			template <typename T>
			Grid<T>& addGrid(const std::string &key, const T &default_value)
			{
				if (hasGrid(key) == true)
				{
					throw std::out_of_range("GridMap::addGrid: The grid with the key '" + key + "' exists already.");
				}

				Grid<T> *new_grid = new Grid<T>(default_value, getGridConfig());
				grids[key] = new_grid;

				return *new_grid;
			}

			/**
			 * @brief [brief description]
			 * @details [long description]
			 * 
			 * @param key [description]
			 * @return [description]
			 */
			bool hasGrid(const std::string &key) const
			{ 
				typename MapType::const_iterator it = grids.find(key);
                if (it == grids.end())
                    return false;
                else 
                    return true;
			}

			bool removeGrid(const std::string &key)
			{
                if(hasGrid(key) == false)
                {
                    return false;
                }

                delete grids[key];
                grids.erase(key);

                return true;
			}

			void removeAllGrids() 
			{
				// TODO: check if the 
				typename MapType::iterator it;
				for (it = grids.begin(); it != grids.end(); ++it)
				{
					delete it->second;
				}
				grids.clear();
			}

			template <typename T>
			const Grid<T>& getGrid(const std::string &key) const
			{
				return *getGridPtr<T>(key);
			}			

			template <typename T>
			Grid<T>& getGrid(const std::string &key)
			{
				return *getGridPtr<T>(key);
			}

			std::vector<std::string> getAllGridKeys() const
			{
				std::vector<std::string> keys;
				typename MapType::const_iterator it;
				for (it = grids.begin(); it != grids.end(); ++it)
				{
					keys.push_back(it->first);
				}
				return keys;
			}

			void write(const std::string &key, const std::string &path) const
			{
				std::cout << "not implemented: " << __PRETTY_FUNCTION__ << std::endl;
			}

			void write(const std::string &key, std::ostream &os)
			{
				std::cout << "not implemented: " << __PRETTY_FUNCTION__ << std::endl;
			}

			void read(const std::string &key, const std::string &path)
			{
				std::cout << "not implemented: " << __PRETTY_FUNCTION__ << std::endl;
			}

			void read(const std::string &key, std::istream &is)
			{
				std::cout << "not implemented: " << __PRETTY_FUNCTION__ << std::endl;
			}

			void copy(const GridMap &src_grid)
			{
				std::cout << "not implemented: " << __PRETTY_FUNCTION__ << std::endl;
			}

			void cloneTo()
			{
				std::cout << "not implemented: " << __PRETTY_FUNCTION__ << std::endl;
			}

		protected:
			typedef std::map<std::string, GridBase*> MapType;
			MapType grids;

			template <typename T>
			Grid<T>* getGridPtr(const std::string &key) const
			{
				Grid<T>* grid = NULL;
				try {
					grid = dynamic_cast<Grid<T>*>(grids.at(key));	
					
					if (grid == NULL)
						throw std::runtime_error("The grid with the key '" + key + "' is not of required type.");
				} catch (const std::out_of_range &e) 
				{
					throw std::out_of_range("The map does not contain the grid with the key '" + key + "'.");
				} catch (const std::bad_cast &e)
				{
					throw std::runtime_error("The grid with the key '" + key + "' is not of required type.");
				}

				return grid;
			}
		};
	}
}

#endif // __ENVIRE_MAPS_GRIDMAP_HPP__