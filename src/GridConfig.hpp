#ifndef __ENVIRE_MAPS_GRIDCONFIG_HPP__
#define __ENVIRE_MAPS_GRIDCONFIG_HPP__		

namespace envire 
{
	namespace maps 
	{
		struct GridConfig
		{
		public:
			GridConfig() :
				cellSizeX(0), cellSizeY(0),
				scaleX(0), scaleY(0),
				offsetX(0), offsetY(0)
			{};

			GridConfig(size_t cellSizeX, size_t cellSizeY,
						double scaleX, double scaleY,
						double offsetX = 0, double offsetY = 0)
				: cellSizeX(cellSizeX), cellSizeY(cellSizeY),
				scaleX(scaleX), scaleY(scaleY),
				offsetX(offsetX), offsetY(offsetY)
			{};

			size_t cellSizeX;
			size_t cellSizeY;
			double scaleX;
			double scaleY;
			double offsetX;
			double offsetY;
		};
	}
}

#endif // __ENVIRE_MAPS_GRIDCONFIG_HPP__		