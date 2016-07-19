#include <vizkit3d/StandaloneVisualizer.hpp>

#include <fstream>
#include <boost/archive/binary_iarchive.hpp>

int main(int argc, char **argv) {
	if(argc < 2)
	{
		std::cerr << "Usage " << argv[0] << " filename\n";
		exit(1);
	}

	std::ifstream input(argv[1],  std::ios::binary);
	boost::archive::binary_iarchive  ia(input);
	maps::grid::MLSMapSloped mls;

	ia >> mls;

	maps::grid::StandaloneVisualizer viz;

	viz.updateData(mls);

	while(viz.wait(1000))
	{
		// waiting
	}
}
