
#ifndef NEGRIDMAPCREATOR_H
#define NEGRIDMAPCREATOR_H

namespace edge2point{
class Edge2point;
}

struct SegPoint;

class NeGridMapCreator
{
	double x_min;
	double y_min;
	double x_max;
	double y_max;
	double search_radius;
	std::vector< std::vector<int> > grid_map;
	std::vector<SegPoint> seg_points;
	std::vector<double> yaws;
	edge2point::Edge2point *ep;

	bool isInRadius(int, int, SegPoint); // x, y, segpoint
	void createGridMap();

	public:
	NeGridMapCreator(std::string);
	std::vector<int> getNearEdge(double, double, double); // x, y, yaw
};

#endif

