#include <ros/ros.h>
#include "node_edge/edge2point.h"
#include "voronoi/NeGridMapCreator.h"

using namespace std;

NeGridMapCreator::NeGridMapCreator(string fname)
{
	ep = new edge2point::Edge2point();
	ep->loadCSV(fname);
	ep->getMinXY(x_min, y_min);
	ep->getMinDist(search_radius);
	ep->getMapSize(x_max, y_max);
	ep->getEdgeYaws(yaws);
	ep->createPoints(seg_points);

	// cout << "seg_points.size() : " << seg_points.size() << endl;

	search_radius /= 2.0;
	if(search_radius < 15){
		search_radius = 15;
	}

	x_min -= search_radius;
	y_min -= search_radius;
	x_max += 2 * search_radius;
	y_max += 2 * search_radius;

	createGridMap();
	// ep->pubPoints(seg_points);
}

bool NeGridMapCreator::isInRadius(int x, int y, SegPoint sp)
{
	double dist = sqrt(pow(x - sp.x, 2) + pow(y - sp.y, 2));
	if(dist < search_radius) return true;
	else return false;
}

void NeGridMapCreator::createGridMap()
{
	int id;
	int edge;
	int grid_size = y_max * x_max;
	int per_cnt = 1;
	vector< vector<int> > gm((y_max + 1) * (x_max + 1) + (x_max + 1));

	cout << "\n\ncreating voronoi diagram..." << endl;
	cout << "[-------------------------]" << endl;
	// printf("\033[%dA" , 1); //up
	// printf("\033[%dC" , 1); //right
	// printf("\033[%dD" , 26); //left
	for(int y = 0; y <= int(y_max); y++){
		for(int x = 0; x <= int(x_max); x++){
			id = y * (int)x_max + x;
			for(auto seg_point = seg_points.begin(); seg_point != seg_points.end(); ++seg_point){
				if(isInRadius(x + (int)x_min, y + (int)y_min, *seg_point)){
					edge = seg_point->edge;
					auto itr = find(gm[id].begin(), gm[id].end(), edge);
					if( itr == gm[id].end() ) {
						gm[id].push_back(edge);
					}
				}
			}
			if(id > (grid_size / 25) * per_cnt){
				printf("\033[%dA" , 1);
				printf("\033[%dC" , per_cnt);
				cout << "=>" << endl;
				per_cnt++;
			}
			if(!ros::ok()){
				cout << "interrupt" << endl;
				return;
			}
		}
	}
	printf("\033[%dA" , 1);
	printf("\033[%dC" , per_cnt);
	cout << "] 100 %\n" << endl;
	// cout << "finish!" << endl;
	
	grid_map = gm;
}

vector<int> NeGridMapCreator::getNearEdge(double x, double y, double yaw)
{
	vector<int> edges;
	int id = int(x_max)*int(y - y_min) + int(x - x_min);
	// if(id < 0 || id > (y_max + 1) * (x_max + 1) + (x_max + 1)) return edges;
	if(id < 0 || id > (int)grid_map.size()){
		cout << "id : " << id << endl;
		return edges;
	}
	
	edges = grid_map[id];
	double diff = 0.0;

	for(auto edge = edges.begin(); edge != edges.end(); ++edge){
		diff = fabs(yaws[*edge] - yaw);
		if(diff > M_PI) diff = 2*M_PI - diff;
		if(diff > M_PI/3.5){
			edges.erase(edge);
			--edge;
		}
	}

	return edges;
}

