#include "lidarAlgo.h"

void proximity(std::vector<int>& cluster, 
				std::vector<bool>& is_visited, 
				const std::vector<std::vector<float>>& points, 
				KdTree* tree,
				std::vector<float> point, 
				int point_id,
				float distanceTol,
				int maxSize)
{
	// node is being visited
	is_visited[point_id] = true;

	//add node to cluster
	cluster.push_back(point_id);

	// look for close points
	std::vector<int> ids = tree->search(point, distanceTol);

	// repeat over close points
	for(int id: ids)
	{
		if((is_visited[id] == false) && (cluster.size() < maxSize))
		{
			proximity(cluster, is_visited, points, tree, points[id], id, distanceTol, maxSize);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> is_visited(points.size(), false);
	
	for(int id=0; id< points.size(); id++)
	{
		if(is_visited[id] == false)
		{
			// create new cluster
			std::vector<int> cluster;

			// fill up cluster 
			proximity(cluster, is_visited, points, tree, points[id], id, distanceTol, maxSize);

			if (cluster.size() > minSize)
			{
				// add new cluster to list
				clusters.push_back(cluster);
			}
		}
	}
	
	return clusters;
}

