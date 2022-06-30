// PCL lib Functions for processing point clouds 

#ifndef LIDARALGO_H_
#define LIDARALGO_H_

#include <pcl/common/common.h>
#include "render/box.h"

#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void recursiveInsertion(Node **node, Node **data, int level=0)
	{
		if((*node) == NULL)
		{
			*node = *data;
		}
		else
		{
			int axis = level%3;

			if((*data)->point[axis] < (*node)->point[axis])
			{
				recursiveInsertion(&((*node)->left), data, level+1);
			}
			else
			{
				recursiveInsertion(&((*node)->right), data, level+1);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		Node* data = new Node(point, id);
		recursiveInsertion(&root, &data);
	}

	bool isPointInBox(Node **node, Node **target, float distanceTol)
	{
		bool is_in = true;
		float min_x, max_x, min_y, max_y, min_z, max_z;
		float point_x = (*node)->point[0];
		float point_y = (*node)->point[1];
		float point_z = (*node)->point[2];

		min_x = (*target)->point[0]-distanceTol;
		min_y = (*target)->point[1]-distanceTol;
		min_z = (*target)->point[2]-distanceTol;
		max_x = (*target)->point[0]+distanceTol;
		max_y = (*target)->point[1]+distanceTol;
		max_z = (*target)->point[2]+distanceTol;

		if((point_x < min_x || point_x > max_x) || (point_y< min_y || point_y > max_y) || (point_z< min_z || point_z > max_z))
		{
			is_in = false;
		}

		return is_in;
	}

	float distance(Node **node, Node **target)
	{
		float x_delta, y_delta, z_delta, distance;

		x_delta = (*target)->point[0]-(*node)->point[0];
		y_delta = (*target)->point[1]-(*node)->point[1];
		z_delta = (*target)->point[2]-(*node)->point[2];

		distance = sqrt(x_delta*x_delta + y_delta*y_delta + z_delta*z_delta);

		return distance;
	}

	void recursiveSearch(std::vector<int>* ids, Node **node, Node **target, float distanceTol, int level=0)
	{

		if((*node) != NULL)
		{
			if(isPointInBox(node, target, distanceTol))
			{
				if(distance(node, target) < distanceTol)
				{
					ids->push_back((*node)->id);
				}
			}

			int axis = level%3;

			if((*target)->point[axis]-distanceTol < (*node)->point[axis])
			{
				recursiveSearch(ids, &((*node)->left), target, distanceTol, level+1);
			}
			if((*target)->point[axis]+distanceTol > (*node)->point[axis])
			{
				recursiveSearch(ids, &((*node)->right), target, distanceTol, level+1);
			}

		}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		Node* target_node = new Node(target, 0);

		recursiveSearch(&ids, &root, &target_node, distanceTol);

		return ids;
	}
	

};

void proximity(std::vector<int>& cluster, std::vector<bool>& is_visited, const std::vector<std::vector<float>>& points, KdTree* tree, std::vector<float> point, int point_id, float distanceTol, int maxSize);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize);

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	
	srand(time(NULL));
	// TODO: Fill in this function

	// For max iterations 
	for(int iter=0; iter<maxIterations; iter++)
	{
		std::unordered_set<int> inliersResultTmp;
		while(inliersResultTmp.size() < 3)
		{	
			int idx = int(rand() % (cloud->points.size()));
			 
			// if point already considered 
			if (inliersResultTmp.begin() != inliersResultTmp.end())
			{
				int first_idx = *inliersResultTmp.begin();
				// chec if lying in the same z plane mor or less
				if(fabs(cloud->points[first_idx].z - cloud->points[first_idx].z) < 2*distanceTol)
				{
					inliersResultTmp.insert(idx);
				}
			}
			// if first point
			else
			{
				inliersResultTmp.insert(idx);
			}
		}

		auto itr = inliersResultTmp.begin();
		// Randomly sample subset and fit line
		PointT point1 = cloud->points[*(itr)]; itr++;
		PointT point2 = cloud->points[*(itr)]; itr++;
		PointT point3 = cloud->points[*(itr)]; itr++;
		
		float a = ((point2.y - point1.y)*(point3.z - point1.z)) - ((point2.z - point1.z)*(point3.y - point1.y)) ;
		float b = ((point2.z - point1.z)*(point3.x - point1.x)) - ((point2.x - point1.x)*(point3.z - point1.z)) ;
		float c = ((point2.x - point1.x)*(point3.y - point1.y)) - ((point2.y - point1.y)*(point3.x - point1.x)) ;
		float d = - (a*point1.x + b*point1.y + c*point1.z);

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(int index=0; index < cloud->points.size(); index++)
		{
			if(inliersResultTmp.count(index) > 0)
			{
				continue;
			}

			PointT point = cloud->points[index];
			float distance = fabs(a*point.x + b*point.y + c*point.z + d) / sqrt(a*a + b*b + c*c);

			if(distance <= distanceTol)
			{
				inliersResultTmp.insert(index);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliersResultTmp.size() > inliersResult.size())
		{
			inliersResult = inliersResultTmp;
		}
	}

	return inliersResult;
}


#endif /* PROCESSPOINTCLOUDS_H_ */