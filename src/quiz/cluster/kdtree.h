/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
			int axis = level%2;

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
		float min_x, max_x, min_y, max_y;
		float point_x = (*node)->point[0];
		float point_y = (*node)->point[1];

		min_x = (*target)->point[0]-distanceTol;
		min_y = (*target)->point[1]-distanceTol;
		max_x = (*target)->point[0]+distanceTol;
		max_y = (*target)->point[1]+distanceTol;

		if((point_x < min_x || point_x > max_x) || (point_y< min_y || point_y > max_y))
		{
			is_in = false;
		}

		return is_in;
	}

	float distance(Node **node, Node **target)
	{
		float x_delta, y_delta, distance;

		x_delta = (*target)->point[0]-(*node)->point[0];
		y_delta = (*target)->point[1]-(*node)->point[1];

		distance = sqrt(x_delta*x_delta + y_delta*y_delta);

		return distance;
	}

	void recursiveSearch(std::vector<int>* ids, Node **node, Node **target, float distanceTol, int level=0)
	{

		if((*node) != NULL)
		{
			if(isPointInBox(node, target, distanceTol))
			{
				if(distance(node, target)  <distanceTol)
				{
					ids->push_back((*node)->id);
				}
			}

			int axis = level%2;

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




