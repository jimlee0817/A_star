#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include <cmath>
#include <sstream>
#include <vector>

#define X_MAX 4.9
#define X_MIN -5
#define Y_MAX 4.9
#define Y_MIN -5

using namespace std;

ros::Subscriber map_sub;
ros::Subscriber pos_sub;

const double PI = 3.14159265359;
int mapGrid[10000] = {}; 
double x, y, theta;
//vector<vector<double> > successors;
bool flagPos = false;
bool flagMap = false;


struct Node
{
	double x;
	double y;
	double theta;
	Node *parent;
	double h;
	double g;
	double f = g + h;
	vector<Node*> successors;
};

int cart2map(double x, double y)
{
	int idx = (x-(-5))*10 + (y-(-5))*1000;

	return mapGrid[idx];
}

bool isValid(Node node)
{
	if(node.x > X_MAX || node.x < X_MIN || node.y > Y_MAX || node.y < Y_MIN)	return false;
	
	if(cart2map(node.x, node.y) != 0)	return false;
	
	return true;
}

double calcHeuristic(Node node1, Node goal)
{
	return sqrt((node1.x - goal.x) * (node1.x - goal.x) + (node1.y - goal.y) * (node1.y - goal.y));
}

double constrainAngle(double x){
     
       //x = fmod(x+PI,PI*2);
       //if (x < 0)
          //x += PI*2;
       x = fmod(x,PI*2);
       if (x>=PI)
          x = -(PI*2-x);

       
       //return x-PI;
       return x;
}

void pos_Callback(const geometry_msgs::Twist::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
	x = msg->linear.x;
	y = msg->linear.y;
	theta =  msg->angular.z;
	theta = constrainAngle(theta);
    	flagPos = true;
}

void map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{
	ROS_INFO ("START MAP_CALLBACK!!!");
	for(int i = 0; i<10000; i++)
	{
		//ROS_INFO ("%d ", msg->data[i]);
		mapGrid[i] = msg->data[i];
	}
	flagMap = true;
}

void getSuccessors(Node node, Node goal)
{
	vector<vector<double> > successors;
	Node newNode;
	newNode.x = node.x + 0.1; 	newNode.y = node.y; // start from EAST
	if(isValid(newNode))
	{
		newNode.g = node.g + 1;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.parent = &node;
		node.successors.push_back(&newNode);

		//ROS_INFO("The first successor is (%f, %f) from parent node (%f, %f)", node.successors[0]->x, node.successors[0]->y, node.x, node.y);
	} 
	newNode.x = node.x - 0.1; 	newNode.y = node.y; // WEST
	if(isValid(newNode))
	{	
		newNode.g = node.g + 1;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.parent = &node;
		node.successors.push_back(&newNode); 
	}
	newNode.x = node.x; 	newNode.y = node.y + 0.1; // NORTH
	if(isValid(newNode))
	{	
		newNode.g = node.g + 1;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.parent = &node;
		node.successors.push_back(&newNode); 
	} 
	newNode.x = node.x; 	newNode.y = node.y - 0.1; // SOUTH
	if(isValid(newNode))
	{	
		newNode.g = node.g + 1;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.parent = &node;
		node.successors.push_back(&newNode); 
	}
	newNode.x = node.x + 0.1; 	newNode.y = node.y + 0.1; // NORTH-EAST
	if(isValid(newNode))
	{	
		newNode.g = node.g + sqrt(2);
		newNode.h = calcHeuristic(newNode, goal);
		newNode.parent = &node;
		node.successors.push_back(&newNode); 
	}
	newNode.x = node.x - 0.1; 	newNode.y = node.y + 0.1; // NORTH-WEST
	if(isValid(newNode)) 
	{	
		newNode.g = node.g + sqrt(2);
		newNode.h = calcHeuristic(newNode, goal);
		newNode.parent = &node;
		node.successors.push_back(&newNode); 
	}
	newNode.x = node.x + 0.1; 	newNode.y = node.y - 0.1; // SOUTH-EAST
	if(isValid(newNode)) 
	{	
		newNode.g = node.g + sqrt(2);
		newNode.h = calcHeuristic(newNode, goal);
		newNode.parent = &node;
		node.successors.push_back(&newNode); 
	}
	newNode.x = node.x - 0.1; 	newNode.y = node.y - 0.1; // SOUTH-WEST
	if(isValid(newNode)) 
	{	
		newNode.g = node.g + sqrt(2);
		newNode.h = calcHeuristic(newNode, goal);	
		newNode.parent = &node;		
		node.successors.push_back(&newNode); 
	}
}


vector<Node*> A_star(Node initNode, Node goalNode)
{
	vector<Node*> expandedList;
	Node *node;
	
	if(!isValid(initNode) || !isValid(goalNode))
	{
		ROS_INFO("Init: (%f, %f), Goal: (%f, %f)", initNode.x, initNode.y, goalNode.x, goalNode.y);
		ROS_INFO("GOAL or INITIAL Point is not valid");
		return {};
	}
	
	ROS_INFO("Start A* Algorithm");
	node = &initNode;

	while(node->x != goalNode.x && node->y != goalNode.y) // Check if it's goal state or not
	{
		ROS_INFO("Now we are at node (%f, %f)", node->x, node->y);
		double min = 1000000;
		int minIdx = 0;
		
		getSuccessors(*node, goalNode);
		for(int i = 0; i < node->successors.size(); i++)
		{	
			expandedList.push_back(node->successors[i]);
		}
		for(int i = 0; i < expandedList.size(); i++)
		{
			if(expandedList[i]->f < min)
			{
				minIdx = i;
				min = expandedList[i]->f;
			}
		}
		
		node = expandedList[minIdx];
		expandedList.erase(expandedList.begin() + minIdx);			
	}
	
	vector<Node*> reachedList; // to record the final path that we go through
	
	while(node->x == node->parent->x && node->y == node->parent->y)
	{
		reachedList.push_back(node);
		node = node->parent;
	}
	
	return reachedList;	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "A_star_sim");
	ros::NodeHandle n;

	map_sub = n.subscribe("/map", 1000, map_Callback);
	pos_sub = n.subscribe("/robot_pose", 1000, pos_Callback);

	ros::Rate loop_rate(10);
	Node goal1;	goal1.x = 3;	goal1.y = 4;
	
	Node nowPos;
	
	while(ros::ok())
	{
		ROS_INFO ("------------------START------------------");
		if(flagPos && flagMap)
		{
			nowPos.x = round(10 * x) / 10.0;
			nowPos.y = round(10 * y) / 10.0;
			nowPos.h = 0; nowPos.g = 0;
			
			nowPos.theta = theta;
			nowPos.parent = &nowPos;
			ROS_INFO ("The Car is initial at (%f, %f, %f)", nowPos.x, nowPos.y, nowPos.theta);
			//ROS_INFO ("The Parant Node of the car position now is (%f, %f, %f)", nowPos.parent->x, nowPos.parent->y, nowPos.parent->theta);
			//getSuccessors(nowPos, goal1);
			A_star(nowPos, goal1);
			
		}
	
	ros::spinOnce();
        loop_rate.sleep();
	ROS_INFO ("------------------END------------------");
	} 
	
	return 0;
}

