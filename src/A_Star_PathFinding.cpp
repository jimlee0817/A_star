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

ros::Publisher velocity_pub; // create a publisher
ros::Subscriber map_sub; // create a subscriber
ros::Subscriber pos_sub; // create a subscriber

const double PI = 3.14159265359;
int mapGrid[10000] = {}; 
double x, y, theta;
bool flagPos = false;
bool flagMap = false;
bool flagPath = false;
double K_rho = 0.3;
double K_alpha = 0.8;
double K_beta = -0.5;

struct Node
{
	double x;
	double y;
	double theta;
	double parent_x;
	double parent_y;
	double h = 0;
	double g = 0;
	double f = g + h;
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
	
	if(cart2map(node.x + 0.3, node.y) != 0)	return false;
	if(cart2map(node.x, node.y+ 0.3) != 0)	return false;
	if(cart2map(node.x - 0.3, node.y) != 0)	return false;
	if(cart2map(node.x, node.y - 0.3) != 0)	return false;
	if(cart2map(node.x + 0.3, node.y + 0.3) != 0)	return false;
	if(cart2map(node.x + 0.3, node.y - 0.3) != 0)	return false;
	if(cart2map(node.x - 0.3, node.y - 0.3) != 0)	return false;
	if(cart2map(node.x - 0.3, node.y + 0.3) != 0)	return false;
	
	if(cart2map(node.x + 0.2, node.y) != 0)	return false;
	if(cart2map(node.x, node.y+ 0.2) != 0)	return false;
	if(cart2map(node.x - 0.2, node.y) != 0)	return false;
	if(cart2map(node.x, node.y - 0.2) != 0)	return false;
	if(cart2map(node.x + 0.2, node.y + 0.2) != 0)	return false;
	if(cart2map(node.x + 0.2, node.y - 0.2) != 0)	return false;
	if(cart2map(node.x - 0.2, node.y - 0.2) != 0)	return false;
	if(cart2map(node.x - 0.2, node.y + 0.2) != 0)	return false;
	
	if(cart2map(node.x + 0.1, node.y) != 0)	return false;
	if(cart2map(node.x, node.y+ 0.1) != 0)	return false;
	if(cart2map(node.x - 0.1, node.y) != 0)	return false;
	if(cart2map(node.x, node.y - 0.1) != 0)	return false;
	if(cart2map(node.x + 0.1, node.y + 0.1) != 0)	return false;
	if(cart2map(node.x + 0.1, node.y - 0.1) != 0)	return false;
	if(cart2map(node.x - 0.1, node.y - 0.1) != 0)	return false;
	if(cart2map(node.x - 0.1, node.y + 0.1) != 0)	return false;
	
	
	return true;
	// Below is the picture on how I choose the valid point on the map (car:0)
	//-	  -	  -
	//  - - -
	//- - 0 - -
	//	- -	-
	//-	  -	  -
}

double calcHeuristic(Node node1, Node goal)
{
	return sqrt((node1.x - goal.x) * (node1.x - goal.x) + (node1.y - goal.y) * (node1.y - goal.y));
}

double constrainAngle(double x)
{
       x = fmod(x,PI*2);
       if (x>=PI)
          x = -(PI*2-x);
       return x;
}

double limitTurn(double x)
{
       if (x>PI)
          x = -(2*PI-x);
       else if(x<-PI)
          x = (2*PI+x);     
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

bool isRepeat(Node successor, Node node) // check if the successors go back or not
{
	if(successor.x == node.parent_x && successor.y == node.parent_y)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool isGoal(Node node, Node goal) // check goal state
{
	if(abs(node.x - goal.x) < 0.0001 && abs(node.y - goal.y) < 0.0001)
	{
		ROS_INFO("Is Goal");
		return true;
	}
	else
	{	
		ROS_INFO("Not Goal yet"); 
		return false;
	}
}

vector<Node> getSuccessors(Node node, Node goal) 
{
	vector<Node> successors;
	Node newNode;
	//ROS_INFO("In getSuccessors: node's parent: (%f, %f), node: (%f, %f)", node.parent_x, node.parent_y, node.x, node.y);
	double factor = calcHeuristic(node, goal);
	double inc;
	if(factor > 4) inc = 0.9;
	else if(factor <= 4 && factor > 2.5) inc = 0.5;
	else if(factor <= 2.5 && factor > 1) inc = 0.3;
	else if(factor <= 1 && factor > 0.5) inc = 0.2;
	else inc = 0.1; 
	
	ROS_INFO("The increment now is %f", inc);
	newNode.x = node.x + inc; 	newNode.y = node.y; // start from EAST
	if(isValid(newNode) && !isRepeat(newNode, node))
	{
		newNode.g = node.g + inc;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;
		successors.push_back(newNode);

		//ROS_INFO("The first successor is (%f, %f) from parent node (%f, %f)", node.successors[0]->x, node.successors[0]->y, node.x, node.y);
	} 
	newNode.x = node.x - inc; 	newNode.y = node.y; // WEST
	if(isValid(newNode) && !isRepeat(newNode, node))
	{	
		newNode.g = node.g + inc;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;
		successors.push_back(newNode); 
	}
	newNode.x = node.x; 	newNode.y = node.y + inc; // NORTH
	if(isValid(newNode) && !isRepeat(newNode, node))
	{	
		newNode.g = node.g + inc;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;
		successors.push_back(newNode); 
	} 
	newNode.x = node.x; 	newNode.y = node.y - inc; // SOUTH
	if(isValid(newNode) && !isRepeat(newNode, node))
	{	
		newNode.g = node.g + inc;
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;
		successors.push_back(newNode);
	}
	newNode.x = node.x + inc; 	newNode.y = node.y + inc; // NORTH-EAST
	if(isValid(newNode) && !isRepeat(newNode, node))
	{	
		newNode.g = node.g + sqrt(inc*inc + inc*inc);
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;
		successors.push_back(newNode);
	}
	newNode.x = node.x - inc; 	newNode.y = node.y + inc; // NORTH-WEST
	if(isValid(newNode) && !isRepeat(newNode, node)) 
	{	
		newNode.g = node.g + sqrt(inc*inc + inc*inc);
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;
		successors.push_back(newNode);
	}
	newNode.x = node.x + inc; 	newNode.y = node.y - inc; // SOUTH-EAST
	if(isValid(newNode) && !isRepeat(newNode, node)) 
	{	
		newNode.g = node.g + sqrt(inc*inc + inc*inc);
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;
		successors.push_back(newNode); 
	}
	newNode.x = node.x - inc; 	newNode.y = node.y - inc; // SOUTH-WEST
	if(isValid(newNode) && !isRepeat(newNode, node)) 
	{	
		newNode.g = node.g + sqrt(inc*inc + inc*inc);
		newNode.h = calcHeuristic(newNode, goal);
		newNode.f = newNode.g + newNode.h;	
		newNode.parent_x = node.x;
		newNode.parent_y = node.y;	
		successors.push_back(newNode); 
	}
	
	
	return successors;
}

vector<Node> A_star(Node initNode, Node goalNode)
{

	vector<vector<Node> > reachedList;
	Node node;
	node = initNode;
	if(!isValid(initNode) || !isValid(goalNode))
	{
		ROS_INFO("Init: (%f, %f), Goal: (%f, %f)", initNode.x, initNode.y, goalNode.x, goalNode.y);
		ROS_INFO("GOAL or INITIAL Point is not valid");
		return {};
	}
	
	vector<Node> reachedInit;
	reachedInit.push_back(node);
	//bool islstRun = true;
	int minIdx = 0;
	
	while(!isGoal(node, goalNode)) // Check if it's goal state or not
	{
		ROS_INFO("Now we are at node (%f, %f)", node.x, node.y);
		
		vector<Node> successors = getSuccessors(node, goalNode);
		ROS_INFO("We have successors:");
		bool canUse = false;
		for(int i = 0; i < successors.size(); i++)
		{
			ROS_INFO("(%f, %f), g = %f", successors[i].x, successors[i].y, successors[i].h);
			for(int j = 0; j < reachedInit.size(); j++)
			{
				if(!(abs(successors[i].x - reachedInit[j].x) < 0.0001 && abs(successors[i].y - reachedInit[j].y) < 0.0001))	canUse = true; // 查看successors是否往回走
				else canUse = false;
			}
			
			/*
			if(i == 0 && !islstRun)
			{
				reachedList[minIdx].push_back(successors[i]);
			}
			*/
			
			if(canUse)
			{
				vector<Node> reached;
				reached = reachedInit;
				reached.push_back(successors[i]); 
				int count = 0;
				
				for(int j = 0; j < reachedList.size(); j++)
				{
					for(int k = 0; k < reachedList[j].size(); k++)
					{
						if(abs(reached[k].x - reachedList[j][k].x) < 0.0001 && abs(reached[k].y - reachedList[j][k].y) < 0.0001) count++;
					}
				}
				if(!(count == reached.size()))
				{
					
					reachedList.push_back(reached);
				}
				else ROS_INFO("YOU GOT REPEATED PATH!!!!!!!!!!!!!!!!!!!");
			}
		}
		
		// Testing
		/*
		for(int temp = 0; temp < reachedList.size(); temp++)
		{
			ROS_INFO("Path %d is:", temp);
			for(int temp2 = 0; temp2 < reachedList[temp].size(); temp2++)
			{
				ROS_INFO("(%f, %f)", reachedList[temp][temp2].x, reachedList[temp][temp2].y);
			}
		}
		*/
		//****************
		
		double min = 10000000;
		
		
		for(int i = 0; i < reachedList.size(); i++)
		{	
			if(reachedList[i][reachedList[i].size() - 1].f < min)
			{
				min = reachedList[i][reachedList[i].size() -1].f;
				minIdx = i;
			}
			/*
			if(reachedList[i][reachedList[i].size() -1].h == 0)
			{
				//min = reachedList[i][reachedList[i].size() -1].f;
				minIdx = i;
				i = reachedList.size(); // out the for loop
				ROS_INFO("Find GOAL!!!!!!!!!!!!");
			}
			else
			{
				if(reachedList[i][reachedList[i].size() - 1].f < min)
				{
					min = reachedList[i][reachedList[i].size() -1].f;
					minIdx = i;
				}
			}
			*/
		}
		
		reachedInit = reachedList[minIdx];
		node = reachedList[minIdx][reachedList[minIdx].size() -1];
		vector<vector<Node> > newReachedList;
		for(int i = 0; i < reachedList.size(); i++)
		{
			if(i != minIdx)
			{
				newReachedList.push_back(reachedList[i]);
			}
		}
		
		reachedList = newReachedList;
		//ROS_INFO("Size if reachedList is %d", reachedList.size());
		//reachedList.erase(reachedList.begin() + minIdx);
		//ROS_INFO("After Size if reachedList is %d", reachedList.size());
		ROS_INFO("The path is: f = %f, List n = %d", reachedInit[reachedInit.size() - 1].f, reachedList.size());
		for(int i = 0; i < reachedInit.size(); i++)
		{
			ROS_INFO("(%f, %f)", reachedInit[i].x, reachedInit[i].y);
		}
		//islstRun = false;
	}
	return reachedInit;	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "A_star_sim");
	ros::NodeHandle n;

	map_sub = n.subscribe("/map", 1000, map_Callback);
	pos_sub = n.subscribe("/robot_pose", 1000, pos_Callback);

	ros::Rate loop_rate(10);

	Node nowPos;
	Node goal1;	goal1.x = 3;	goal1.y = 4;
	Node goal2;	goal2.x = 3;	goal2.y = -2;
	Node goal3;	goal3.x = -4;	goal3.y = -3;
	Node goal4;	goal4.x = -4;	goal4.y = 4;
	
	velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	geometry_msgs::Twist vel_msg;
	int k = 0;
	int goal_idx = 1;
	double rho, v, w, phiT, alpha, beta, Tolerance;
	vector<Node> path1;
	vector<Node> path2;
	vector<Node> path3;
	vector<Node> path4;
	int Mode = 1;
	int ModeSize = 0;
	while(ros::ok())
	{
		ROS_INFO ("------------------START------------------");
		
		if(flagPos && flagMap && k == 0)
		{
		
			nowPos.x = round(10 * x) / 10.0;
			nowPos.y = round(10 * y) / 10.0;
			nowPos.h = 0; nowPos.g = 0;
		
			nowPos.theta = theta;
			nowPos.parent_x = -9999; nowPos.parent_y = -9999;
		
			ROS_INFO ("The Car is initial at (%f, %f, %f)", nowPos.x, nowPos.y, nowPos.theta);
			//ROS_INFO ("The Parant Node of the car position now is (%f, %f, %f)", nowPos.parent->x, nowPos.parent->y, nowPos.parent->theta);
			//getSuccessors(nowPos, goal1);
			path1 = A_star(nowPos, goal1);
			path2 = A_star(path1[path1.size() - 1], goal2);
			path3 = A_star(path2[path2.size() - 1], goal3);
			path4 = A_star(path3[path3.size() - 1], goal4);
			ROS_INFO ("Path1: ");
			for(int i = 0; i < path1.size(); i++)
			{
				ROS_INFO ("(%f, %f)", path1[i].x, path1[i].y);
			}
			ROS_INFO ("Path2: ");
			for(int i = 0; i < path2.size(); i++)
			{
				ROS_INFO ("(%f, %f)", path2[i].x, path2[i].y);
			}
			
			ROS_INFO ("Path3: ");
			for(int i = 0; i < path3.size(); i++)
			{
				ROS_INFO ("(%f, %f)", path3[i].x, path3[i].y);
			}
			
			ROS_INFO ("Path4: ");
			for(int i = 0; i < path4.size(); i++)
			{
				ROS_INFO ("(%f, %f)", path4[i].x, path4[i].y);
			}
			
			k++;
			flagPath = true;		
		}
		
		// Now start to move the car
		vector<Node> PATH;
		
		if(Mode == 1)
		{
			PATH = path1;
		}
		else if(Mode == 2)
		{
			PATH = path2;
		}
		else if(Mode == 3)
		{
			PATH = path3;
		}
		else if(Mode == 4)
		{
			PATH = path4;
		}
		
		if(flagPath)
		{
			Node goalStep = PATH[goal_idx];
			goalStep.theta = constrainAngle(atan2(goalStep.y - PATH[goal_idx - 1].y, goalStep.x - PATH[goal_idx - 1].x));
		
		    double AngleD = goalStep.theta - theta;
		    double AngleT = 0.005;
		    ROS_INFO ("delta Angle: %f", AngleD / PI*180);
		    ROS_INFO ("Distance: %f", rho);
		    
		    rho = sqrt(pow(goalStep.x - x,2) + pow(goalStep.y - y,2));
		    
		    ROS_INFO ("Now Goal: (%f, %f, %f)", goalStep.x, goalStep.y, goalStep.theta * 180 / PI);
		    if(goal_idx != PATH.size() - 1)
		    {
		    	if(abs(goalStep.parent_x - goalStep.x) == 0)
		    		Tolerance = abs(goalStep.parent_y - goalStep.y) - 0.08;
		    	else
		    		Tolerance = abs(goalStep.parent_x - goalStep.x) - 0.08;
		    }
		    else
		    {
		    	Tolerance = 0.01;
		    }
		    
		    if (rho < Tolerance)// Stop when close to the goal
		    {
		       goal_idx++;
		       if(goal_idx == PATH.size())
		       {
					goal_idx = 1;
					v = 0; w = 0;
					Mode++;
		       }
		       
		       vel_msg.linear.x = v;
		       vel_msg.angular.z = w;
		       velocity_pub.publish(vel_msg);
			}
		    else
		    {
		       phiT = atan2(goalStep.y - y, goalStep.x - x);
		       
		       double phiT2deg = phiT/PI*180;
		       ROS_INFO ("phiT: %f", phiT2deg);

		       alpha = phiT - theta;
		       alpha = fmod(alpha, 2*PI);
		       alpha = limitTurn(alpha);

		       beta = goalStep.theta - theta - alpha;
		       //beta = theta_goal-theta_pose;
		       beta = fmod(beta, 2*PI);
		       beta = limitTurn(beta);
		       
		       double theta2degree =  theta / PI * 180;
		       ROS_INFO ("theta: %f", theta / PI * 180);
		       
		       v = rho * K_rho;;
		       w = alpha * K_alpha + beta * K_beta;
		       if(v > 0.5){	v = 0.5;}
		       if(w > 1.0){	w = 1.0;}
		       vel_msg.linear.x = v;
		       vel_msg.angular.z = w;
		       velocity_pub.publish(vel_msg); 
		    }                  
		}
		
                
		ros::spinOnce();
        loop_rate.sleep();
		ROS_INFO ("------------------END------------------");
	} 
	
	return 0;
}

