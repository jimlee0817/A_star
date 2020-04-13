#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include <cmath>
#include <sstream>
#include <vector>
#include <stack>

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
	double h;
	double g;
	double f;
};


int double2index(double x)
{
        int x_int;
        double x_temp;
        x_temp = (x+5)*10+0.5;
        x_int = (int)x_temp;
        return x_int;
}
double double2decimal1(double x)
{
        double x_decimal1;
        double x_temp;
        int x_int;
        x_temp = (x*10)+0.5;
        x_int = (int)x_temp;
        x_decimal1 = x_int/10.0;
        return x_decimal1;
}
double index2double(int x)
{
        double x_double;
        int x_temp;
        x_temp = x-50;
        x_double = (double)x_temp;
        x_double = x_double/10;
        double2decimal1(x_double);
        return x_double;
}

int cart2map(double x, double y)
{
	int idx = (x-(-5))*10 + (y-(-5))*1000;

	return mapGrid[idx];
}

bool isValid(double x, double y)
{
	if(x > X_MAX || x < X_MIN || y > Y_MAX || y < Y_MIN)	return false;
	
	for(double newX = -0.2; newX <= 0.2; newX = newX + 0.1)
	{
		for(double newY = -0.2; newY <= 0.2; newY = newY + 0.1)
		{
			if(cart2map(x + newX, y + newY) != 0) return false;
		}
		
	}
	
	
	return true;
	// Below is the picture on how I choose the valid point on the map (car:0)
	//-	  -	  -
	//  - - -
	//- - 0 - -
	//	- -	-
	//-	  -	  -
}

double calcHeuristic(double x, double y, Node goal)
{
	return sqrt((x - goal.x) * (x - goal.x) + (y - goal.y) * (y - goal.y));
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

bool isGoal(double x, double y, Node goal) // check goal state
{
	if(abs(x - goal.x) < 0.0001 && abs(y - goal.y) < 0.0001)
	{
		ROS_INFO("Is Goal (%f, %f)", goal.x, goal.y);
		return true;
	}
	else
	{	
		ROS_INFO("Not Goal (%f, %f) yet, now at (%f, %f)", goal.x, goal.y, x, y); 
		return false;
	}
}


vector<Node> makePath(Node map[100][100], Node goal)
{
	double x = goal.x;
	double y = goal.y;
	
	vector<Node> path;
	vector<Node> outputPath;
	int idx_x = double2index(x);
	int idx_y = double2index(y);
	
	while(!(map[idx_x][idx_y].parent_x == x && map[idx_x][idx_y].parent_y == y) && map[idx_x][idx_y].parent_x != -6 && map[idx_x][idx_y].parent_y != -6)
	{
		idx_x = double2index(x);
		idx_y = double2index(y);
		path.push_back(map[idx_x][idx_y]);
		x = map[idx_x][idx_y].parent_x;
		y = map[idx_x][idx_y].parent_y;
		idx_x = double2index(x);
		idx_y = double2index(y);
	}
	idx_x = double2index(x);
	idx_y = double2index(y);
	path.push_back(map[idx_x][idx_y]);
	
	//outputPath.resize(path.size());
	for(int i = path.size(); i >= 0; i--)
	{
		outputPath.push_back(path[i]);
	}
	return outputPath;
}

vector<Node> A_star(Node init, Node goal)
{
	vector<Node> empty;
	if(isValid(goal.x, goal.y) == false)
	{
		ROS_INFO("GOAL IST A WALL FUCK!!!!");
		return empty;
	}
	if(isGoal(init.x, init.y, goal))
	{
		ROS_INFO("YOU ARE AT GOAL!!!");
		return empty;
	}
	
	bool closedList[100][100];
	
	int idx_x;
	int idx_y;
	//INITIALIZE THE WHOLE MAP
	Node allMap[100][100];
	for(double x = -5; x <= 4.9; x = x + 0.1 )
	{
		for(double y = -5; y <= 4.9; y = y + 0.1)
		{
			idx_x = double2index(x);
			idx_y = double2index(y);
			allMap[idx_x][idx_y].f = FLT_MAX;
			allMap[idx_x][idx_y].g = FLT_MAX;
			allMap[idx_x][idx_y].h = FLT_MAX;
			allMap[idx_x][idx_y].parent_x = -6;
			allMap[idx_x][idx_y].parent_y = -6;
			allMap[idx_x][idx_y].x = x;
			allMap[idx_x][idx_y].y = y;
			
			closedList[idx_x][idx_y] = false;
		}
	}
	
	double x = init.x;
	double y = init.y;
	idx_x = double2index(x);
	idx_y = double2index(y);
	allMap[idx_x][idx_y].f = 0.0;
	allMap[idx_x][idx_y].g = 0.0;
	allMap[idx_x][idx_y].h = 0.0;
	allMap[idx_x][idx_y].parent_x = x;
	allMap[idx_x][idx_y].parent_y = y;
	
	vector<Node> openList;
	openList.push_back(allMap[idx_x][idx_y]); // push the inital point into the open list
	bool goalFound = false;
	
	while(!openList.empty() && openList.size() < 10000)
	{
		Node node;
		do
		{
			double temp = FLT_MAX;
			vector<Node>::iterator itNode;
			for(vector<Node>::iterator it = openList.begin(); it != openList.end(); it++)
			{
				Node n = *it;
				if(n.f < temp)
				{
					temp = n.f;
					itNode = it;
				}
			}
			node = *itNode;
			openList.erase(itNode);
		} while(!isValid(node.x, node.y));
		
		x = node.x;
		y = node.y;
		
		closedList[idx_x][idx_y] = true;
		
		for(double newX = -0.1; newX <= 0.1; newX = newX + 0.1)
		{
			for(double newY = -0.1; newY <= 0.1; newY = newY + 0.1)
			{
				double newF, newG, newH;
				idx_x = double2index(x + newX);
				idx_y = double2index(y + newY);
				
				if(isValid(x + newX, y + newY))
				{
					if(isGoal(x + newX, y + newY, goal))
					{
						allMap[idx_x][idx_y].parent_x = x;
						allMap[idx_x][idx_y].parent_y = y;
						goalFound = true;
						ROS_INFO("TESTING (%f, %f)", x, y);
						ROS_INFO("TESTING PARENT (%f, %f)", allMap[idx_x][idx_y].parent_x, allMap[idx_x][idx_y].parent_y);
						return makePath(allMap, goal);
					}
					else if(closedList[idx_x][idx_y] == false)
					{
						newG = node.g + sqrt(newX*newX + newY*newY);
						newH = calcHeuristic(x + newX, y + newY, goal);
						newF = newG + newH;
						// check if this path better than the other
						if(allMap[idx_x][idx_y].f == FLT_MAX || allMap[idx_x][idx_y].f > newF) // don't go back
						{
							allMap[idx_x][idx_y].f = newF;
							allMap[idx_x][idx_y].g = newG;
							allMap[idx_x][idx_y].h = newH;
							allMap[idx_x][idx_y].parent_x = x;
							allMap[idx_x][idx_y].parent_y = y;
							openList.push_back(allMap[idx_x][idx_y]);
						}
					}
				}
			}
		}
	}
	if(goalFound == false)
	{
		ROS_INFO("CAN'T FIND GOAL");
		return empty;
	}
	
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
			
			//nowPos.h = 0; nowPos.g = 0;
		
			nowPos.theta = theta;
			//nowPos.parent_x = -9999; nowPos.parent_y = -9999;
		
			ROS_INFO ("The Car is initial at (%f, %f, %f)", nowPos.x, nowPos.y, nowPos.theta);
			//ROS_INFO ("The Parant Node of the car position now is (%f, %f, %f)", nowPos.parent->x, nowPos.parent->y, nowPos.parent->theta);
			//getSuccessors(nowPos, goal1);
			path1 = A_star(nowPos, goal1);
			path2 = A_star(goal1, goal2);
			path3 = A_star(goal2, goal3);
			path4 = A_star(goal3, goal4);
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
		    	Tolerance = 0.2;
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
		       ROS_INFO ("velocity: %f", v);
		       ROS_INFO ("angular velocity: %f", w);
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

