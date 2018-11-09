#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#define PI 3.141592

class RRT{

private:

  struct node{
    geometry_msgs::Point wp;
    node* parent;
    std::vector<node*> childs;
    std::vector<float> headingBlackList;
  };

  node* root;
  node* current;
  node* temp;
  void AddNodeToCurrentPrivate(geometry_msgs::Point wp);
  node* CreateLeaf(geometry_msgs::Point wp);
  void RemoveRRT(node* Ptr);
  void MoveForward(void);
  int CompareWp(geometry_msgs::Point wp, float alpha);
  float dwp_th; //0.75*1 if 1 is the step
  float angCompTh; // 2*arcsin(dwp_th/(2*step))
  float d1,d2;
  float angComp;
  std::vector<geometry_msgs::Point> bestPath;
  geometry_msgs::Point generatePoint(geometry_msgs::Point basept,float alpha);
  geometry_msgs::Point getCurrentPrivate(void);
  float step = 2.0;

  geometry_msgs::Point wp_n;

public:
  RRT(geometry_msgs::Point initwp);
  ~RRT();
  int AddNodeToCurrent(float alpha,geometry_msgs::Point wpPosition);
  bool MoveBackward(void);
  geometry_msgs::Point getCurrent(void);
  void AddObstacleChild(float alpha);
  int getNumberOfTrials(void);
  float getParentGlobalYaw(void);
  bool isValidWp(float alpha);
};
