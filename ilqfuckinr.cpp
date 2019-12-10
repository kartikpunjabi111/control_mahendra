#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <unistd.h>
#include "vector"
#include <beginner_tutorials/Control.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#define scale 20000
#define constD 1
#define k 0.5
using namespace ros;
using namespace std;
using namespace Eigen;


double M=1356;
// double Iz=2681.95008625;
// double Cf=155494.663;
// double Cr=155494.663;
double v=0;
double lf=1.1898;
double lr=0.7932;
double delta=0;
double phi=0;
double sampling=0.01;
double theta=0;


MatrixXcd A(4,4);
MatrixXcd B(4,1);
MatrixXcd Ad(4,4);
MatrixXcd Bd(4,1);
MatrixXcd diagA(4,4);
MatrixXcd diagB(4,4);
MatrixXcd eigVal(4,1);
MatrixXcd eigVect(4,4);
MatrixXcd C_badi(5,3);

int i = 0;

float j,e ;

float a, b;
//float Xpos[100], Ypos[100];

//float d = 5.0
// void discretise(); //this will give Ad and Bd, from A and B using sampling time
// void riccati();   //this will take Ad and Bd and give P to get K  // for A-B*K 
void Ft();
void Ct();
void ct();
void LQRbackwardPass();
void forwardPass_on_real_non_linear_dynamics();
void update(); //update xt and ut on the basis of state and actions in the forward pass
void badi_matrix();

//void steer();
beginner_tutorials::Control v;
std_msgs::Float64 msg;      // to record bag file for Cross track Error
double wheelBase=1.983;

struct path_point{
	double px,py,d; // path points and  d=distance(ed) of car from that particular point index
	double slope;	// slope at that point(0p)
	int index;		//	
};

int direction=0;  //To check whether it is LHS or RHS to the path
path_point p;   //variable p is of type struct(path_point)
double x=0,y=0;   // coordinates of car
double ayaw=0,ryaw0=0,ryaw1=0,ryawDiff=0,speed=5,error0=0,error1=0,errorDiff=0,angleOffset=0;
int prevIndex=0;         // for starting the loop from that index which was found at min distance from car in the previous loop
double yawRate=0;            // rate of change of theta(0)
double errorRate=0;     // rate of 
double sec0=0,sec1=0;    //to take that sampling time in account
vector<geometry_msgs::Pose> incomePath;  // Create a vector of path which we aer taking by subscribing to geometry_msgs Pose
std_msgs::Float64 steer;
double minD;
int minindex=0;
path_point pointSearch(){
	path_point temp,goal;	
	//<<incomePath.size()<<endl;
	////<<"prevIndex entered in the function: "<<index<<endl;

		int index=0;
		temp.px=incomePath[0].position.x;
		////<<"x coord: "<<temp.px<<endl;
		temp.py=incomePath[0].position.y;
		////<<"y coord: "<<temp.py<<endl;
		temp.d=sqrt(pow(x-temp.px,2)+pow(y-temp.py,2));
		////<<"distance from car: "<<temp.d<<endl;
		minD=temp.d;
		////<<"finding closest point"<<endl;

		for(int i=prevIndex;i<incomePath.size();i++)

			{
				if(sqrt(pow(x-incomePath[i].position.x,2)+pow(y-incomePath[i].position.y,2))<minD)
				{
					index=i;
					minD=sqrt(pow(x-incomePath[i].position.x,2)+pow(y-incomePath[i].position.y,2));
						////<<"if me chala 1212"<<endl;

				}
			}
		////<<"got closest point"<<endl;
		goal.index=index;
		prevIndex=index;
		
		cout<<"Index found: "<<goal.index<<endl;
		goal.d=55*minD;
		cout<<"cross track error: "<<goal.d<<endl;
		msg.data=goal.d;    //for rosbag file

		goal.px=incomePath[index].position.x;
		goal.py=incomePath[index].position.y;	
		goal.slope=atan2(incomePath[index+1].position.y-goal.py,incomePath[index+1].position.x-goal.px);
		////<<"calculated slope"<<endl;
		if((x*goal.py-y*goal.px)>0)
			direction=-1;
		else
			direction=1;
		prevIndex=goal.index;
		goal.d*=direction;//
		//cout<<"Direction : "<<direction<<endl;
	
	
	return goal;
}

void posCallBack(const nav_msgs::Odometry::ConstPtr &msg){
	double wheelBase=1.98;
	x=msg->pose.pose.position.x;
	y=msg->pose.pose.position.y;
	double q1= msg->pose.pose.orientation.x;
	double q2= msg->pose.pose.orientation.y;
	double q3= msg->pose.pose.orientation.z;
	double q4= msg->pose.pose.orientation.w;
	double siny=2.0*(q4*q3+q1*q2),cosy=1-2*(q2*q2);	
	ayaw=atan2(siny,cosy);
	////<<"got position and orientation"<<endl;
	ryaw0=ryaw1;
	////<<"prevIndex: "<<prevIndex<<endl;
	x=msg->pose.pose.position.x;
	y=msg->pose.pose.position.y;
	// x=x+0.5*(wheelBase)*cos(ayaw);
	// y=y+0.5*(wheelBase)*sin(ayaw);
	if(incomePath.size()>0)
		p=pointSearch();
	else{
		////<<"path yet not recieved"<<endl;
		return;
	}
	////<<"got the closest point"<<endl;
	ryaw0=atan2(p.slope,1)-ayaw;
	ryaw0=ryaw0*100;		
	ryawDiff=ryaw1-ryaw0;	
	
	error0=error1;
	error1=p.d;
	errorDiff=error1-error0;
	sec0=0;
	sec1=0.5;
	////<<"sec0: "<<sec0<<" sec1: "<<sec1<<endl;
	if(sec0==sec1){
		yawRate=0;
		errorRate=0;
	}
	else{
		yawRate=ryawDiff/(sec1-sec0);
		errorRate=errorDiff/(sec1-sec0);
	}
	////<<"ready to run LQR"<<endl;
	LQR();
}


void velCallBack(const nav_msgs::Odometry::ConstPtr msg){
vx=sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));
	}

void call(const nav_msgs::Path::ConstPtr msg){

	////<<"path liya "<<endl;
	incomePath.clear();
	// incomePath=*msg;
	int i=0;
	////<<"Incoming PATH"<<endl;
	for(i=0;i<msg->poses.size();i++){
		incomePath.push_back(msg->poses[i].pose);
		////<<msg->poses[i].pose.position.x<<endl;
	}

	void Ft()

		{

		}
	void Ct()
		{

		}
	void ct()
		{

		}
	void LQRbackwardPass()
		{

		}
	void forwardPass_on_real_non_linear_dynamics()
		{

		}
	void badi_matrix()
		{double (v*sampling*tan(delta))/L;
			C_badi<<	-1,0,-2L*cot(delta)*sin(phi/2)*cos(theta+phi/2),v*sampling*sin(rough+theta)*sec(delta)*sec(delta)*cot(delta)  - 2L*sin(rough/2)*sin(theta+(rough/2))*cosec(delta)*cosec(delta) ,sampling*sin(rough+theta)*cot(delta)*tan(delta),
				   	0,-1,-2L*cot(delta)*sin(phi/2)*sin(theta+phi/2),v*sampling*cos(rough+theta)*sec(delta)*sec(delta)*cot(delta)  - 2L*sin(rough/2)*cos(theta+(rough/2))*cosec(delta)*cosec(delta),sampling*cos(rough+theta)*cot(delta)*tan(delta),
				   	0,0,-1,-v*sec(delta)*sec(delta)*sampling/L,-tan(delta)/L*sampling;


		}

int main(int argc, char **argv)
{
	ros::init(argc ,argv, "pursuit");

	ros::NodeHandle n1;

	ros::Subscriber s1 = n1.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1,&posCallBack);

	ros::Subscriber s2 = n1.subscribe<nav_msgs::Path>("astroid_path",1,&call);

	ros::Publisher p = n1.advertise<std_msgs::Float64>("cmd_steer",10);
	
	ros::Subscriber sub = n1.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1,velCallBack);

	ros::Publisher bag_pub = n1.advertise<std_msgs::Float64>("Cross_track_bag",1);

	ros::Rate loop_rate(10);

	
	while(ros::ok())
	{		
		p.publish(steer);
		bag_pub.publish(msg);
		ros::spinOnce();
		
		loop_rate.sleep();
	}

	return 0;
}




