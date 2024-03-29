#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "tku_libs/WalkContinuouse.h"
#include "strategy/basketballinfo.h"
#include "strategy/loadparameter.h"

using namespace std;

class KidsizeStrategy
{

public:
	KidsizeStrategy() 
	{
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
		walk_con = WalkContinuouseInstance::getInstance();
		sendbodystandflag = false;
		DIOSTRATAGAIN = true;
	};
	~KidsizeStrategy()
	{
		StrategyInfoInstance::deleteInstance();
		ToolInstance::deleteInstance();
		RosCommunicationInstance::deleteInstance();
		WalkContinuouseInstance::deleteInstance();
	};

	StrategyInfoInstance *strategy_info;
	ToolInstance *tool;
	RosCommunicationInstance *ros_com;
	WalkContinuouseInstance *walk_con;

	void strategymain();
	void InversePerspective();
	void Triangulation();
	void image();
	void MoveHead(HeadMotorID , int , int );
	void ComputeSpeed();
	void SelectBaseLine();
	void FindballInitial();
	void FindballHead();
	void TraceballHead();
	void TraceballBody();
	void FindbasketHead();
	void TracebasketHead();
	void TracebasketBody();
	void Draw();
	void MoveContinuous(int );
	void AreaSizeMeasure();
	
	//LayUp
	void UPbasket();
	void SlamDunk();
	
	typedef enum{
		//BB
		//Catch Ball
		BB_WaistDown		= 118,
		BB_WaistCatch       = 1110,
		BB_WaistUp          = 1111,
		BB_WaistUpFeedBack  = 1112,
		//Shooting
		BB_RaiseHand		= 1115,
		BB_ShootingBall		= 1116,
		BB_KeepBall		    = 1117,
		//LayUp
		BB_UpHand           = 1140,
		BB_SlamDunk         = 1150,
		//Stand
		BB_StandFix         = 1218,

		Robot_StandUp		= 29	
		
	}MotionTable;

	SensorMode IMUSet;
	WalkingMode walkingmode;
	bool sendbodystandflag;
	bool DIOSTRATAGAIN;
	struct timeval tstart, tend;
	double timeuse;

};
