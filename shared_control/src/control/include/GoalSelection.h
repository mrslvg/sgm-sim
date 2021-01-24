#pragma once

#define START 0
#define SELECT_ITEM 1
#define GRASPING_PHASE 2
#define LIFTING_ITEM_PHASE 3
#define PLACING_PHASE 4
#define LIFTING_NO_ITEM_PHASE 5
#define END 6

class Goal2
{
	float pos[3];
	bool active;

public:
	Goal2();
	~Goal2();
	Goal2(float x, float y, float z);
	void SetActive(bool a);
	bool GetActive();
	void X(int x);
	float X();
	void Y(int y);
	float Y();
	void Z(int z);
	float Z();
};

class GoalSelection2
{
	//ITEM the item to be picked up or placed
	//TARGET the final position where to put an item
	//GOAL the current objective, which could be an item if the system is reaching an item, or a target if the item was already picked and the system is reaching for a target

	/********************************STATES********************************/
	int state_curr;			//Current State
	int state_prev;			//Previous State
	int item_curr;			//Current Item to be picked up or placed

	/********************************COORDINATES********************************/
	Goal2 item1;				//Object for item 1
	Goal2 item2;				//Object for item 2

	Goal2 target1;			//Object for target 1
	Goal2 target2;			//Object for target 2

	float ee[3];			//Current position of the end effector
	float goal[3];			//Position of the current goal
		
	/********************************THRESHOLDS********************************/	
	float z_offset;					//Goes over the target of this length in mm
	float bending_z_offset;			//Length of the robot from the base to the bending point, in mm	

	/********************************LOG********************************/
	bool log;						//If true prints the log

public:
	GoalSelection2();
	~GoalSelection2();
	void Reset(float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z, float t1_x, float t1_y, float t1_z, float t2_x, float t2_y, float t2_z);
	void Run(float ee_x, float ee_y, float ee_z, bool itemGrasped, bool eversionIsAutomated);
	void GetGoal(float &x, float &y, float &z);		
	void LogOn();
	void LogOff();
	void SetZOffset(float zo);
	void SetBendingZOffset(float zo);

private:
	bool SelectGoalFromItems();
	void SetGoalToItem();
	void SetGoalToOver();
	void SetGoalToTarget();	
	void SetGoalToEndEffector(float ee_x, float ee_y, float ee_z);
	bool CheckRobotIsLifted();	
	bool ChangeState(int newState);
	void WriteLog();
};

