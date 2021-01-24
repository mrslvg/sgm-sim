#pragma once

#define SELECT_ITEM 0
#define GO_TO_ITEM 1
#define ITEM_REACHED 2
#define GO_OVER_ITEM 3
#define OVER_ITEM_REACHED 4
#define GO_TO_TARGET 5
#define TARGET_REACHED 6
#define GO_OVER_TARGET 7
#define OVER_TARGET_REACHED 8
#define END 9
#define HOLD_ON_GOAL 10

class Goal
{
	float pos[3];
	bool active;

	public:
		Goal();
		~Goal();
		Goal(float x, float y, float z);
		void SetActive(bool a);
		bool GetActive();
		void X(int x);
		float X();
		void Y(int y);
		float Y();
		void Z(int z);
		float Z();
};




class GoalSelection
{
	//ITEM the item to be picked up or placed
	//TARGET the final position where to put an item
	//GOAL the current objective, which could be an item if the system is reaching an item, or a target if the item was already picked and the system is reaching for a target

	/********************************STATES********************************/
	int state_curr;			//Current State
	int state_prev;			//Previous State
	int item_curr;			//Current Item to be picked up or placed

	/********************************COORDINATES********************************/
	Goal item1;				//Object for item 1
	Goal item2;				//Object for item 2

	Goal target1;			//Object for target 1
	Goal target2;			//Object for target 2

	float ee[3];			//Current position of the end effector
	float goal[3];			//Position of the current goal

	/********************************ROBOT********************************/
	bool gripperState_isOpen;				//If true, the gripper is open, if false is closed
	bool gripperCommand_Open;		//For automated gripper command outside the method, when sends true the gripper should open, when false the gripper should close

	/********************************THRESHOLDS********************************/
	float reachedThreshold;			//Threshold that identifies when the end effector is has reached the goal, in mm
	float z_offset;					//Goes over the target of this length in mm
	float bending_z_offset;			//Length of the robot from the base to the bending point, in mm

	/********************************HOLD ON GOAL********************************/
	int hold_count;
	int hold_maxCount;
	int hold_nextState;
	float hold_goal[3];

	/********************************SEARCH ALGORITHM********************************/
	bool firstIteration;			//If true, the system is at its first iteration

	/********************************LOG********************************/
	bool log;						//If true prints the log

public:
	GoalSelection();
	~GoalSelection();
	void Reset(float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z, float t1_x, float t1_y, float t1_z, float t2_x, float t2_y, float t2_z, float reachedThreshold, int timeOfWaitingOnTarget);
	void Run_Complex(float ee_x, float ee_y, float ee_z, bool gripperIsOpen, float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z);
	void Run_Simple(float ee_x, float ee_y, float ee_z, bool gripperIsOpen, float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z);
	void GetGoal(float &x, float &y, float &z);
	bool GetGripperCommand();
	void SetReachingThreshold(float th);
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
	bool Check_GoalIsReached();
	bool Check_Over();
	float SignedEDist(float ee[3], float g[3]);
	float EDist(float p1[3], float p2[3]);
	bool ChangeState(int newState);
	void WriteLog();
};

