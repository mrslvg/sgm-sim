#pragma once
#include <vector>

#define INIT -1
#define REACHED 0
#define STEADY 1
#define AWAY 2
#define CLOSER 3

class SmartAssFS
{
	float slopeTh;          //slope threshold
	float nearbyTh;         //thershold of target reached    
	int reactTimeCloser;    //reaction time for 'getting close' case
	int reactTimeAway;      //reaction time for 'moving away' case
	int reactTimeSteady;    //reaction time for 'steady' case
	
	float deltaStiff;       //unit of increment for each iteration (abs value)	
	float maxStiff;			//max stiffness before saturation
	float maxForce;			//max force before saturation

	int status;				//current status  -1 = init, 0 = reached, 1 = steady, 2 = away, 3 = close
	int status_prev;		//previous status

	int count;              //iteration counter for reaction  
	int countI_prev;		//previous value of counter for increment before be set to -1
	int countD_prev;		//previous value of counter for decrement before be set to -1
	bool firstIt;			//Flag for fitst iteration

	std::vector<float> unit;	//Unit vector towards the target direction

	float d_prev;			//Previous distance
	float k_prev;			//Previous stiffness

	bool log;				//If true, prints the status of the algorithm
	bool continuative;		//Whenever the status goes from increment to decrement (or viceversa):
							//- if false the counter starts back from zero
							//- if true the counter restart from where it was before

public:
	SmartAssFS(float slopeTh, float nearbyTh, int reactTimeCloser, int reactTimeAway, int reactTimeSteady, float deltaForce, float maxStiff, float maxForce, bool continuative);
	~SmartAssFS();
	std::vector<float> Run_Vector(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z);
	float Run_Magnitude(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z);

	void LogOn();
	void LogOff();
	void Continuative(bool s);

	void Set_SlopeThreshold(float slopeTh);
	float Get_SlopeThreshold();
	void Set_NearbyThreshold(float nearbyTh);
	float Get_NearbyThreshold();
	void Set_ReactionTime_Closer(int reactTimeCloser);
	int Get_ReactionTime_Closer();
	void Set_ReactionTime_Away(int reactTimeAway);
	int Get_ReactionTime_Away();
	void Set_ReactionTime_Steady(int reactTimeSteady);
	int Get_ReactionTime_Steady();
	void Set_DeltaStiffness(float deltaStiff);
	float Get_DeltaStiffness();
	void Set_MaxStiffness(float maxStiff);
	float Get_MaxStiffness();
	void Set_MaxForce(float maxForce);
	float Get_MaxForce();

private:
	std::vector<float> GetForceProxy(float robot_x, float robot_y, float robot_z, float f);
	float GetDistance_EE2T(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z);
	float IncrementStiffness();
	float DecrementStiffness();
	float SaturateStiffness(float k);
	float SaturateForce(float f);
};

