#pragma once
#include <vector>

class SmartAssF
{

	float slopeTh;          //slope threshold
	float nearbyTh;         //thershold of target reached    
	int reactTimeCloser;    //reaction time for 'getting close' case
	int reactTimeAway;      //reaction time for 'moving away' case
	int reactTimeSteady;    //reaction time for 'steady' case
	float deltaForce;       //unit of increment for each iteration (abs value)	
	float maxForce;			//max force before saturation

	int movement;           //current movement --> 0 = steady, 1 = closer, -1 = away
	int count;              //iteration counter for reaction    
			
	int it;                 //iteration number

	std::vector<float> unit;	//Unit vector towards the target direction

	std::vector<float> D;    //List of distances EE-Target
	std::vector<float> F;    //List of forces produces by the algorithm

	bool debug;				//If true, prints the status of the algorithm

public:
	SmartAssF();
	SmartAssF(float slopeTh, float nearbyTh, int reactTimeCloser, int reactTimeAway, int reactTimeSteady, float deltaForce, float maxForce);
	~SmartAssF();
	
	std::vector<float> Run_Vector(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z);
	float Run_Magnitude(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z);

	void DebugOn();
	void DebugOff();

	 

private:

	std::vector<float> GetForceProxy(float robot_x, float robot_y, float robot_z, float f);
	float GetDistance_EE2T(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z);	
	float IncrementForce(float f_prev);
	float DecrementForce(float f_prev);
	float PosSaturation(float f);
	float Manage_SteadyOrNoOperator(float f_prev);
	float Manage_MovingAway(float f_prev);
	float Manage_GettingCloser(float f_prev);
};

