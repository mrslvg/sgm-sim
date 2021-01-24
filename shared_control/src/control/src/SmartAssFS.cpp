#include "SmartAssFS.h"

#include <iostream>
#include <cmath> 

SmartAssFS::SmartAssFS(float slopeTh, float nearbyTh, int reactTimeCloser, int reactTimeAway, int reactTimeSteady, float deltaForce, float maxStiff, float maxForce, bool continuative){
	this->slopeTh = slopeTh;
	this->nearbyTh = nearbyTh;
	this->reactTimeCloser = reactTimeCloser;
	this->reactTimeAway = reactTimeAway;
	this->reactTimeSteady = reactTimeSteady;
	this->deltaStiff = deltaForce;
	this->maxStiff = maxStiff;
	this->maxForce = maxForce;
	this->continuative = continuative;

	this->status = INIT;
	this->status_prev = INIT;
	this->count = 0;
	this->firstIt = true;

	this->d_prev = 0.0f;
	this->k_prev = 0.0f;

	this->unit = std::vector<float>();
	this->unit.push_back(0.0f);
	this->unit.push_back(0.0f);
	this->unit.push_back(0.0f);

	this->log = false;
}


SmartAssFS::~SmartAssFS()
{
}

std::vector<float> SmartAssFS::Run_Vector(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z)
{
	//----------INIT----------
	float f = 0.0f;
	float k = 0.0f;
	float d = this->GetDistance_EE2T(target_x, target_y, target_z, robot_x, robot_y, robot_z);

	if (d != 0.0f)
	{
		this->unit[0] = (target_x - robot_x) / d;
		this->unit[1] = (target_y - robot_y) / d;
		this->unit[2] = (target_z - robot_z) / d;
	}
	else
	{
		this->unit[0] = 0.0f;
		this->unit[1] = 0.0f;
		this->unit[2] = 0.0f;
	}

	//----------CHECK STATUS----------
	if (this->firstIt == true)
	{
		//FIRST ITERATION			
		this->status = INIT;
		this->firstIt = false;
	}
	else if (d <= this->nearbyTh)
	{
		//TARGET REACHED 
		this->status = REACHED;
	}
	else
	{
		float m = -1 * (d - d_prev);
		if (m > this->slopeTh)
		{
			//---------GETTING CLOSER---------
			this->status = CLOSER;
		}
		else if (m < -this->slopeTh)
		{
			//---------MOVING AWAY---------
			this->status = AWAY;
		}
		else
		{
			//---------STEADY---------
			this->status = STEADY;
		}
	}

	//----------EVALUATING COUNTER----------
	if (this->status_prev == INIT && this->status == STEADY)
	{
		this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == INIT && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == INIT && this->status == AWAY)
	{
		this->count = 0;
		if (this->count < this->reactTimeAway)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == INIT && this->status == CLOSER)
	{
		this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == STEADY && this->status == STEADY)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeSteady)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == STEADY && this->status == REACHED)
	{
		//this->count = -1;
	}
	else if (this->status_prev == STEADY && this->status == AWAY)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeAway)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == STEADY && this->status == CLOSER)
	{
		this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == REACHED && this->status == STEADY)
	{
		this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == REACHED && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == REACHED && this->status == AWAY)
	{
		this->count = 0;
		if (this->count < this->reactTimeAway)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == REACHED && this->status == CLOSER)
	{
		this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == AWAY && this->status == STEADY)
	{
		// WITH PAUSE
		/*this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else {
			this->count = -1;
		}*/

		//WITHOUT PAUSE
		if (this->count != -1)
		{
			if (this->count < this->reactTimeAway)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == AWAY && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == AWAY && this->status == AWAY)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeAway)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == AWAY && this->status == CLOSER)
	{
		this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == CLOSER && this->status == STEADY)
	{
		this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == CLOSER && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == CLOSER && this->status == AWAY)
	{
		this->count = 0;
		if (this->count < this->reactTimeAway)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == CLOSER && this->status == CLOSER)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeCloser)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}

	//----------EVALUATING STIFFNESS----------
	if (this->count == -1)
	{
		if (this->status == STEADY || this->status == AWAY)
		{
			k = IncrementStiffness();
		}
		else if (this->status == CLOSER || this->status == REACHED)
		{
			k = DecrementStiffness();
		}
		else
		{
			k = this->k_prev;
		}
	}
	else
	{
		k = this->k_prev;
	}

	//----------EVALUATING FORCE----------
	f = k * d;
	f = SaturateForce(f);
	std::vector<float> proxy = this->GetForceProxy(robot_x, robot_y, robot_z, f);

	//----------BACK PROPAGATION----------	
	this->status_prev = this->status;
	this->k_prev = k;
	this->d_prev = d;	

	//----------LOG----------
	if (this->log) {
		std::cout << std::endl << "---------------------------" << std::endl;
		std::cout << "Distance EE-T: " << d << std::endl;
		switch (this->status)
		{
		case INIT:
			std::cout << "Init" << std::endl;
			std::cout << "IT: " << this->count << std::endl;
			break;
		case REACHED:
			std::cout << "Reached" << std::endl;
			std::cout << "IT: " << this->count << std::endl;
			break;
		case STEADY:
			std::cout << "Steady" << std::endl;
			std::cout << "IT: " << this->count << "/" << this->reactTimeSteady << std::endl;
			break;
		case CLOSER:
			std::cout << "Getting Close" << std::endl;
			std::cout << "IT: " << this->count << "/" << this->reactTimeCloser << std::endl;
			break;
		case AWAY:
			std::cout << "Moving Away" << std::endl;
			std::cout << "IT: " << this->count << "/" << this->reactTimeAway << std::endl;
			break;
		}
		std::cout << "Stiffness: " << k*1000.0f << "/" << maxStiff*1000.0f << " [N/m]\t";
		std::cout << "Force: " << f << "/" << maxForce << " [N]" << std::endl;
		std::cout << "---------------------------" << std::endl << std::endl;
	}

	return proxy;
}

float SmartAssFS::Run_Magnitude(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z)
{
	//----------INIT----------
	float f = 0.0f;
	float k = 0.0f;
	float d = this->GetDistance_EE2T(target_x, target_y, target_z, robot_x, robot_y, robot_z);

	//----------CHECK STATUS----------
	if (this->firstIt == true)
	{
		//FIRST ITERATION			
		this->status = INIT;
		this->firstIt = false;
	}
	else if (d <= this->nearbyTh)
	{
		//TARGET REACHED 
		this->status = REACHED;
	}
	else
	{
		float m = -1 * (d - d_prev);
		if (m > this->slopeTh)
		{
			//---------GETTING CLOSER---------
			this->status = CLOSER;
		}
		else if (m < -this->slopeTh)
		{
			//---------MOVING AWAY---------
			this->status = AWAY;
		}
		else
		{
			//---------STEADY---------
			this->status = STEADY;
		}
	}

	//----------EVALUATING COUNTER----------
	if (this->status_prev == INIT && this->status == STEADY)
	{
		this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == INIT && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == INIT && this->status == AWAY)
	{
		this->count = 0;
		if (this->count < this->reactTimeAway)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == INIT && this->status == CLOSER)
	{
		this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == STEADY && this->status == STEADY)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeSteady)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == STEADY && this->status == REACHED)
	{
		//this->count = -1;
	}
	else if (this->status_prev == STEADY && this->status == AWAY)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeAway)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == STEADY && this->status == CLOSER)
	{
		if (continuative)
			this->count = countD_prev;
		else
			this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == REACHED && this->status == STEADY)
	{
		this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == REACHED && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == REACHED && this->status == AWAY)
	{
		this->count = 0;
		if (this->count < this->reactTimeAway)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == REACHED && this->status == CLOSER)
	{
		this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == AWAY && this->status == STEADY)
	{
		// WITH PAUSE
		/*this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else {
			this->count = -1;
		}*/

		//WITHOUT PAUSE
		if (this->count != -1)
		{
			if (this->count < this->reactTimeAway)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == AWAY && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == AWAY && this->status == AWAY)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeAway)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}
	else if (this->status_prev == AWAY && this->status == CLOSER)
	{
		if (continuative)
			this->count = countD_prev;
		else
			this->count = 0;
		if (this->count < this->reactTimeCloser)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == CLOSER && this->status == STEADY)
	{
		if (continuative)
			this->count = countI_prev;
		else
			this->count = 0;
		if (this->count < this->reactTimeSteady)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == CLOSER && this->status == REACHED)
	{
		this->count = -1;
	}
	else if (this->status_prev == CLOSER && this->status == AWAY)
	{
		if (continuative)
			this->count = countI_prev;
		else
			this->count = 0;
		if (this->count < this->reactTimeAway)
		{
			this->count++;
		}
		else
		{
			this->count = -1;
		}
	}
	else if (this->status_prev == CLOSER && this->status == CLOSER)
	{
		if (this->count != -1)
		{
			if (this->count < this->reactTimeCloser)
			{
				this->count++;
			}
			else
			{
				this->count = -1;
			}
		}
	}

	//----------EVALUATING STIFFNESS----------
	if (this->count == -1)
	{
		if (this->status == STEADY || this->status == AWAY)
		{
			k = IncrementStiffness();
			this->countI_prev = 0;
		}
		else if (this->status == CLOSER || this->status == REACHED)
		{
			k = DecrementStiffness();
			this->countD_prev = 0;
		}
		else
		{
			k = this->k_prev;
		}
	}
	else
	{
		k = this->k_prev;
	}

	//----------EVALUATING FORCE----------
	f = k * d;
	f = SaturateForce(f);

	//----------BACK PROPAGATION----------	
	this->status_prev = this->status;
	this->k_prev = k;
	this->d_prev = d;
	if (count != -1) {
		if (this->status == STEADY || this->status == AWAY)
		{
			this->countI_prev = count;
		}
		else if (this->status == CLOSER)
		{
			this->countD_prev = count;
		}			
	}

	//----------LOG----------
	if (this->log) {
		std::cout << std::endl << "---------------------------" << std::endl;
		std::cout << "Distance EE-T: " << d << std::endl;		
		switch (this->status)
		{
		case INIT:
			std::cout << "Init" << std::endl;
			std::cout << "IT: " << this->count << std::endl;
			break;
		case REACHED:
			std::cout << "Reached" << std::endl;
			std::cout << "IT: " << this->count << std::endl;
			break;
		case STEADY:
			std::cout << "Steady" << std::endl;
			std::cout << "IT: " << this->count << "/" << this->reactTimeSteady << std::endl;
			break;
		case CLOSER:
			std::cout << "Getting Close" << std::endl;
			std::cout << "IT: " << this->count << "/" << this->reactTimeCloser << std::endl;
			break;
		case AWAY:
			std::cout << "Moving Away" << std::endl;
			std::cout << "IT: " << this->count << "/" << this->reactTimeAway << std::endl;
			break;
		}
		std::cout << "Stiffness: " << k*1000.0f << "/" << maxStiff*1000.0f << " [N/m]\t";
		std::cout << "Force: " << f << "/" << maxForce << " [N]" << std::endl;
		std::cout << "---------------------------" << std::endl << std::endl;
	}

	return f;
}

void SmartAssFS::LogOn()
{
	this->log = true;
}

void SmartAssFS::LogOff()
{
	this->log = false;
}

void SmartAssFS::Continuative(bool s)
{
	this->continuative = s;
}

void SmartAssFS::Set_SlopeThreshold(float slopeTh) {
	this->slopeTh = slopeTh;
}

float SmartAssFS::Get_SlopeThreshold() {
	return this->slopeTh;
}

void SmartAssFS::Set_NearbyThreshold(float nearbyTh) {
	this->nearbyTh = nearbyTh;
}

float SmartAssFS::Get_NearbyThreshold() {
	return this->nearbyTh;
}

void SmartAssFS::Set_ReactionTime_Closer(int reactTimeCloser) {
	this->reactTimeCloser = reactTimeCloser;
}

int SmartAssFS::Get_ReactionTime_Closer() {
	return this->reactTimeCloser;
}

void SmartAssFS::Set_ReactionTime_Away(int reactTimeAway) {
	this->reactTimeAway = reactTimeAway;
}

int SmartAssFS::Get_ReactionTime_Away() {
	return this->reactTimeAway;
}

void SmartAssFS::Set_ReactionTime_Steady(int reactTimeSteady) {
	this->reactTimeSteady = reactTimeSteady;
}

int SmartAssFS::Get_ReactionTime_Steady() {
	return this->reactTimeSteady;
}

void SmartAssFS::Set_DeltaStiffness(float deltaStiff) {
	this->deltaStiff = deltaStiff;
}

float SmartAssFS::Get_DeltaStiffness() {
	return this->deltaStiff;
}

void SmartAssFS::Set_MaxStiffness(float maxStiff) {
	this->maxStiff = maxStiff;
}

float SmartAssFS::Get_MaxStiffness() {
	return this->maxStiff;
}

void SmartAssFS::Set_MaxForce(float maxForce) {
	this->maxForce = maxForce;
}

float SmartAssFS::Get_MaxForce() {
	return this->maxForce;
}

std::vector<float> SmartAssFS::GetForceProxy(float robot_x, float robot_y, float robot_z, float f)
{
	std::vector<float> proxy = { 0.0f, 0.0f, 0.0f };
	proxy[0] = (this->unit[0] * f);
	proxy[1] = (this->unit[1] * f);
	proxy[2] = (this->unit[2] * f);
	return proxy;
}

float SmartAssFS::GetDistance_EE2T(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z)
{
	float a = (target_x - robot_x);
	float b = (target_y - robot_y);
	float c = (target_z - robot_z);
	return sqrtf((a * a) + (b * b) + (c * c));
}

float SmartAssFS::IncrementStiffness()
{
	float k = this->k_prev + this->deltaStiff;
	k = this->SaturateStiffness(k);
	return k;
}

float SmartAssFS::DecrementStiffness()
{
	float f = this->k_prev - this->deltaStiff;
	f = this->SaturateStiffness(f);
	return f;
}

float SmartAssFS::SaturateStiffness(float k)
{
	if (k > this->maxStiff)
		k = this->maxStiff;
	if (k < 0)
		k = 0.0f;
	return k;
}

float SmartAssFS::SaturateForce(float f)
{
	if (f > this->maxForce)
		f = this->maxForce;
	if (f < 0)
		f = 0.0f;
	return f;
}
