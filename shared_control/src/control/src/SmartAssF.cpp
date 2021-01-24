// #include "pch.h"
#include "SmartAssF.h"

#include <iostream>
#include <cmath> 

SmartAssF::SmartAssF()
{
	this->slopeTh = 4.0f;
	this->nearbyTh = 10.0f;
	this->reactTimeCloser = 5;
	this->reactTimeAway = 1;
	this->reactTimeSteady = 5;
	this->deltaForce = 1.0f;
	this->maxForce = 30.0f;

	this->movement = -2;
	this->count = 0;	
	this->it = 0;

	this->D = std::vector<float>();
	this->F = std::vector<float>();

	this->unit = std::vector<float>();
	this->unit.push_back(0.0f);
	this->unit.push_back(0.0f);
	this->unit.push_back(0.0f);

	this->debug = false;
}

SmartAssF::SmartAssF(float slopeTh, float nearbyTh, int reactTimeCloser, int reactTimeAway, int reactTimeSteady, float deltaForce, float maxForce)
{
	this->slopeTh = slopeTh;
	this->nearbyTh = nearbyTh;
	this->reactTimeCloser = reactTimeCloser;
	this->reactTimeAway = reactTimeAway;
	this->reactTimeSteady = reactTimeSteady;
	this->deltaForce = deltaForce;
	this->maxForce = maxForce;

	this->movement = -2;
	this->count = 0;	
	this->it = 0;

	this->D = std::vector<float>();
	this->F = std::vector<float>();

	this->unit = std::vector<float>();
	this->unit.push_back(0.0f);
	this->unit.push_back(0.0f);
	this->unit.push_back(0.0f);

	this->debug = false;
}


SmartAssF::~SmartAssF()
{
}

std::vector<float> SmartAssF::Run_Vector(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z)
{
	//----------INIT----------
	float f = 0.0f;
	float f_prev = 0.0f;
	if (this->it != 0)
	{
		f_prev = F[F.size() - 1];
	}
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

	//----------CORE----------
	this->D.push_back(d);
	if (this->it == 0)
	{
		//FIRST ITERATION		
		this->F.push_back(f);
		this->movement = -2;	//CHANGE <--0
		this->count = -1;
	}
	else if (d <= this->nearbyTh)
	{
		//TARGET REACHED 
		f = this->DecrementForce(f_prev);
		F.push_back(f);
	}
	else
	{
		float m = -1 * (this->D[it] - this->D[it - 1]);       // <---------------------------------TODO   
		if (m > this->slopeTh)
		{
			//---------GETTING CLOSER---------
			f = this->Manage_GettingCloser(f_prev);
			this->F.push_back(f);
		}
		else if (m < -this->slopeTh)
		{
			//---------MOVING AWAY---------
			f = this->Manage_MovingAway(f_prev);
			this->F.push_back(f);
		}
		else
		{
			//---------STEADY---------
			f = this->Manage_SteadyOrNoOperator(f_prev);
			this->F.push_back(f);
		}
	}

	std::vector<float> proxy = this->GetForceProxy(robot_x, robot_y, robot_z, f);


	//----------FINAL STATEMENTS----------	
	this->it++;

	if (this->debug) {
		std::cout << std::endl << "---------------------------" << std::endl;
		std::cout << "Distance EE-T: " << d;
		if (d <= this->nearbyTh)
			std::cout << " Target Reached! ";
		std::cout << std::endl;
		switch (movement)
		{
		case 0:
			std::cout << "Steady" << std::endl;
			std::cout << "IT: " << it << "--> " << count << "/" << reactTimeSteady << std::endl;
			break;
		case 1:
			std::cout << "Getting Close" << std::endl;
			std::cout << "IT: " << it << "--> " << count << "/" << reactTimeCloser << std::endl;
			break;
		case -1:
			std::cout << "Moving Away" << std::endl;
			std::cout << "IT: " << it << "--> " << count << "/" << reactTimeAway << std::endl;
			break;
		default:
			std::cout << "Init" << std::endl;
			std::cout << "IT: " << it << "--> " << count << std::endl;
			break;
		}
		std::cout << "Force: " << F[F.size() - 1] << "/" << maxForce << std::endl;
		std::cout << "---------------------------" << std::endl << std::endl;
	}

	return proxy;
}


float SmartAssF::Run_Magnitude(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z)
{
	//----------INIT----------
	float f = 0.0f;
	float f_prev = 0.0f;
	if (this->it != 0)
	{
		f_prev = F[F.size() - 1];
	}
	float d = this->GetDistance_EE2T(target_x, target_y, target_z, robot_x, robot_y, robot_z);
	
	//----------CORE----------
	this->D.push_back(d);
	if (this->it == 0)
	{
		//FIRST ITERATION		
		this->F.push_back(f);
		this->movement = -2;	//CHANGE <--0
		this->count = -1;
	}
	else if (d <= this->nearbyTh)
	{
		//TARGET REACHED 
		f = this->DecrementForce(f_prev);
		F.push_back(f);
	}
	else
	{
		float m = -1 * (this->D[it] - this->D[it - 1]);       // <---------------------------------TODO   
		if (m > this->slopeTh)
		{
			//---------GETTING CLOSER---------
			f = this->Manage_GettingCloser(f_prev);
			this->F.push_back(f);
		}
		else if (m < -this->slopeTh)
		{
			//---------MOVING AWAY---------
			f = this->Manage_MovingAway(f_prev);
			this->F.push_back(f);
		}
		else
		{
			//---------STEADY---------
			f = this->Manage_SteadyOrNoOperator(f_prev);
			this->F.push_back(f);
		}
	}
	
	//----------FINAL STATEMENTS----------	
	this->it++;

	if (this->debug) {
		std::cout << std::endl << "---------------------------" << std::endl;
		std::cout << "Distance EE-T: " << d;
		if (d <= this->nearbyTh)
			std::cout << " Target Reached! ";
		std::cout << std::endl;
		switch (movement)
		{
		case 0:
			std::cout << "Steady" << std::endl;
			std::cout << "IT: " << it << "--> " << count << "/" << reactTimeSteady << std::endl;
			break;
		case 1:
			std::cout << "Getting Close" << std::endl;
			std::cout << "IT: " << it << "--> " << count << "/" << reactTimeCloser << std::endl;
			break;
		case -1:
			std::cout << "Moving Away" << std::endl;
			std::cout << "IT: " << it << "--> " << count << "/" << reactTimeAway << std::endl;
			break;
		default:
			std::cout << "Init" << std::endl;
			std::cout << "IT: " << it << "--> " << count << std::endl;
			break;
		}
		std::cout << "Force: " << F[F.size() - 1] << "/" << maxForce << std::endl;
		std::cout << "---------------------------" << std::endl << std::endl;
	}

	return f;
}

void SmartAssF::DebugOn()
{
	this->debug = true;
}

void SmartAssF::DebugOff()
{
	this->debug = false;
}

std::vector<float> SmartAssF::GetForceProxy(float robot_x, float robot_y, float robot_z, float f)
{
	std::vector<float> proxy = { 0.0f, 0.0f, 0.0f };
	proxy[0] = (this->unit[0] * f);
	proxy[1] = (this->unit[1] * f);
	proxy[2] = (this->unit[2] * f);
	return proxy;
}

float SmartAssF::GetDistance_EE2T(float target_x, float target_y, float target_z, float robot_x, float robot_y, float robot_z)
{
	float a = (target_x - robot_x);
	float b = (target_y - robot_y);
	float c = (target_z - robot_z);
	return sqrtf((a * a) + (b * b) + (c * c));
}

float SmartAssF::IncrementForce(float f_prev)
{
	float f = f_prev + this->deltaForce;
	f = this->PosSaturation(f);
	return f;
}

float SmartAssF::DecrementForce(float f_prev)
{
	float f = f_prev - this->deltaForce;
	f = this->PosSaturation(f);
	return f;
}

float SmartAssF::PosSaturation(float f)
{
	if (f > this->maxForce)
		f = this->maxForce;
	if (f < 0)
		f = 0.0f;
	return f;
}

float SmartAssF::Manage_SteadyOrNoOperator(float f_prev)
{
	//---------STEADY-or-NO OPERATOR---------
	float result = f_prev;
	if (this->movement != 0)
	{
		this->count = 0;
	}
	if (this->count == -1)
	{
		//MODIFY FORCE AS LONG AS COUNT == -1
		result = this->IncrementForce(f_prev);
	}
	else
	{
		//KEEP FORCE CONSTANT AS LONG AS COUNT IS > -1 AND < REACT TIME	-1	
		if (this->count < this->reactTimeSteady - 1)
		{
			this->count++;
			//result = this->KeepForce(f_prev);
		}
		else
		{
			this->count = -1;
			result = this->IncrementForce(f_prev);
		}
	}
	this->movement = 0;
	return result;
}

float SmartAssF::Manage_MovingAway(float f_prev)
{
	//---------MOVING AWAY---------
	float result = f_prev;
	if (this->movement != -1)
	{
		this->count = 0;
	}
	if (this->count == -1)
	{
		//MODIFY FORCE AS LONG AS COUNT == -1
		result = this->IncrementForce(f_prev);
	}
	else
	{
		//KEEP FORCE CONSTANT AS LONG AS COUNT IS > -1 AND < REACT TIME -1

		if (this->count < this->reactTimeAway - 1)
		{
			//result = this->KeepForce(f_prev);
			this->count++;
		}
		else
		{
			this->count = -1;
			result = this->IncrementForce(f_prev);
		}
	}
	this->movement = -1;
	return result;
}

float SmartAssF::Manage_GettingCloser(float f_prev)
{
	//---------GETTING CLOSER---------
	float result = f_prev;
	if (this->movement != 1)
	{
		this->count = 0;
	}
	if (this->count == -1)
	{
		//MODIFY FORCE AS LONG AS COUNT == -1
		result = this->DecrementForce(f_prev);
	}
	else
	{
		//KEEP FORCE CONSTANT AS LONG AS COUNT IS > -1 AND < REACT TIME	-1	
		if (this->count <= this->reactTimeSteady - 1)
		{
			//result = this->KeepForce(f_prev);
			this->count++;
		}
		else
		{
			this->count = -1;
			result = this->DecrementForce(f_prev);
		}
	}
	this->movement = 1;
	return result;
}
