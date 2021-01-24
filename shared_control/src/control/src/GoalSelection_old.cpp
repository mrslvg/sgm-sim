#include "GoalSelection.h"

#include <iostream>
#include <cmath>

Goal::Goal() {
	pos[0] = 0.0f;
	pos[1] = 0.0f;
	pos[2] = 0.0f;
	active = true;
}

Goal::~Goal() {

}

Goal::Goal(float x, float y, float z) {
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	active = true;
}

void Goal::SetActive(bool a) {
	active = a;
}

bool Goal::GetActive() {
	return active;
}

void Goal::X(int x) {
	pos[0] = x;
}

float Goal::X() {
	return pos[0];
}

void Goal::Y(int y) {
	pos[1] = y;
}

float Goal::Y() {
	return pos[1];
}

void Goal::Z(int z) {
	pos[2] = z;
}

float Goal::Z() {
	return pos[2];
}

GoalSelection::GoalSelection()
{
	state_curr = SELECT_ITEM;
	state_prev = state_curr;

	item1 = Goal(0.0f, 0.0f, 0.0f);
	item2 = Goal(0.0f, 0.0f, 0.0f);
	target1 = Goal(0.0f, 0.0f, 0.0f);
	target2 = Goal(0.0f, 0.0f, 0.0f);

	ee[0] = 0.0f;
	ee[1] = 0.0f;
	ee[2] = 0.0f;
	goal[0] = 0.0f;
	goal[1] = 0.0f;
	goal[2] = 0.0f;

	gripperState_isOpen = true;
	gripperCommand_Open = true;

	reachedThreshold = 100.0f;

	item_curr = 0;	

	z_offset = 100.0f;
	bending_z_offset = 0.0f;

	firstIteration = true;

	hold_count = 0;
	hold_maxCount = 0;
	hold_nextState = END;
	hold_goal[0] = 0.0f;
	hold_goal[1] = 0.0f;
	hold_goal[2] = 0.0f;
}

GoalSelection::~GoalSelection()
{
}

void GoalSelection::Reset(float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z, float t1_x, float t1_y, float t1_z, float t2_x, float t2_y, float t2_z, float reachedThreshold, int timeOfWaitingOnTarget)
{
	state_curr = SELECT_ITEM;
	state_prev = state_curr;

	item1 = Goal(i1_x, i1_y, i1_z);
	item2 = Goal(i2_x, i2_y, i2_z);
	target1 = Goal(t1_x, t1_y, t1_z);
	target2 = Goal(t2_x, t2_y, t2_z);

	ee[0] = 0.0f;
	ee[1] = 0.0f;
	ee[2] = 0.0f;
	goal[0] = 0.0f;
	goal[1] = 0.0f;
	goal[2] = 0.0f;
	gripperState_isOpen = true;
	gripperCommand_Open = true;

	this->reachedThreshold = reachedThreshold;
	item_curr = 0;	

	firstIteration = true;

	hold_count = 0;
	hold_maxCount = timeOfWaitingOnTarget;
	hold_nextState = END;	
	hold_goal[0] = 0.0f;
	hold_goal[1] = 0.0f;
	hold_goal[2] = 0.0f;

	WriteLog();
}

void GoalSelection::Run_Complex(float ee_x, float ee_y, float ee_z, bool gripperIsOpen, float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z)
{
	ee[0] = ee_x;
	ee[1] = ee_y;
	ee[2] = ee_z;

	this->gripperState_isOpen = gripperIsOpen;

	item1.X(i1_x); item1.Y(i1_y); item1.Z(i1_z);
	item2.X(i2_x); item2.Y(i2_y); item2.Z(i2_z);

	if (firstIteration)
	{
		state_curr = SELECT_ITEM;
		state_prev = SELECT_ITEM;
		firstIteration = false;
	}

	bool reiterate = false;
	do
	{
		switch (state_curr)
		{
		case SELECT_ITEM:
			gripperCommand_Open = false;
			if (SelectGoalFromItems())
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			else
			{
				reiterate = ChangeState(END);
			}
			break;
		case GO_TO_ITEM:
			SetGoalToItem();
			gripperCommand_Open = false;
			if (Check_GoalIsReached())
			{
				reiterate = ChangeState(ITEM_REACHED);
			}
			else
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			break;
		case ITEM_REACHED:
			gripperCommand_Open = false;
			if (Check_GoalIsReached())
			{
				if (!gripperIsOpen)
				{
					/*reiterate = ChangeState(GO_OVER_ITEM);
					SetGoalToOver();*/
					hold_count = 0;
					hold_nextState = GO_OVER_ITEM;
					reiterate = ChangeState(HOLD_ON_GOAL);
					SetGoalToOver();
					//--
				}
				else
				{
					reiterate = ChangeState(ITEM_REACHED);
				}
			}
			else
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			break;
		case GO_OVER_ITEM:
			gripperCommand_Open = false;			
			if (gripperIsOpen)
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			else
			{
				if (Check_Over())
				{
					reiterate = ChangeState(OVER_ITEM_REACHED);
				}
				else
				{
					reiterate = ChangeState(GO_OVER_ITEM);
				}
			}
			break;
		case OVER_ITEM_REACHED:
			gripperCommand_Open = false;
			if (gripperIsOpen)
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			else
			{
				SetGoalToTarget();
				reiterate = ChangeState(GO_TO_TARGET);
			}
			break;
		case GO_TO_TARGET:
			gripperCommand_Open = false;
			if (gripperIsOpen)
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			else
			{
				if (Check_GoalIsReached())
				{
					reiterate = ChangeState(TARGET_REACHED);
				}
				else
				{
					reiterate = ChangeState(GO_TO_TARGET);
				}
			}
			break;
		case TARGET_REACHED:
			gripperCommand_Open = true;
			if (Check_GoalIsReached())
			{
				if (gripperIsOpen)
				{
					switch (item_curr)
					{
					case 1:
						item1.SetActive(false);
						break;
					case 2:
						item2.SetActive(false);
						break;
					case 0:
						break;
					}
					/*reiterate = ChangeState(GO_OVER_TARGET);
					SetGoalToOver();*/
					hold_count = 0;
					hold_nextState = GO_OVER_TARGET;
					reiterate = ChangeState(HOLD_ON_GOAL);
					SetGoalToOver();
					//---
				}
				else
				{
					reiterate = ChangeState(TARGET_REACHED);
				}
			}
			else
			{
				reiterate = ChangeState(GO_TO_TARGET);
			}
			break;
		case GO_OVER_TARGET:
			gripperCommand_Open = false;
			if (Check_Over())
			{
				reiterate = ChangeState(OVER_TARGET_REACHED);
			}
			else
			{
				reiterate = ChangeState(GO_OVER_TARGET);
			}
			break;
		case OVER_TARGET_REACHED:
			gripperCommand_Open = false;
			reiterate = ChangeState(SELECT_ITEM);
			break;
		case END:
			SetGoalToEndEffector(ee[0], ee[1], ee[2]);
			gripperCommand_Open = false;
			reiterate = ChangeState(END);
			break;
		case HOLD_ON_GOAL:
			if (hold_count == 0) {
				goal[0] = ee[0];
				goal[1] = ee[1];
				goal[2] = ee[2];
			}
			if (hold_count < hold_maxCount)
			{	
				hold_count++;
				reiterate = false;				
			}
			else
			{
				reiterate = ChangeState(hold_nextState);
				hold_count = 0;
				goal[0] = hold_goal[0];
				goal[1] = hold_goal[1];
				goal[2] = hold_goal[2];
			}
			break;
		}

		WriteLog();
	} while (reiterate);
}

void GoalSelection::Run_Simple(float ee_x, float ee_y, float ee_z, bool gripperIsOpen, float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z)
{
	ee[0] = ee_x;
	ee[1] = ee_y;
	ee[2] = ee_z;

	this->gripperState_isOpen = gripperIsOpen;

	item1.X(i1_x); item1.Y(i1_y); item1.Z(i1_z);
	item2.X(i2_x); item2.Y(i2_y); item2.Z(i2_z);

	if (firstIteration)
	{
		state_curr = SELECT_ITEM;
		state_prev = SELECT_ITEM;
		//GetMinDistance_NotGeeedy();
		firstIteration = false;
	}

	bool reiterate = false;
	do
	{
		switch (state_curr)
		{
		case SELECT_ITEM:
			gripperCommand_Open = false;
			if (SelectGoalFromItems())
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			else
			{
				reiterate = ChangeState(END);
			}
			break;
		case GO_TO_ITEM:
			SetGoalToItem();
			gripperCommand_Open = false;
			if (Check_GoalIsReached())
			{
				reiterate = ChangeState(ITEM_REACHED);
			}
			else
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			break;
		case ITEM_REACHED:
			gripperCommand_Open = false;
			if (Check_GoalIsReached())
			{
				if (!gripperIsOpen)
				{					
					/*reiterate = ChangeState(GO_TO_TARGET);
					SetGoalToTarget();*/
					hold_count = 0;
					hold_nextState = GO_TO_TARGET;
					reiterate = ChangeState(HOLD_ON_GOAL);
					SetGoalToTarget();
					//---
				}
				else
				{
					reiterate = ChangeState(ITEM_REACHED);
				}
			}
			else
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			break;
		case GO_TO_TARGET:
			gripperCommand_Open = false;
			if (gripperIsOpen)
			{
				reiterate = ChangeState(GO_TO_ITEM);
			}
			else
			{
				if (Check_GoalIsReached())
				{
					reiterate = ChangeState(TARGET_REACHED);
				}
				else
				{
					reiterate = ChangeState(GO_TO_TARGET);
				}
			}
			break;
		case TARGET_REACHED:
			gripperCommand_Open = true;
			if (Check_GoalIsReached())
			{
				if (gripperIsOpen)
				{
					switch (item_curr)
					{
					case 1:
						item1.SetActive(false);
						break;
					case 2:
						item2.SetActive(false);
						break;
					case 0:
						break;
					}
					//reiterate = ChangeState(SELECT_ITEM);
					hold_count = 0;
					hold_nextState = SELECT_ITEM;
					reiterate = ChangeState(HOLD_ON_GOAL);
					//---
					break;
				}
				else
				{
					reiterate = ChangeState(TARGET_REACHED);
				}
			}
			else
			{
				reiterate = ChangeState(GO_TO_TARGET);
			}
			break;
		case END:
			SetGoalToEndEffector(ee[0], ee[1], ee[2]);
			gripperCommand_Open = false;
			reiterate = ChangeState(END);
			break;
		case HOLD_ON_GOAL:
			if (hold_count == 0) {
				goal[0] = ee[0];
				goal[1] = ee[1];
				goal[2] = ee[2];
			}
			if (hold_count < hold_maxCount)
			{
				hold_count++;
				reiterate = false;
			}
			else
			{
				reiterate = ChangeState(hold_nextState);
				hold_count = 0;
				goal[0] = hold_goal[0];
				goal[1] = hold_goal[1];
				goal[2] = hold_goal[2];
			}
			break;
		}

		WriteLog();
	} while (reiterate);
}


void GoalSelection::GetGoal(float & x, float & y, float & z)
{
	x = goal[0];
	y = goal[1];
	z = goal[2];
}

/**
 * Returns true if the grupper is supposed to be open, false if the gripper is supposed to be closed
 */
bool GoalSelection::GetGripperCommand()
{
	return gripperCommand_Open;
}

void GoalSelection::SetReachingThreshold(float th)
{
	reachedThreshold = th;
}

void GoalSelection::LogOn()
{
	log = true;
}

void GoalSelection::LogOff()
{
	log = false;
}

void GoalSelection::SetZOffset(float zo)
{
	z_offset = zo;
}

void GoalSelection::SetBendingZOffset(float zo)
{
	bending_z_offset = zo;
}

/********************************PRIVATE METHODS********************************/


/**
 * Returns true if an item was selected, false if there is no item left
 */
bool GoalSelection::SelectGoalFromItems()
{
	bool result = true;
	
	switch (item_curr)
	{
	case 0:
		
		break;
	case 1:
		if (item1.GetActive()) {
			return true;
		}
		break;
	case 2:
		if (item2.GetActive()) {
			return true;
		}
		break;
	}	

	switch (item_curr)
	{
	case 0:
		item_curr = 1;
		break;
	case 1:
		item_curr = 2;
		break;
	case 2:
		result = false;
		break;
	}	
	return result;
}

void GoalSelection::SetGoalToItem()
{
	switch (item_curr)
	{
	case 0:
		break;
	case 1:
		goal[0] = item1.X();
		goal[1] = item1.Y();
		goal[2] = item1.Z();
		break;
	case 2:
		goal[0] = item2.X();
		goal[1] = item2.Y();
		goal[2] = item2.Z();
		break;
	}
}

void GoalSelection::SetGoalToOver()
{
	//goal[0] = ee[0];
	//goal[1] = ee[1];
	//goal[2] = target1.Z() - z_offset;


	float l1 = sqrtf((ee[0] * ee[0]) + (ee[1] * ee[1]) + ((ee[2] - bending_z_offset) * (ee[2] - bending_z_offset)));
	float a = sqrtf((ee[0] * ee[0]) + (ee[1] * ee[1]));
	float alpha = asin(a / l1);
	float beta = (3.14f / 2.0f) - alpha;
	float l2 = 0.0f;
	if (beta != 0)
		l2 = z_offset / (sin(beta));
	float l3 = l1 - l2;

	float u[3];
	u[0] = ee[0] / l1;
	u[1] = ee[1] / l1;
	u[2] = (ee[2] - bending_z_offset) / l1;

	goal[0] = l3 * u[0];
	goal[1] = l3 * u[1];
	goal[2] = l3 * u[2] + bending_z_offset;

	hold_goal[0] = goal[0];
	hold_goal[1] = goal[1];
	hold_goal[2] = goal[2];

}

void GoalSelection::SetGoalToTarget()
{
	switch (item_curr)
	{
	case 0:
		break;
	case 1:
		goal[0] = target1.X();
		goal[1] = target1.Y();
		goal[2] = target1.Z();
		break;
	case 2:
		goal[0] = target2.X();
		goal[1] = target2.Y();
		goal[2] = target2.Z();
		break;
	}
	hold_goal[0] = goal[0];
	hold_goal[1] = goal[1];
	hold_goal[2] = goal[2];
}

void GoalSelection::SetGoalToEndEffector(float ee_x, float ee_y, float ee_z)
{
	goal[0] = ee_x;
	goal[1] = ee_y;
	goal[2] = ee_z;
}

/**
 * Returns true the end effector is close the goal, false otherwise
 */
bool GoalSelection::Check_GoalIsReached()
{
	//float dist = SignedEDist(ee, goal);
	//if (-reachedThreshold <= dist && dist <= reachedThreshold)
	//	return true;
	//else
	//	return false;

	float dist = EDist(ee, goal);
	if (dist <= reachedThreshold)
		return true;
	else
		return false;
}

/**
 * Returns true the end effector is over the goal in the z axis, false otherwise
 */
bool GoalSelection::Check_Over()
{
	if (ee[2] <= goal[2])
		return true;
	else
		return false;
}

float GoalSelection::SignedEDist(float ee[3], float g[3])
{
	float c = sqrtf(ee[0] * ee[0] + ee[1] * ee[1] + ee[2] * ee[2]);
	float d = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);

	return d - c;
}

float GoalSelection::EDist(float p1[3], float p2[3])
{
	return sqrtf(((p1[0] - p2[0]) * (p1[0] - p2[0])) + ((p1[1] - p2[1]) * (p1[1] - p2[1])) + ((p1[2] - p2[2]) * (p1[2] - p2[2])));
}

bool GoalSelection::ChangeState(int newState)
{
	bool reiterate;
	if (state_curr != newState)
		reiterate = true;
	else
		reiterate = false;
	state_prev = state_curr;
	state_curr = newState;
	return reiterate;
}

void GoalSelection::WriteLog()
{
	if (log) {
		
		std::cout << "STATE: ";
		switch (state_curr)
		{
		case SELECT_ITEM:
			std::cout << "Select item " << " <-- ";
			break;
		case GO_TO_ITEM:
			std::cout << "Go to item " << item_curr << " <-- ";
			break;
		case ITEM_REACHED:
			std::cout << "Reached item " << item_curr << " <-- ";
			break;
		case GO_OVER_ITEM:
			std::cout << "Go over item " << item_curr << " <-- ";
			break;
		case OVER_ITEM_REACHED:
			std::cout << "Reached over item " << item_curr << " <-- ";
			break;
		case GO_TO_TARGET:
			std::cout << "Go to target " << item_curr << " <-- ";
			break;
		case TARGET_REACHED:
			std::cout << "Reached target " << item_curr << " <-- ";
			break;
		case GO_OVER_TARGET:
			std::cout << "Go over target " << item_curr << " <-- ";
			break;
		case OVER_TARGET_REACHED:
			std::cout << "Reached over target " << item_curr << " <-- ";
			break;
		case END:
			std::cout << "Task Completed\n";
			break;
		case HOLD_ON_GOAL:
			std::cout << "Hold on goal <-- ";
			break;
		}

		switch (state_prev)
		{
		case SELECT_ITEM:
			std::cout << "Select item \n";
			break;
		case GO_TO_ITEM:
			std::cout << "Go to item \n";
			break;
		case ITEM_REACHED:
			std::cout << "Reached item \n";
			break;
		case GO_OVER_ITEM:
			std::cout << "Go over item \n";
			break;
		case OVER_ITEM_REACHED:
			std::cout << "Reached over item \n";
			break;
		case GO_TO_TARGET:
			std::cout << "Go to target \n";
			break;
		case TARGET_REACHED:
			std::cout << "Reached target \n";
			break;
		case GO_OVER_TARGET:
			std::cout << "Go over target \n";
			break;
		case OVER_TARGET_REACHED:
			std::cout << "Reached over target \n";
			break;
		case END:
			std::cout << "Task Completed\n";
			break;
		case HOLD_ON_GOAL:
			std::cout << "Hold on goal\n";
			break;
		}

		std::cout << "CURRENT GOAL < " << goal[0] << " , " << goal[1] << " , " << goal[2] << " >\t";
		std::cout << "EE < " << ee[0] << " , " << ee[1] << " , " << ee[2] << " >\n";

		//float d = SignedEDist(ee, goal);
		float d = EDist(ee, goal);
		std::cout << "Dist " << d << "\n";
		
		std::cout << "GRIPPER STATE: ";
		if (gripperState_isOpen)
			std::cout << "open\t";
		else
			std::cout << "closed\t";
		
		std::cout << "GRIPPER COMMAND OUT: ";
		if (gripperCommand_Open)
			std::cout << "open\n" << std::endl;
		else
			std::cout << "closed\n" << std::endl;

		std::cout << "HOLD: " << hold_count << "/" << hold_maxCount << "\n";

		std::cout << "-------------------------------\n";
	}	
}
