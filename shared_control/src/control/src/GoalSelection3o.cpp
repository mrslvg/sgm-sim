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
	item3 = Goal(0.0f, 0.0f, 0.0f);
	target1 = Goal(0.0f, 0.0f, 0.0f);
	target2 = Goal(0.0f, 0.0f, 0.0f);
	target3 = Goal(0.0f, 0.0f, 0.0f);

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

	order = DIST_123;
	firstIteration = true;

	WriteLog();
}

GoalSelection::~GoalSelection()
{
}

void GoalSelection::Reset(float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z, float i3_x, float i3_y, float i3_z, float t1_x, float t1_y, float t1_z, float t2_x, float t2_y, float t2_z, float t3_x, float t3_y, float t3_z, float reachedThreshold)
{
	state_curr = SELECT_ITEM;
	state_prev = state_curr;

	item1 = Goal(i1_x, i1_y, i1_z);
	item2 = Goal(i2_x, i2_y, i2_z);
	item3 = Goal(i3_x, i3_y, i3_z);
	target1 = Goal(t1_x, t1_y, t1_z);
	target2 = Goal(t2_x, t2_y, t2_z);
	target3 = Goal(t3_x, t3_y, t3_z);

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

	order = DIST_123;
	firstIteration = true;

	WriteLog();
}

void GoalSelection::Run_Complex(float ee_x, float ee_y, float ee_z, bool gripperIsOpen, float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z, float i3_x, float i3_y, float i3_z)
{
	ee[0] = ee_x;
	ee[1] = ee_y;
	ee[2] = ee_z;

	this->gripperState_isOpen = gripperIsOpen;

	item1.X(i1_x); item1.Y(i1_y); item1.Z(i1_z);
	item2.X(i2_x); item2.Y(i2_y); item2.Z(i2_z);
	item3.X(i3_x); item3.Y(i3_y); item3.Z(i3_z);

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
			//if (SelectGoalFromItems_Greedy())
			if (SelectGoalFromItems_NotGreedy())
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
			gripperCommand_Open = true;
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
					reiterate = ChangeState(GO_OVER_ITEM);
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
			SetGoalToOver();
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
					case 3:
						item3.SetActive(false);
						break;
					case 0:
						break;
					}
					reiterate = ChangeState(GO_OVER_TARGET);
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
			gripperCommand_Open = true;
			SetGoalToOver();
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
			gripperCommand_Open = true;
			reiterate = ChangeState(SELECT_ITEM);
			break;
		case END:
			gripperCommand_Open = true;
			reiterate = ChangeState(END);
			break;
		}

		WriteLog();
	} while (reiterate);
}

void GoalSelection::Run_Simple(float ee_x, float ee_y, float ee_z, bool gripperIsOpen, float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z, float i3_x, float i3_y, float i3_z)
{
	ee[0] = ee_x;
	ee[1] = ee_y;
	ee[2] = ee_z;

	this->gripperState_isOpen = gripperIsOpen;

	item1.X(i1_x); item1.Y(i1_y); item1.Z(i1_z);
	item2.X(i2_x); item2.Y(i2_y); item2.Z(i2_z);
	item3.X(i3_x); item3.Y(i3_y); item3.Z(i3_z);

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
			//if (SelectGoalFromItems_Greedy())
			if (SelectGoalFromItems_NotGreedy())
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
			gripperCommand_Open = true;
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
					SetGoalToTarget();
					reiterate = ChangeState(GO_TO_TARGET);
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
					case 3:
						item3.SetActive(false);
						break;
					case 0:
						break;
					}
					reiterate = ChangeState(SELECT_ITEM);
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
			gripperCommand_Open = true;
			reiterate = ChangeState(END);
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

/********************************PRIVATE METHODS********************************/

/**
 * Returns true if an item was selected, false if there is no item left
 */
bool GoalSelection::SelectGoalFromItems_Greedy()
{
	float dist = 0.0f;
	int item = 0;
	float minDist = 0.0f;
	bool result = false;

	if (item1.GetActive() == true)
	{
		dist = EDist(ee, item1);
		if (item == 0 || dist < minDist)
		{
			minDist = dist;
			item = 1;
		}
	}
	if (item2.GetActive() == true)
	{
		dist = EDist(ee, item2);
		if (item == 0 || dist < minDist)
		{
			minDist = dist;
			item = 2;
		}
	}
	if (item3.GetActive() == true)
	{
		dist = EDist(ee, item3);
		if (item == 0 || dist < minDist)
		{
			minDist = dist;
			item = 3;
		}
	}

	switch (item)
	{
	case 0:
		result = false;
		item_curr = 0;
		break;
	case 1:
		goal[0] = item1.X();
		goal[1] = item1.Y();
		goal[2] = item1.Z();
		result = true;
		item_curr = 1;
		break;
	case 2:
		goal[0] = item2.X();
		goal[1] = item2.Y();
		goal[2] = item2.Z();
		result = true;
		item_curr = 2;
		break;
	case 3:
		goal[0] = item3.X();
		goal[1] = item3.Y();
		goal[2] = item3.Z();
		result = true;
		item_curr = 3;
		break;
	}
	return result;
}

/**
 * Returns true if an item was selected, false if there is no item left
 */
bool GoalSelection::SelectGoalFromItems_NotGreedy()
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
	case 3:
		if (item3.GetActive()) {
			return true;
		}
		break;
	}	

	switch (order)
	{
		
	case DIST_123:
		switch (item_curr)
		{
		case 0:
			item_curr = 1;
			break;
		case 1:
			item_curr = 2;
			break;
		case 2:
			item_curr = 3;
			break;
		case 3:
			result = false;
			break;
		}
		break;
	case DIST_132:
		switch (item_curr)
		{
		case 0:
			item_curr = 1;
			break;
		case 1:
			item_curr = 3;
			break;
		case 3:
			item_curr = 2;
			break;
		case 2:
			result = false;
			break;
		}
		break;
	case DIST_213:
		switch (item_curr)
		{
		case 0:
			item_curr = 2;
			break;
		case 2:
			item_curr = 1;
			break;
		case 1:
			item_curr = 3;
			break;
		case 3:
			result = false;
			break;
		}
		break;
	case DIST_231:
		switch (item_curr)
		{
		case 0:
			item_curr = 2;
			break;
		case 2:
			item_curr = 3;
			break;
		case 3:
			item_curr = 1;
			break;
		case 1:
			result = false;
			break;
		}
		break;
	case DIST_312:
		switch (item_curr)
		{
		case 0:
			item_curr = 3;
			break;
		case 3:
			item_curr = 1;
			break;
		case 1:
			item_curr = 2;
			break;
		case 2:
			result = false;
			break;
		}
		break;
	case DIST_321:
		switch (item_curr)
		{
		case 0:
			item_curr = 3;
			break;
		case 3:
			item_curr = 2;
			break;
		case 2:
			item_curr = 1;
			break;
		case 1:
			result = false;
			break;
		}
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
	case 3:
		goal[0] = item3.X();
		goal[1] = item3.Y();
		goal[2] = item3.Z();
		break;
	}
}

void GoalSelection::SetGoalToOver()
{
	goal[0] = ee[0];
	goal[1] = ee[1];
	goal[2] = target1.Z() - z_offset;
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
	case 3:
		goal[0] = target3.X();
		goal[1] = target3.Y();
		goal[2] = target3.Z();
		break;
	}
}

/**
 * Returns true the end effector is close the goal, false otherwise
 */
bool GoalSelection::Check_GoalIsReached()
{
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

float GoalSelection::EDist(float p1[3], float p2[3])
{
	return sqrtf(((p1[0] - p2[0]) * (p1[0] - p2[0])) + ((p1[1] - p2[1]) * (p1[1] - p2[1])) + ((p1[2] - p2[2]) * (p1[2] - p2[2])));
}

float GoalSelection::EDist(Goal g1, Goal g2)
{
	return (float)sqrtf(((g1.X() - g2.X()) * (g1.X() - g2.X())) + ((g1.Y() - g2.Y()) * (g1.Y() - g2.Y())) + ((g1.Z() - g2.Z()) * (g1.Z() - g2.Z())));
}

float GoalSelection::EDist(float p[3], Goal g)
{
	return sqrtf(((p[0] - g.X()) * (p[0] - g.X())) + ((p[1] - g.Y()) * (p[1] - g.Y())) + ((p[2] - g.Z()) * (p[2] - g.Z())));
}

void GoalSelection::GetMinDistance_NotGeeedy()
{
	float dist_eei1 = EDist(ee, item1);
	float dist_eei2 = EDist(ee, item2);
	float dist_eei3 = EDist(ee, item3);

	float dist_i1t1 = EDist(item1, target1);
	float dist_i2t2 = EDist(item2, target2);
	float dist_i3t3 = EDist(item3, target3);

	float dist_t1i2 = EDist(target1, item2);
	float dist_t1i3 = EDist(target1, item3);
	float dist_t2i1 = EDist(target2, item1);
	float dist_t2i3 = EDist(target2, item3);
	float dist_t3i1 = EDist(target3, item1);
	float dist_t3i2 = EDist(target3, item2);

	float dists[6];
	dists[0] = dist_eei1 + dist_i1t1 + dist_t1i2 + dist_i2t2 + dist_t2i3 + dist_i3t3;     //0 -> 123
	dists[1] = dist_eei1 + dist_i1t1 + dist_t1i3 + dist_i3t3 + dist_t3i2 + dist_i2t2;     //1 -> 132
	dists[2] = dist_eei2 + dist_i2t2 + dist_t2i1 + dist_i1t1 + dist_t1i3 + dist_i3t3;     //2 -> 213
	dists[3] = dist_eei2 + dist_i2t2 + dist_t2i3 + dist_i3t3 + dist_t3i1 + dist_i1t1;     //3 -> 231
	dists[4] = dist_eei3 + dist_i3t3 + dist_t3i1 + dist_i1t1 + dist_t1i2 + dist_i2t2;     //4 -> 312
	dists[5] = dist_eei3 + dist_i3t3 + dist_t3i2 + dist_i2t2 + dist_t2i1 + dist_i1t1;     //5 -> 321

	float minDist = 0.0f;
	order = 0;
	for (int i = 0; i < 6; i++)
	{
		if (i == 0)
		{
			minDist = dists[0];
			order = 0;
		}
		else if (dists[i] < minDist)
		{
			minDist = dists[i];
			order = i;
		}
	}
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
		}

		std::cout << "CURRENT GOAL < " << goal[0] << " , " << goal[1] << " , " << goal[2] << " >\t";
		std::cout << "EE < " << ee[0] << " , " << ee[1] << " , " << ee[2] << " >\n";
		
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
		std::cout << "-------------------------------\n";
	}	
}
