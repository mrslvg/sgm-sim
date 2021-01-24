#include "GoalSelection.h"

#include <iostream>
#include <cmath>

Goal2::Goal2() {
	pos[0] = 0.0f;
	pos[1] = 0.0f;
	pos[2] = 0.0f;
	active = true;
}

Goal2::~Goal2() {

}

Goal2::Goal2(float x, float y, float z) {
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	active = true;
}

void Goal2::SetActive(bool a) {
	active = a;
}

bool Goal2::GetActive() {
	return active;
}

void Goal2::X(int x) {
	pos[0] = x;
}

float Goal2::X() {
	return pos[0];
}

void Goal2::Y(int y) {
	pos[1] = y;
}

float Goal2::Y() {
	return pos[1];
}

void Goal2::Z(int z) {
	pos[2] = z;
}

float Goal2::Z() {
	return pos[2];
}

GoalSelection2::GoalSelection2()
{
	state_curr = START;
	state_prev = state_curr;
	item1 = Goal2(0.0f, 0.0f, 0.0f);
	item2 = Goal2(0.0f, 0.0f, 0.0f);
	target1 = Goal2(0.0f, 0.0f, 0.0f);
	target2 = Goal2(0.0f, 0.0f, 0.0f);
	ee[0] = 0.0f;
	ee[1] = 0.0f;
	ee[2] = 0.0f;
	goal[0] = 0.0f;
	goal[1] = 0.0f;
	goal[2] = 0.0f;
	item_curr = 0;
	z_offset = 100.0f;
	bending_z_offset = 0.0f;
}


GoalSelection2::~GoalSelection2()
{
}

void GoalSelection2::Reset(float i1_x, float i1_y, float i1_z, float i2_x, float i2_y, float i2_z, float t1_x, float t1_y, float t1_z, float t2_x, float t2_y, float t2_z)
{
	state_curr = START;
	state_prev = state_curr;
	item1 = Goal2(i1_x, i1_y, i1_z);
	item2 = Goal2(i2_x, i2_y, i2_z);
	target1 = Goal2(t1_x, t1_y, t1_z);
	target2 = Goal2(t2_x, t2_y, t2_z);
	ee[0] = 0.0f;
	ee[1] = 0.0f;
	ee[2] = 0.0f;
	goal[0] = 0.0f;
	goal[1] = 0.0f;
	goal[2] = 0.0f;
	item_curr = 0;
	WriteLog();
}

void GoalSelection2::Run(float ee_x, float ee_y, float ee_z, bool itemGrasped, bool eversionIsAutomated)
{
	ee[0] = ee_x;
	ee[1] = ee_y;
	ee[2] = ee_z;

	bool reiterate = false;
	do
	{
		switch (state_curr)
		{
		case START:
			reiterate = ChangeState(SELECT_ITEM);
			break;
		case SELECT_ITEM:
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
			if (SelectGoalFromItems())
				reiterate = ChangeState(GRASPING_PHASE);
			else
				reiterate = ChangeState(END);
			break;
		case GRASPING_PHASE:
			if (state_prev != state_curr)
				SetGoalToItem();
			if (itemGrasped)
			{
				if(eversionIsAutomated)
					reiterate = ChangeState(LIFTING_ITEM_PHASE);
				else
					reiterate = ChangeState(PLACING_PHASE);					
			}
			else
				reiterate = ChangeState(GRASPING_PHASE);
			break;
		case LIFTING_ITEM_PHASE:
			if (state_prev != state_curr)
				SetGoalToOver();
			if (CheckRobotIsLifted())
				reiterate = ChangeState(PLACING_PHASE);
			else
				reiterate = ChangeState(LIFTING_ITEM_PHASE);
			break;
		case PLACING_PHASE:
			if (state_prev != state_curr)
				SetGoalToTarget();
			if (!itemGrasped)
			{
				if (eversionIsAutomated)
					reiterate = ChangeState(LIFTING_NO_ITEM_PHASE);
				else
					reiterate = ChangeState(SELECT_ITEM);
			}
			else
				reiterate = ChangeState(PLACING_PHASE);
			break;
		case LIFTING_NO_ITEM_PHASE:
			if (state_prev != state_curr)
				SetGoalToOver();
			if (CheckRobotIsLifted())
				reiterate = ChangeState(SELECT_ITEM);
			else
				reiterate = ChangeState(LIFTING_NO_ITEM_PHASE);
			break;
		case END:
			SetGoalToEndEffector(ee[0], ee[1], ee[2]);
			reiterate = ChangeState(END);
			break;
		}
		WriteLog();
	} while (reiterate);
}

void GoalSelection2::GetGoal(float & x, float & y, float & z)
{
	x = goal[0];
	y = goal[1];
	z = goal[2];
}

void GoalSelection2::LogOn()
{
	log = true;
}

void GoalSelection2::LogOff()
{
	log = false;
}

void GoalSelection2::SetZOffset(float zo)
{
	z_offset = zo;
}

void GoalSelection2::SetBendingZOffset(float zo)
{
	bending_z_offset = zo;
}

/********************************PRIVATE METHODS********************************/

bool GoalSelection2::SelectGoalFromItems()
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

void GoalSelection2::SetGoalToItem()
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

void GoalSelection2::SetGoalToOver()
{
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
}

void GoalSelection2::SetGoalToTarget()
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
}

void GoalSelection2::SetGoalToEndEffector(float ee_x, float ee_y, float ee_z)
{
	goal[0] = ee_x;
	goal[1] = ee_y;
	goal[2] = ee_z;
}

bool GoalSelection2::CheckRobotIsLifted()
{
	if (ee[2] <= 550.0f)
		return true;
	else
		return false;
}

bool GoalSelection2::ChangeState(int newState)
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

void GoalSelection2::WriteLog()
{
	if (log) {

		std::cout << "STATE: ";
		switch (state_curr)
		{
		case START:
			std::cout << "Start " << " <-- ";
			break;
		case SELECT_ITEM:
			std::cout << "Select item " << " <-- ";
			break;
		case GRASPING_PHASE:
			std::cout << "Grasping Phase, Item " << item_curr << " <-- ";
			break;
		case LIFTING_ITEM_PHASE:
			std::cout << "Lifting Phase, Item " << item_curr << " <-- ";
			break;
		case PLACING_PHASE:
			std::cout << "Placing Phase, Item " << item_curr << " <-- ";
			break;		
		case LIFTING_NO_ITEM_PHASE:
			std::cout << "Lifting Phase, No Item " << " <-- ";
			break;
		case END:
			std::cout << "Task Completed\n";
			break;
		}

		switch (state_prev)
		{
		case START:
			std::cout << "Start " << " <-- ";
			break;
		case SELECT_ITEM:
			std::cout << "Select item\n";
			break;
		case GRASPING_PHASE:
			std::cout << "Grasping Phase\n";
			break;
		case LIFTING_ITEM_PHASE:
			std::cout << "Lifting Phase\n";
			break;
		case PLACING_PHASE:
			std::cout << "Placing Phase\n";
			break;
		case LIFTING_NO_ITEM_PHASE:
			std::cout << "Lifting Phase\n";
			break;
		case END:
			std::cout << "Task Completed\n";
			break;
		}

		std::cout << "CURRENT GOAL < " << goal[0] << " , " << goal[1] << " , " << goal[2] << " >\t";
		std::cout << "EE < " << ee[0] << " , " << ee[1] << " , " << ee[2] << " >\n";

		std::cout << "-------------------------------\n";
	}
}
