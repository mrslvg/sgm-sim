template<typename T, typename Container = std::deque<T> >
class iterable_queue : public std::queue<T, Container>
{
public:
	typedef typename Container::iterator iterator;
	typedef typename Container::const_iterator const_iterator;

	iterator begin() { return this->c.begin(); }
	iterator end() { return this->c.end(); }
	const_iterator begin() const { return this->c.begin(); }
	const_iterator end() const { return this->c.end(); }
};

iterable_queue<double> qForce;

void initFilter(int f = 50)
{
	for (int i = 0; i < f; i++)
		qForce.push(0.0);
}

double filterForce(double value, int f = 50)
{
	qForce.push(value);
	if (qForce.size() > f)
		qForce.pop();
	double result = 0.0;
	for (auto it = qForce.begin(); it != qForce.end(); ++it)
		result += *it;
	result /= qForce.size();
	return result;
}

double runFixedAssistance(double robotEE_x, double robotEE_y, double robotEE_z, double currentGoal_x, double currentGoal_y, double currentGoal_z)
{
	double a = (currentGoal_x - robotEE_x);
	double b = (currentGoal_y - robotEE_y);
	double c = (currentGoal_z - robotEE_z);
	double d = sqrt((a * a) + (b * b) + (c * c));	//Distance End Effector - Current Goal
	double k = 10.0 / 1000.0;							//Stiffness fixed at 10N/mm - valid for 100Hz refresh
	if (d < 30.0)
		k = 0.0;										//If distance is below 30mm put stiffness = 0
	double f = min(d * k, 7.0);					//Max force = 7N
	f = filterForce(f, 50);								//Filter Over 50 samples	
	return f;
}

int main()
{
	initFilter(50);	//Filter over 50 samples
	//---at each loop
	double force = runFixedAssistance(eex,eey,eez,cgx,cgy,cgz);
}