#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include "fstream"
#include "string"

#include "extApi.h"
#include "extApiPlatform.h"


class CoppeliaWrapper
{

public:
    CoppeliaWrapper();
    bool connect(bool startSimulation);
    simxInt clientID() const;
    simxInt getObjectHandle(std::string name) const;
    bool getObjectPosition(simxInt object_handle, simxInt relative_to_object_handle, float position[3]);

private:
    bool retrieveHandles();
    simxInt clientID_;
    std::string paramFile_;
    std::map<std::string, simxInt> handles_;
};



