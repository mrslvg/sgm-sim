#include "CoppeliaWrapper.h"

using namespace std;

CoppeliaWrapper::CoppeliaWrapper()
{

}

bool CoppeliaWrapper::connect(bool startSimulation)
{
    // CONNECT TO COPPELIA
    simxFinish(-1);
    clientID_ = simxStart("127.0.0.1", 19997, 1, 1, 5000, 5);
    if(clientID_ != -1)
    {
        printf("Connected to Coppelia Sim \n");
    }
    else
    {
        printf("Impossible to connect to Coppelia Sim \n");
        return false;
    }

    if(startSimulation)
    {
        if(simxStartSimulation(clientID_, simx_opmode_blocking) != 0)
        {
            printf("Impossible to start simulation \n");
            return false;
        }
    }
    retrieveHandles();
    return true;
}

bool CoppeliaWrapper::retrieveHandles()
{
    string filename = "./../src/coppelia/config/handles.txt";
    std::cout << "Reading handles from " << filename << " file" << std::endl;
    ifstream file(filename.c_str());
    string line, handle_name;
    handles_.clear();

    if(file.is_open())
    {
        while (getline(file, handle_name)) {
            istringstream ss(line);
            ss >> handle_name;
            simxInt handle;
            simxInt returnCode = simxGetObjectHandle(clientID_, handle_name.c_str(), &handle, simx_opmode_blocking);

            if(returnCode == 0)
            {
                handles_[handle_name.c_str()] = handle;
                cout << handle_name << " handle: " << handle << endl;
            }
            else
            {
                cout << "Impossible to retrieve" << handle_name << " handle" << endl;
                cout << "Return code: " << returnCode << endl;
                return 0;
            }
        }
        std::cout<< "Loading handles completed!" << std::endl << std::endl;
        return true;
    }
    else
    {
        std::cout<< "[ERROR] cannot open the file " << filename << std::endl;
        return false;
    }
}

simxInt CoppeliaWrapper::clientID() const
{
    return clientID_;
}

simxInt CoppeliaWrapper::getObjectHandle(std::string name) const
{
    std::map<std::string, simxInt>::const_iterator it = handles_.find(name);
    if(it != handles_.end())
    {
        std::cout << name << " handle found!"<<std::endl;
        return it->second;
    }
    else
    {
        std::cout << name << " handle not found!"<<std::endl;
    }
    return 0;
}

bool CoppeliaWrapper::getObjectPosition(simxInt object_handle, simxInt relative_to_object_handle, float position[])
{
    simxInt returnCode = simxGetObjectPosition(clientID_,
                                               object_handle,
                                               relative_to_object_handle,
                                               position,
                                               simx_opmode_blocking);
    if(returnCode == 0)
    {
        cout << "Position of the object successfully retrieved: ";
        for(int i = 0; i < 3; i++){cout << position[i] << " ";} cout << endl;
        return 1;
    }
    else
    {
        cout << "Impossible to retrieve object position" << endl;
        cout << "Return code: " << returnCode << endl;
        return 0;
    }
}
