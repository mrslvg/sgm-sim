#ifndef FALCON_DEVICE
#define FALCON_DEVICE

#include <stdlib.h>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

class HapticDevice{

public:

    HapticDevice();
    ~HapticDevice();
    void update();
    int initialize();
    int close();
    void getDevicePosition(double position[3], double deadzone_size);
    void getDeviceVelocity(double velocity[3]);
    void getDeviceButton(unsigned int& button);
    void getDeviceForce(double force[3]);
    void setDeviceForce(std::array<double, 3> force);

private:

    bool init_falcon(int nr);
    FalconDevice m_HapticDevice;
    std::array<double, 3> m_p, m_p_prev, m_v;
    double m_f[3], m_g;

};


#endif
