#include "HapticDevice.h"

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

//Haptic device constructor:
HapticDevice::HapticDevice()
{
    for(unsigned int i = 0; i < 3; i++)
        m_f[i] = 0.0f;
}	

HapticDevice::~HapticDevice()
{
    // close the connection
    printf ("cleaning up...\n");

    m_HapticDevice.close();

    // happily exit
    printf ("\ndone.\n");
}

int HapticDevice::close()
{
    // close the connection
    printf ("cleaning up...\n");

    m_HapticDevice.close();

    // happily exit
    printf ("\ndone.\n");

    return 0;
}

void HapticDevice::getDeviceVelocity(double velocity[3])
{
    for(unsigned int i = 0; i < 3; i++)
        velocity[i] = m_v[i];
}

void HapticDevice::getDeviceForce(double force[3])
{
    for(unsigned int i = 0; i < 3; i++)
        force[i] = m_f[i];
}

void HapticDevice::getDevicePosition(double position[3], double deadzone_size)
{
    m_p[2] = m_p[2] - 0.12;
    for(unsigned int i = 0; i < 3; i++)
    {
        m_p[i] = (fabs (m_p[i]) > deadzone_size) ? (m_p[i]>0.0) ? m_p[i] - deadzone_size : m_p[i] + deadzone_size : 0.0;
        position[i] = m_p[i];
    }
}

void HapticDevice::getDeviceButton(unsigned int& button)
{
    m_HapticDevice.runIOLoop();
    button = m_HapticDevice.getFalconGrip().get()->getDigitalInputs();
}
						
int HapticDevice::initialize()
{
    if(init_falcon(0))
    {
        printf("Falcon Initialised\n");
        m_HapticDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
    }
    else
    {
        printf("Impossible to initialise Falcon\n");
        return -1;
    }

    return 0;
}

void HapticDevice::update()
{
    m_HapticDevice.runIOLoop();
    m_p = m_HapticDevice.getPosition();
    for(unsigned int i = 0; i < 3; i++)
        m_v[i] = (m_p[i] - m_p_prev[i])/0.001; // fix rate

    m_p_prev = m_p;
}

void HapticDevice::setDeviceForce(std::array<double, 3> force)
{
    m_HapticDevice.setForce(force);
}

bool HapticDevice::init_falcon(int NoFalcon)
{
    std::cout << "Setting up LibUSB" << endl;

    m_HapticDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware
    std::shared_ptr<FalconFirmware> firmware = m_HapticDevice.getFalconFirmware();
    firmware->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE));
    m_HapticDevice.setFalconGrip<FalconGripFourButton>(); //Set Grip
    if(!m_HapticDevice.open(NoFalcon)) //Open falcon @ index
    {
        cout << "Failed to find falcon" << endl;
        return false;
    }
    else
    {
        cout << "Falcon Found" << endl;
    }

    // There's only one kind of firmware right now, so automatically set that.
    m_HapticDevice.setFalconFirmware<FalconFirmwareNovintSDK>();
    // Next load the firmware to the device

    bool skip_checksum = false;
    //See if we have firmware
    bool firmware_loaded = false;
    firmware_loaded = m_HapticDevice.isFirmwareLoaded();
    if(!firmware_loaded)
    {
        cout << "Loading firmware" << endl;
        uint8_t* firmware_block;
        long firmware_size;
        {
            firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
            firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

            for(int i = 0; i < 20; ++i)
            {
                if(!m_HapticDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

                {
                    cout << "Firmware loading try failed" <<endl;
                }
                else
                {
                    firmware_loaded = true;
                    break;
                }
            }
        }
    }
    else if(!firmware_loaded)
    {
        cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << endl;
        return false;
    }
    if(!firmware_loaded || !m_HapticDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue" << endl;
        return false;
    }
    cout << "Firmware loaded" << endl;

    m_HapticDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set" << endl;

    std::array<int, 3> forces = {0, 0, 0};
    m_HapticDevice.getFalconFirmware()->setForces(forces);
    m_HapticDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    bool homing_reset = false;
    usleep(1000);
    int tryLoad = 0;
    while(!stop ) //&& tryLoad < 100)
    {
        if(!m_HapticDevice.runIOLoop()) continue;
        if(!m_HapticDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_HapticDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in." << endl;

            }
            homing = true;
        }

        if(homing && m_HapticDevice.getFalconFirmware()->isHomed())
        {
            m_HapticDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
            cout << "Falcon homed." << endl;
            homing_reset = true;
            stop = true;
        }
        tryLoad++;
    }
    /*
    if(tryLoad >= 100)
    {
        return false;
    }
    */
    m_HapticDevice.runIOLoop();

    return true;
}
