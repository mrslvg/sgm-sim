#include <fstream>
#include <string.h>

// read params from text file
inline bool read_params(std::string filename,
                        bool & use_haptic_device,
                        float & hd_deadzone,
                        float & hd_rate_gain,
                        float & hd_pos_gain,
                        float & kb_rate_gain,
                        float & cycle_time,
                        float & deltaStiffnessPerSecond,
                        int & reactTimeCloser_seconds,
                        int & reactTimeAway_seconds,
                        int & reactTimeSteady_seconds,
                        float & slopeTh,
                        float & nearbyTh,
                        float & maxStiff,
                        float & maxForce,
                        bool & teleoperation,
                        bool & targeting_assistance,
                        bool & targeting_fixed_assistance,
                        bool & auto_steering_manual_eversion,
                        bool & manual_steering_auto_eversion,
                        bool & full_autonomy,
                        bool & debug,
                        bool & save_data)
{
    std::cout << "Reading parameters from " << filename << " file" << std::endl;
    // load file
    std::ifstream file(filename.c_str());
    std::string line, param;

    float value;
    int check_file = 0;

    while (std::getline(file, line)) {

        std::istringstream ss(line);
        ss >> param >> value;

        if (param == "use_haptic_device") use_haptic_device = (bool)value;
        else if (param == "hd_deadzone") hd_deadzone = value;
        else if (param == "hd_rate_gain") hd_rate_gain = value;
        else if (param == "hd_pos_gain") hd_pos_gain = value;
        else if (param == "kb_rate_gain") kb_rate_gain = value;
        else if (param == "cycle_time") cycle_time = value;
        else if (param == "deltaStiffnessPerSecond") deltaStiffnessPerSecond = value;
        else if (param == "reactTimeCloser_seconds") reactTimeCloser_seconds = value;
        else if (param == "reactTimeAway_seconds") reactTimeAway_seconds = value;
        else if (param == "reactTimeSteady_seconds") reactTimeSteady_seconds = value;
        else if (param == "slopeTh") slopeTh = value;
        else if (param == "nearbyTh") nearbyTh = value;
        else if (param == "maxStiff") maxStiff = value;
        else if (param == "maxForce") maxForce = value;
        else if (param == "teleoperation") teleoperation = (bool) value;
        else if (param == "targeting_assistance") targeting_assistance = (bool) value;
        else if (param == "targeting_fixed_assistance") targeting_fixed_assistance = (bool) value;
        else if (param == "auto_steering_manual_eversion") auto_steering_manual_eversion = (bool) value;
        else if (param == "manual_steering_auto_eversion") manual_steering_auto_eversion = (bool) value;
        else if (param == "full_autonomy") full_autonomy = (bool) value;
        else if (param == "debug") debug = (bool) value;
        else if (param == "save_data") save_data = (bool) value;

        std::cout << param << ": " << value << std::endl;
        check_file++;
    }

    if( check_file == 0) {
        std::cout<< "[ERROR] cannot open the file " << filename << std::endl;
        return false;
    }
    else {
        std::cout<< "Loading params completed!" << std::endl << std::endl;
        return true;
    }
}

// measure time difference
inline timespec diff_time(timespec *start, timespec *stop)
{
    timespec* result = new timespec;
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }

    return *result;
}

// measure distance
inline float dist(float p1[3], float p2[3])
{
    return sqrtf(((p1[0] - p2[0]) * (p1[0] - p2[0])) + ((p1[1] - p2[1]) * (p1[1] - p2[1])) + ((p1[2] - p2[2]) * (p1[2] - p2[2])));
}
