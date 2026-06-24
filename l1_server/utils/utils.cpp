#include "utils.h"

namespace robot_server_lib {
namespace utils {

std::ostringstream RobotStatuswordToJsonString(const int &status_word) {
    std::ostringstream ss;
    switch (status_word) {
        case 0:
            ss << "\"NOT_READY\"";
            break;
        case 1:
            ss << "\"READY\"";
            break;
        case 2:
            ss << "\"ENABLED\"";
            break;
        case 4:
            ss << "\"MOTOR_FAULT\"";
            break;
        case 8:
            ss << "\"MOTOR_QUICK_STOP\"";
            break;
        default:
            ss << status_word;
            break;
    }
    return ss;
}

std::ostringstream RobotControlwordToJsonString(const int &control_word) {
    std::ostringstream ss;
    std::vector<std::string> flags;

    if (control_word & CTRLWORD_MOTOR_SWITCH_ON) flags.push_back("\"MOTOR_SWITCH_ON\"");
    if (control_word & CTRLWORD_MOTOR_ENABLE) flags.push_back("\"MOTOR_ENABLE\"");
    if (control_word & CTRLWORD_MOTOR_QUICK_STOP) flags.push_back("\"MOTOR_QUICK_STOP\"");
    if (control_word & CTRLWORD_MOTOR_CLEAR_FAULT) flags.push_back("\"MOTOR_CLEAR_FAULT\"");
    if (control_word & CTRLWORD_MOTOR_CST_MODE) flags.push_back("\"MOTOR_CST_MODE\"");

    if (!flags.empty()) {
        ss << "[";
        for (size_t i = 0; i < flags.size(); ++i) {
            if (i > 0) ss << ",";
            ss << flags[i];
        }
        ss << "]";
    } else {
        ss << control_word;
    }

    return ss;
}

}  // namespace utils
}  // namespace robot_server_lib
