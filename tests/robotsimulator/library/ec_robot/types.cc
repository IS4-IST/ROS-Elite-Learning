#include "ec_robot/types.hpp"

namespace ec {

const char* to_string(Mode m) {
    switch (m) {
        case Mode::Teach: return "teach";
        case Mode::Play: return "play";
        case Mode::Remote: return "remote";
    }
    return "unknown";
}
const char* to_string(State s) {
    switch (s) {
        case State::Stop: return "stop";
        case State::Pause: return "pause";
        case State::EStop: return "emergency stop";
        case State::Running: return "running";
        case State::Alarm: return "alarm";
        case State::Collision: return "collision";
    }
    return "unknown";
}

}  // namespace ec
