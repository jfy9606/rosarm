#include "vision/view_control.h"

namespace vision {

ViewControl::ViewControl(int initial_view_mode)
    : current_mode_(initial_view_mode) {
    // Ensure the initial mode is valid
    if (!isValidMode(current_mode_)) {
        current_mode_ = 0; // Default to left view
    }
}

ViewControl::~ViewControl() {
    // Nothing to do here
}

bool ViewControl::switchViewMode(int mode) {
    // Check if the mode is valid
    if (!isValidMode(mode)) {
        return false;
    }
    
    // Set the new mode
    current_mode_ = mode;
    return true;
}

std::string ViewControl::getCurrentModeDescription() const {
    switch (current_mode_) {
        case 0:
            return "Left Camera";
        case 1:
            return "Right Camera";
        default:
            return "Unknown";
    }
}

} // namespace vision 