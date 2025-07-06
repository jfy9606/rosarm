#ifndef VISION_VIEW_CONTROL_H
#define VISION_VIEW_CONTROL_H

#include <string>

namespace vision {

/**
 * @brief View control class for managing camera view modes
 * 
 * This class provides functionality for managing camera view modes
 * following the control part of the node+control architecture.
 */
class ViewControl {
public:
    /**
     * @brief Constructor
     * 
     * @param initial_view_mode Initial view mode (0=left, 1=right)
     */
    ViewControl(int initial_view_mode = 0);
    
    /**
     * @brief Destructor
     */
    ~ViewControl();
    
    /**
     * @brief Switch to a different view mode
     * 
     * @param mode New view mode (0=left, 1=right)
     * @return True if switch was successful, false otherwise
     */
    bool switchViewMode(int mode);
    
    /**
     * @brief Get the current view mode
     * 
     * @return Current view mode (0=left, 1=right)
     */
    int getCurrentMode() const { return current_mode_; }
    
    /**
     * @brief Get a description of the current view mode
     * 
     * @return Description of the current view mode
     */
    std::string getCurrentModeDescription() const;
    
    /**
     * @brief Check if the view mode is valid
     * 
     * @param mode View mode to check
     * @return True if the view mode is valid, false otherwise
     */
    static bool isValidMode(int mode) { return mode == 0 || mode == 1; }
    
private:
    int current_mode_;
};

} // namespace vision

#endif // VISION_VIEW_CONTROL_H 