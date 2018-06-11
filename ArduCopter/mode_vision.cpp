#include "Copter.h"

/*
 * Init and run calls for vision flight mode
 */


bool Copter::ModeVision::init(bool ignore_checks)
{
    pos_control->set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ModeVision::run()
{

}
