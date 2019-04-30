@lazyglobal off.
wait 1.0.

// This script measures the angular acceleration of a vessel
// for the pitch, yaw and roll axis. The vessel should be 
// placed in an orbit, before running this script. This 
// ensures that the angular acceleration is caused by 
// the RCS or engines, and not by the ground or atmosphere.


// Modify this to suit your needs
rcs on.
global test_throttle_level to 0.0.





print "Pitch angular acceleration:".
test_program({set ship:control:pitch to 1.}).

print "Yaw angular acceleration:".
test_program({set ship:control:yaw to 1.}).

print "Roll angular acceleration:".
test_program({set ship:control:roll to 1.}).


function test_program
{
    parameter control_callback.
    wait 1.0.

    // Set angular velocity to zero by starting on-rails physics.
    set kuniverse:timewarp:warp to 1. 
    wait 1.0.
    set kuniverse:timewarp:warp to 0.
    wait 1.0.
    sas off.

    if ship:angularvel:mag > 1e-4
    {
        print "Error: Angular velocity too high.".
        shutdown.
    }

    control_callback().
    lock throttle to test_throttle_level.
    
    wait 2.0.
    local t_start to time:seconds.
    local t_prev to 0.
    local value_prev to 0.
    wait 0.1.

    until t_start + 5 < time:seconds
    {
        local t_now to time:seconds.
        local value_now to ship:angularvel:mag.
        if t_prev > 0 { print (value_now - value_prev) / (t_now - t_prev). }
        set t_prev to t_now.
        set value_prev to value_now.
        wait 0.7.
    }

    set ship:control:neutralize to true.
    lock throttle to 0.
    wait 1.0.
}