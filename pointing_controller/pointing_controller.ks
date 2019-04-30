@lazyglobal off.

// This controller is made for steering a vessel 
// in situations where the roll angle does not 
// matter (e.g. most rockets). It is capable of 
// performing large changes in orientation. And it 
// can control vessels in a large range of attitude 
// control 'strength'. However, this strength, which 
// is the available angular acceleration, must be 
// known in advance. It can be measured with the 
// accompanying script `angular_acceleration_measurement`.


// Usage example:
// 
//    local pointing_controller to pointing_controller_create().
//    until 0
//    {
//        local target_inertial to V(1,2,3):normalized.
//        local max_angular_acceleration to 0.03.
//        pointing_controller_update(
//            pointing_controller, 
//            target_inertial, 
//            max_angular_acceleration
//        ).
//        wait 0.
//    }

function pointing_controller_create
{
    parameter show_vectors is true.

    // Settling time in seconds.
    // High values make the controller react slowly.
    // Very low values may cause instability.
    parameter tuning_time is 0.5.
    parameter vector_scale is 20.

    local self to lexicon().

    local w0 to (1.0 / tuning_time).
    self:add("Kp", w0^2).
    self:add("Kd", 1.4142 * w0).
    self:add("show_vectors", true).

    if self["show_vectors"]
    {
        self:add("vecdraw_scale", vector_scale).
        self:add("vecdraw_attitude", vecdrawargs(v(0,0,0), 10*v(1,1,1), rgb(1,0,0), "attitude", 1.0, true, 0.2)).
        self:add("vecdraw_target", vecdrawargs(v(0,0,0), v(0,0,0), rgb(0,0,1), "target", 1.0, true, 0.2)).
        self:add("vecdraw_error", vecdrawargs(v(0,0,0), v(0,0,0), rgb(0,1,0), "attitude_error", 1.0, true, 0.2)).
        self:add("vecdraw_velocity", vecdrawargs(v(0,0,0), v(0,0,0), rgb(1,1,0), "ang_velocity", 1.0, true, 0.2)).
    }

    return self.
}

function pointing_controller_update
{
    // The object returned by `pointing_controller_create()`.
    parameter self.

    // The direction the vessel should be pointed to.
    // Must be a unit vector vector.
    parameter target_inertial.

    // The maximum angular acceleration in the pitch and yaw axes
    // for the current configuration of the vessel.
    // In radian per second per second.
    // Appropriate values can be measured with 
    // the script `angular_acceleration_measurement`.
    parameter max_angular_acceleration.


    local v_switch to -self["Kd"]/self["Kp"] * max_angular_acceleration.
    local x_switch to (-max_angular_acceleration - self["Kd"] * v_switch) / self["Kp"].

    local b2i to ship:facing.
    local i2b to ship:facing:inverse.
    local target_body to i2b * target_inertial.
    local attitude_error_mag to constant:degtorad * arctan2(sqrt(target_body:x^2+target_body:y^2), target_body:z).
    local attitude_error_dir_body to V(target_body:y, -target_body:x, 0):normalized.
    local attitude_error_body to attitude_error_mag * attitude_error_dir_body.

    local angular_vel_body to (i2b * ship:angularvel).
    local angular_accel_body to V(0,0,0).

    if attitude_error_mag < x_switch
    {
        // PD controller
        set angular_accel_body to -self["Kd"] * angular_vel_body - self["Kp"] * attitude_error_body.
    }
    else
    {
        // sliding mode controller
        local angular_vel_reference to 
            - attitude_error_dir_body 
            * signsqrt(2 * abs(attitude_error_mag) * max_angular_acceleration).

        set angular_accel_body to -self["Kd"] * (angular_vel_body - angular_vel_reference).
    }

    if self["show_vectors"]
    {
        set self["vecdraw_attitude"]:vec to b2i * V(0,0,1) * self["vecdraw_scale"].
        set self["vecdraw_target"]:vec to target_inertial * self["vecdraw_scale"].
        set self["vecdraw_error"]:vec to 0.3 * (b2i * attitude_error_body) * self["vecdraw_scale"].
        set self["vecdraw_velocity"]:vec to 3 * ship:angularvel * self["vecdraw_scale"].
    }    

    set ship:control:pitch to -angular_accel_body:x / max_angular_acceleration.
    set ship:control:yaw   to  angular_accel_body:y / max_angular_acceleration.
    set ship:control:roll  to -angular_accel_body:z / max_angular_acceleration.

    return attitude_error_mag.
}

function signsqrt
{
    parameter x.
    if x > 0 { return sqrt(x). }
    if x < 0 { return -sqrt(-x). }
    return 0.
}