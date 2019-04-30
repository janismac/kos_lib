@lazyglobal off.

// Linear interpolation in 1D.
// Usage example:
//    local x to list(17.4, 61.9, 147.8, 291.4, 501.9, 782.5).
//    local y to list(-20.13, -41.38, -75.05, -117.40, -163.58, -210.58).
//    
//    from {local x_eval is 0.} until x_eval > 800 step {set x_eval to x_eval + 12.3.} do 
//    {
//        print x_eval + ", " + interpolation(x_eval, x, y).
//    }

function interpolation
{
    // number, value of the independent variable for the interpolation
    parameter x_eval. 

    // list of numbers, data for the independent variable,
    // must be strictly increasing, must have two or more elements
    parameter x_data. 

    // list of numbers, data for the dependent variable,
    // must have the same length as x_data
    parameter y_data. 


    // lower and upper indices for bisection search
    local i to 0.
    local j to x_data:length-1.


    // extrapolation: keep first/last value constant
    if x_eval <= x_data[i]
    {
        return y_data[i].
    }
    if x_eval >= x_data[j]
    {
        return y_data[j].
    }

    // bisection search for the correct interval
    until j - i < 1.5
    {
        local k to round((i+j)/2). // midpoint index
        if x_eval < x_data[k]
        {
            set j to k.
        }
        else
        {
            set i to k.
        }
    }

    // interpolate
    local x0 to x_data[i].
    local x1 to x_data[j].
    local y0 to y_data[i].
    local y1 to y_data[j].

    return ((x_eval - x0) * (y1 - y0) / (x1 - x0) + y0).
}