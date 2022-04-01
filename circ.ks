declare function cosm_vel_1 { 
    declare local parameter _alt.
    declare local parameter _body.

    return sqrt(
        constant:G * (_body:mass / (_body:radius + _alt))
        ).
}

declare function speed_at {
    declare local parameter _alt.
    declare local parameter _body.

    return sqrt(
        constant:G * _body:mass * ((2 / (_body:radius + _alt)) - ( 1 / ship:orbit:semimajoraxis))
        ).
}

declare function need_deltaV {
    declare local parameter _alt.
    declare local parameter _body.

    return cosm_vel_1(_alt, _body) - speed_at(_alt, _body).
    //return ship:velocity:orbit:mag - cosm_vel_1(_alt, _body).
}


declare function g_at {
    declare local parameter _alt.
    declare local parameter _body.

    return (constant:G * _body:mass) / (_body:radius + _alt)^2.
}

declare function ship_vert_speed {

    return ship:verticalspeed.
}

declare function centripetal_acceleration {
    declare local parameter _alt.
    declare local parameter _body.

    return (ship:velocity:ORBIT:mag^2) / (_body:radius + _alt).
}


declare function cur_thrust {
    local _thrust to 0.
    local k to list().
    list ENGINES in k.

    for i in k {
        if i:ignition = true and i:flameout = false {
            set _thrust to _thrust + i:thrust.
        }
    }

    return _thrust. // ship:mass.
}

declare function cur_isp {
    local _Isp to 0.
    local k to list().
    local j to 0.
    list ENGINES in k.

    for l in k {
        if l:ignition = true and l:flameout = false {
            set _Isp to _Isp + l:isp.
            set j to j + 1.
        }
    }

    return _Isp / j.
}

declare function burn_time{
    declare local parameter _Isp.
    declare local parameter _delta_v.
    declare local parameter _thrust.

    return ((_Isp * constant:g0 * ship:mass) / _thrust) * (1 - constant:e^(-1 * _delta_v / (_Isp * constant:g0))).
}

// declare function second_stage_steering {
//     declare local parameter _body.
//     declare local parameter _time.

//     return min(1,max(-1,   ((g_at( ship:ALTITUDE, _body) - centripetal_acceleration(ship:ALTITUDE, _body)) / cur_thrust() )   )).
// }

// declare function second_stage_steering {
//     local _g to g_at(ship:altitude, Kerbin).
//     local _delta_t to burn_time(cur_isp(), need_deltaV(ship:apoapsis, Kerbin), cur_thrust()).
//     local ver_speed to ship:verticalspeed.
//     local _delta_v1_speed to need_deltaV(ship:apoapsis, Kerbin).
//     local cosm_1_speed to cosm_vel_1(ship:apoapsis, Kerbin).

//     local _result to (_g * ((cosm_1_speed - VXCL(Ship:UP:vector, ship:velocity:orbit):mag)/cosm_1_speed) * _delta_t - ver_speed) / _delta_v1_speed.

//     return arcsin((min(1,max(-1,  _result)))).
// }


declare function smooth_inc {
    declare local parameter _stage.
    
    from {local x is 0.} until x = number_of_stages step {set x to x + 1.} do {
        list_of_deltas:add(ship:stagedeltav(x):vacuum).
    }

        return ship:stagedeltav(_stage):vacuum / list_of_deltas[_stage].

}


declare function second_stage_steering {
    declare local parameter _body.
    //declare local parameter _time.

    return min(1,max(-1,   ((g_at( ship:ALTITUDE, _body) - centripetal_acceleration(ship:ALTITUDE, _body)) / cur_thrust() )   )).
}



until orbit:eccentricity <= 0.00015 {

    if orbit:eccentricity <= 0.01 {
        lock throttle to 0.01.
        set kuniverse:timewarp:rate to 1.
        }
    else
    {
        lock throttle to 0.6.
    }
    
    set delta_t to burn_time(cur_isp(), need_deltaV(ship:apoapsis, Kerbin), cur_thrust()).
   
    lock steering to heading( azim, max(min(second_stage_steering(),90), -15)).

   
    print delta_t.
    wait 0.1.



}