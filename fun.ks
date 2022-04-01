clearScreen.

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
    local thrust to 0.
    local k to list().
    list ENGINES in k.

    for i in k {
        if i:ignition = true and i:flameout = false {
            set thrust to thrust + i:thrust.
        }
    }

    return thrust. // ship:mass.
}

// declare function time_to_v1 {
//     declare local parameter _accel.
//     declare local parameter _v1_speed.

//     return _v1_speed / _accel.

// }

declare function burn_time{
    declare local parameter _Isp.
    declare local parameter _delta_v.
    declare local parameter _thrust.

    return ((_Isp * constant:g0 * ship:mass) / _thrust) * (1 - constant:e^(-_delta_v / (_Isp * constant:g0))).
}


declare function orbit_sin_angle {
    declare local parameter _body.

    return min(1,max(-1,( g_at( ship:ALTITUDE, _body ) - centripetal_acceleration( ship:ALTITUDE, _body ) ) / cur_thrust())).
}



//print need_deltaV(ship:periapsis, Kerbin).
//print speed_at(ship:apoapsis, Kerbin) - cosm_vel_1(ship:apoapsis, Kerbin).
//print time_to_v1(cur_thrust, need_deltaV(ship:apoapsis, Kerbin)).
print burn_time(345,need_deltaV(apoapsis, Kerbin), 60).
print need_deltaV(apoapsis, Kerbin).
   // lock steering to heading( 90 , arcsin(min(1,max(-1,orbit_sin_angle(Kerbin) - ship:verticalspeed / 5)))).
 
