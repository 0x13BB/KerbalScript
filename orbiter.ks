clearScreen.
//set number_of_stages to stage:number.
set list_of_deltas to list().
set number_of_stages to ship:stagenum.
set close_loop_stage_ang to 0.
set azim to 90.
set target_apoapsis to 80000.

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

declare function second_stage_steering {
    local _g to g_at(ship:altitude, Kerbin).
    local _delta_t to burn_time(cur_isp(), need_deltaV(ship:apoapsis, Kerbin), cur_thrust()).
    local ver_speed to ship:verticalspeed.
    local _delta_v1_speed to need_deltaV(ship:apoapsis, Kerbin).
    local cosm_1_speed to cosm_vel_1(ship:apoapsis, Kerbin).

    local _result to (_g * ((cosm_1_speed - VXCL(Ship:UP:vector, ship:velocity:orbit):mag)/cosm_1_speed) * _delta_t - ver_speed) / _delta_v1_speed.

    return arcsin((min(1,max(-1,  _result)))).
}


declare function smooth_inc {
    declare local parameter _stage.
    
    from {local x is 0.} until x = number_of_stages step {set x to x + 1.} do {
        list_of_deltas:add(ship:stagedeltav(x):vacuum).
    }

        return ship:stagedeltav(_stage):vacuum / list_of_deltas[_stage].

}
wait 0.1.
stage.

until  stage:number = number_of_stages - 1 {
    wait 0.1.
    print Orbit:ECCENTRICITY.
}

wait 0.1.


print stage:number.
until ship:stagedeltav(number_of_stages - 1):vacuum <= 10 { //stage#1 close-loop guidance

    lock throttle to 1.
    print ship:stagedeltav(number_of_stages - 1):vacuum.

   
    //lock steering to heading( 90,  90 - 90(1 - smooth_inc(1))^tan(smooth_inc(1) + 0.2)).

    //set steer_var to ( ( 1 / ( 1 + 2^( -2 * smooth_inc(1) * 6 + 6 ) ) ) * ( 90 - close_loop_stage_ang ) ) + close_loop_stage_ang.
    set steer_var to max(close_loop_stage_ang,(target_apoapsis - ship:apoapsis) / (target_apoapsis / 90)).

    if ship:airspeed < 10 { lock steering to heading( azim , 90 ).}
    else {
        lock steering to heading( azim , steer_var  ).
    }

    
    //lock steering to heading( 90,steer_var).

    //2.5^ = 70 + 400 ang 10 
    //2^   = 84 + 394 ang 10 
    //3^   = 60 + 415 ang 10 
    //1.5^ = 112 + 421 w 100ms lock ang 10 
    //2^   = 77 + 420 w 100ms lock ang 10 
    //2^   = 76 + 387 ang 7
    //2^   = 73 + 339 w/o fin
    //2^   = 78 + 340 w/o fin ang 8
    //2^   = 82 + 353 w small fin ang 8
    //2^   = 70 + 335 w/o fin ang 7

    
    
    wait 0.1.
    
    //print steer_var.
}
wait 0.5.




lock throttle to 0.
wait 0.5.
stage.
lock steering to heading( azim, 0).
wait 5.
lock throttle to 0.10.
wait 0.5.
//until speed_at(ship:apoapsis, Kerbin) >= cosm_vel_1(ship:apoapsis, Kerbin)*0.995{
until orbit:eccentricity <= 0.0002 OR ship:velocity:orbit:mag >= cosm_vel_1(ship:altitude, Kerbin) {

    if orbit:eccentricity <= 0.01 {
        lock throttle to 0.02.
        set kuniverse:timewarp:rate to 1.
        }
    else
    {
        lock throttle to 0.2.
    }
    
    set delta_t to burn_time(cur_isp(), need_deltaV(ship:apoapsis, Kerbin), cur_thrust()).
    //print delta_t.

    //lock steering to heading( 90 , arcsin(second_stage_steering(Kerbin, delta_t)) + min(3,max(-3,-(ship:verticalspeed / delta_t))) / 1 ). //- (ship:verticalspeed / delta_t)
    //lock steering to heading( 90 , arcsin(min(1,max(-1,(-(ship:verticalspeed / delta_t)) ))) ). //- (ship:verticalspeed / delta_t)
    lock steering to heading( azim, max(min(second_stage_steering(),90), -15)).

    //print arcsin(second_stage_steering(Kerbin, delta_t)) -(ship:verticalspeed / delta_t) / 10 . MAX(MIN(Fi,90),-10)
    print delta_t.
    wait 0.1.

// 3600 90 0.2 -35
// 3428 78 0.1 -15 apoapsis

}

print orbit:eccentricity.

lock throttle to 0.
UNLOCK STEERING.
UNLOCK THROTTLE.

//SAS ON.
wait 0.1.

FOR P IN SHIP:PARTSDUBBEDPATTERN("Antenna") {

   p:getmodule("ModuleRTAntenna"):doevent("activate").

}.

//print list_of_deltas[0].
//run fun.ks.