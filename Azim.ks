declare function Azim_fun {
    declare local parameter A_ang.
    declare local parameter _lan.

    local B_ang to 90.
    local c to ship:latitude.
    local Y_ang to ship:latitude.
    //local Y_ang to arccos(sin(A_ang) * cos(c)).
    local a to arccos(cos(A_ang) / sin(Y_ang)).
    local Lan to a .// + ship:longitude.

   
    local Aa_ang to arcsin(cos(Y_ang) / cos(c)).



    return Lan.
    
}

print Azim_fun(135, 1).
//print Lan.