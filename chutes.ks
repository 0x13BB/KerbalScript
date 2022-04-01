until  ship:altitude < 10000 {
    wait 0.1.
    

}
stage.

until  ship:altitude < 1000 {
    wait 0.1.
    

}
FOR P IN SHIP:PARTSDUBBEDPATTERN("Antenna") {

   p:getmodule("ModuleRTAntenna"):doevent("activate").

}.