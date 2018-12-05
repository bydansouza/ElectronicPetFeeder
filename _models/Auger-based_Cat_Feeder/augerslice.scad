
// Take the STL render of auger_import() and drivemount() together
// to generate a printable plate

translate([40,-25,0]) rotate([0,-90,0]) intersection() {
    translate([0,-30,-10]) cube([30,60,120]);
    import("auger_screw.stl");
}

translate([40,25,0]) rotate([0,90,0]) rotate([180,0,0]) intersection() {
    translate([-30,-30,-10]) cube([30,60,120]);
    import("auger_screw.stl");
}