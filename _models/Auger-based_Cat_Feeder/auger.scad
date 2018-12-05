use <Thread_Library.scad>

// slice along Y-axis

*auger();
auger_import();

drivemount();

tubemount();

*servoshim();

%pvctube();

*servo();

module auger_import() {
    intersection() {
        translate([-45,-45,0]) cube(96);
        rotate([0,0,-15]) import("auger.stl");
    }
}
module auger() {
    cylinder(h=96, r=4, $fn=24);    
    trapezoidThread( 
        length=118,
        pitch=26,
        pitchRadius=8,
        threadHeightToPitch=0.70,
        profileRatio=0.20,
        threadAngle=10,
        RH=true,
        clearance=0.1,
        backlash=0.1,
        stepsPerTurn=48
    );
}

module drivemount() {
    translate([0,0,-2]) difference() {
        cylinder(h=5, r=40/2, $fn=48);
        for (angle = [0, 90, 180, 270]) translate([0,0,-1]) rotate([0,0,-45+angle]) {
            linear_extrude(height=2) polygon([[-8.5/2,0],[8.5/2,0],[8.5/2,8.5/2],[5.4/2,42/2],[-5.4/2,42/2],[-8.5/2,8.5/2]]);
            translate([13,0,0]) cylinder(h=8, r=0.9, $fn=12);  // 10 or 13
        }
        cylinder(h=2, r=3, $fn=8);
    }
}

module pvctube() {
    translate([0,0,-3]) difference() {
        cylinder(h=90, r=55/2, $fn=48);
        translate([0,0,-1]) cylinder(h=92, r=48/2, $fn=48);
    }
    // TODO: measure PVC tube and make sure these numbers make sense
}

module tubemount() {
    color([1,0,0])
    translate([0,0,-1-3]) difference() {
        union() {
            difference() {
                union() {
                    cylinder(h=3+5, r=48/2, $fn=48);
                    translate([0,0,-6.5]) cylinder(h=7.5, r=56/2, $fn=48);
                    translate([13,-22/2-3,-6.5]) cube([25.5,22+3*2,4]);
                }
                translate([0,0,-7]) cylinder(h=20, r=42/2, $fn=48);
                translate([0,-22/2,-19.5]) cube([32,22,20+10]);
            }
            translate([-21.5,-24/2,-6.5]) cube([10,24,4]);
        }
        translate([-4.5-10,-20.2/2,-11]) union() {
            translate([0,5,0]) cylinder(h=10, r=1.2);
            translate([0,5+10.2,0]) cylinder(h=10, r=1.2);
            translate([50.5-1,5,0]) cylinder(h=10, r=1.2);
            translate([50.5-1,5+10.2,0]) cylinder(h=10, r=1.2);
        }
    }
    // #2 x length? 2.0mm?  hole is ~1.55mm
}


module servoshim() {
    color([.7,.7,0])
    translate([0,0,-1-3-20]) difference() {
        union() {
            difference() {
                translate([-18.5,-22/2-3,-6.5]) cube([31.5+25.5,22+3*2,4]);
                translate([-18,-22/2,-19.5]) cube([18+32,22,20+10]);
            }
            translate([-18.5,-24/2,-6.5]) cube([7,24,4]);
        }
        translate([-4.5-10,-20.2/2,-11]) union() {
            translate([0,5,0]) cylinder(h=10, r=1.2);
            translate([0,5+10.2,0]) cylinder(h=10, r=1.2);
            translate([50.5-1,5,0]) cylinder(h=10, r=1.2);
            translate([50.5-1,5+10.2,0]) cylinder(h=10, r=1.2);
        }
    }
}




module servo() {
    color([0,0.7,.3])
    translate([-4.5-10,-20.2/2,-12-3]) union() {
        cube([50.5,20.2,4.5]);
        translate([4.5,0,0]) cube([41.5,20.2,11]);
        translate([4.5+10,20.2/2,11]) cylinder(h=4, r=3, $fn=24);
        translate([0+1,5,0]) cylinder(h=10, r=1);
        translate([0+1,5+10.2,0]) cylinder(h=10, r=1);
        translate([50.5-1,5,0]) cylinder(h=10, r=1);
        translate([50.5-1,5+10.2,0]) cylinder(h=10, r=1);
    }
}

// TODO: T-junction tube?  Or drill hole in top?
// TODO: diagonal mount to the bin?
