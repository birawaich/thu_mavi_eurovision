
/* PARAMETERS 
units in mm
*/
plate_thickness = 6.9;
feature_thickness = 2;

plate_edge = 15;
screwhole_separation = 28;
screwhole_diameter = 2.5;

cablehole_radius = screwhole_diameter;

num_negative_translations = 4;
num_positive_translations = 10;
divisor_translations = 4;

baseline = 100;

epsilon = 1;

/* MODULES */

//mount for 1 camera
//center in bottom left corner
//true to add markers (to mark the main holds)
module camera_mount() {  
    //screwholes
    translate([0, 0, -epsilon])
    cylinder(h = plate_thickness+2*epsilon, r = screwhole_diameter/2);
    translate([screwhole_separation, 0, -epsilon])
    cylinder(h = plate_thickness+2*epsilon, r = screwhole_diameter/2);
    translate([screwhole_separation, screwhole_separation, -epsilon])
    cylinder(h = plate_thickness+2*epsilon, r = screwhole_diameter/2);
    translate([0, screwhole_separation, -epsilon])
    cylinder(h = plate_thickness+2*epsilon, r = screwhole_diameter/2);
    
    //add cable stuff    
    translate([screwhole_separation/2,screwhole_separation+plate_edge,-epsilon]) {
        cylinder(h = plate_thickness+2*epsilon, r = cablehole_radius);
    }
    
}

module camera_mount_pos() {
    translate([0-screwhole_diameter/2, screwhole_separation / 4, plate_thickness]) {
            cube([screwhole_diameter, screwhole_separation / 2, feature_thickness]);
    }
    translate([screwhole_separation-screwhole_diameter/2, screwhole_separation / 4, plate_thickness]) {
        cube([screwhole_diameter, screwhole_separation / 2, feature_thickness]);
    }
}

/* BUILD */

difference() {
    // Add the plate below everything
    translate([-plate_edge, -plate_edge,0]){
        cube([2*plate_edge+screwhole_separation+baseline+screwhole_separation/divisor_translations*num_positive_translations,2*plate_edge+screwhole_separation,plate_thickness]);
    }

    camera_mount();

    translate([baseline,0,0]) {
        camera_mount();
    }


    // Add negative mounts
    for (i = [1:num_negative_translations]) {
        translate([baseline - i * screwhole_separation / divisor_translations, 0, 0]) {
            camera_mount();
        }
    }
    // Add positive mounts
    for (i = [1:num_positive_translations]) {
        translate([baseline + i * screwhole_separation / divisor_translations, 0, 0]) {
            camera_mount();
        }
    }
}

//add marker
camera_mount_pos();
translate([baseline,0,0]) {
    camera_mount_pos();
}

//add logo
// Import SVG (ensure you have the correct path)
translate([screwhole_separation+5,3, plate_thickness])  // Position the SVG above the plate
scale([1, 1, 1])                             // Scale the SVG to fit on the plate
linear_extrude(height = feature_thickness) {
    import("logo_eurovision.svg");
}  




