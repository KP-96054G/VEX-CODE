// void autonplus(){
//     clamp.set_value(1);
// chassis.setPose(-48, -24, 270);
// chassis.moveToPoint(-18,-24, 1500, {.forwards =  false, .maxSpeed = 80}, false);
// float v = chassis.getPose().x;
// float h = chassis.getPose().y;
// pros::delay(500);
// clamp.set_value(0);
// pros::delay(1000);
// intake.move(127);
// chassis.moveToPose(-18, -24, 175, 1500, {.maxSpeed = 80}, true);
// chassis.moveToPoint(-18, -46, 1500, {.maxSpeed = 80}, true);
// intake.move(0);
// pros::delay(1);
// intake.move(127);
// chassis.moveToPoint(-18, -24, 1500, {.maxSpeed = 80}, true);
// }

// void runIntake(){
//     for(int i = 0; i < 660; i++){
//                intake.move_velocity(1000);
//                intake.move(127);
//                pros::delay(1);

//            }
// // }

// void mtp(int k, int l, int p, bool f, int s, bool g){
//     chassis.moveToPoint(k, l, p, {.forwards = f, .maxSpeed = s}, g);
// }



        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        //     Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        //  Arm.move(-127);
            
        // }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        //     Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        //  Arm.move(127);
            
        // }else{

        //     Arm.brake();
            
        // }

        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        //     Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        //     for(int i = 0; i < 160; i++){
        //         Arm.move_velocity(1000);
        //         Arm.move(-127);

        //         pros::delay(1);
        //     }
        //     Arm.brake();
        //     pros::delay(20);
        // }

