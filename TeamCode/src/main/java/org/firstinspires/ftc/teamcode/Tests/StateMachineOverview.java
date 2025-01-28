//package org.firstinspires.ftc.teamcode.Tests;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//// Enum should be declared inside or outside the class
//enum StateMachine {
//    START,
//    INTAKE,
//    OUTAKE,
//    END,
//    IDLE
//}
//
//public class StateMachineOverview {
//    public static boolean intakeReady = false;
//
//    ElapsedTime timer = new ElapsedTime();
//    // Declare state variable
//    private StateMachine state;
//
//    public StateMachineOverview() {
//        // Initialize state with a valid enum value
//    }
//
//    public void updateState() {
//        // Example of how to switch based on state
//        switch (state) {
//            case INTAKE:
//                System.out.println("Currently in INTAKE state.");
//                INTAKE_STATE();
//                break;
//
//            case OUTAKE:
//                System.out.println("Currently in OUTAKE state.");
//                break;
//
//            case START:
//                System.out.println("Currently in START state.");
//                break;
//
//            case END:
//                System.out.println("Currently in END state.");
//                break;
//
//            default:
//                throw new IllegalStateException("Unexpected value: " + state);
//        }
//    }
//
//    public void setState(StateMachine newState) {
//        // Method to update the state
//        if(newState != state) {
//            timer.reset();
//            state = newState;
//        }
//    }
//    boolean isInPosition;
//    public void INTAKE_STATE(){
//
//        //slides PID
//
//
//        if(timer.milliseconds() > 600){
//            if(isInPosition){
//                //dropped sucessfully
//                setState(StateMachine.IDLE);
//                intakeReady = true;
//            }
//
//            //dopping
//
//        }else if (timer.milliseconds() > 300){
//            //slides extened, now put out outake
//        }else{
//            //Extend slides
//        }
//    }
//
//    public void scan(){
//
//        if(see_block){
//            y_servo_position = ;//something based on camera
//
//            setState(PICKUP);
//        }
//    }
//
//    boolean positionReached = false;
//
//    //when we switch to this state, we assume, target y_servo_position is set and that
//    //target x_turret_rotation is set
//    public void PICKUP{
//
//        if(positionReached){
//            setState(RETRACT);
//        }else{
//            servo1.setposition(y_servo_position);
//        }
//    }
//
//}
//
//
//public class auto{
//
//
//    autostate
//
//    swtich(autoState){
//
//
//    }
//
//    public void pickupState(){
//
//
//        drivetrain.setTarget(pickupPosition1);
//        outtake.setPositon();
//        intake.setPOsitom
//    }
//
//    drive.driveTOPosition(pickupPosition1.x, pickuppsoition1.y, targetheading)
//
//    position = odo.getGlobalPOsition();
//
//    rightmotor.setPOwer()
//}