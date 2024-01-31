package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    public boolean passed, notPassed;
    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;

    public Lift lift;

    public Intake intake;

    public Scoring scoring;

    public Drone drone;

    public DualPortalCameras camera;

    public Robot(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init(){
        drivetrain = new Drivetrain(myOpMode);
        lift = new Lift(myOpMode);
        intake = new Intake(myOpMode);
        scoring = new Scoring(myOpMode);
        drone = new Drone(myOpMode);
        camera = new DualPortalCameras(myOpMode);

        myOpMode.telemetry.addData("IsWorking", drone);

        drivetrain.init();
        lift.init();
        intake.init();
        scoring.init();
        drone.init();
        camera.init();
    }

    public void teleOp(){
        drivetrain.teleOp();
        lift.teleOp();
        intake.teleOp();
        scoring.teleOp(passed, notPassed);
        drone.teleOp();

        if(intake.frontPixel && intake.backPixel){
           scoring.rightGateServo.setPosition(scoring.GATE_DOWN_RIGHT);
           scoring.leftGateServo.setPosition(scoring.GATE_DOWN_LEFT);
        }

        if(myOpMode.gamepad2.dpad_right){
            lift.liftMode = Lift.LiftMode.LOW;
            scoring.leftArmServo.setPosition(scoring.ARM_UP_LEFT);
            scoring.rightArmServo.setPosition(scoring.ARM_UP_RIGHT);
            scoring.isUp = true;
            scoring.timer.reset();
            if(scoring.timer.seconds() > 1){
                scoring.boxServo.setPosition(scoring.BOX_OUT);
            }

        }
    }
    /*public void touchSense(){
        if(!lift.getTouch()){
            intake.intakeLeft.setPower(0);
            intake.intakeRight.setPower(0);
            intake.intakeMotor.setPower(0);
        }
    }*/
}
