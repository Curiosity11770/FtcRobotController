package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.SimpleCamera;

public class Robot {
    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;

    public Lift lift;

    public Intake intake;

    public Scoring scoring;

    public Drone drone;

    public SimpleCamera camera;

    public Robot(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init(){
        drivetrain = new Drivetrain(myOpMode);
        lift = new Lift(myOpMode);
        intake = new Intake(myOpMode);
        scoring = new Scoring(myOpMode);
        drone = new Drone(myOpMode);
        camera = new SimpleCamera(myOpMode);

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
        scoring.teleOp(intake.passed1, intake.passed2);
        drone.teleOp();
    }
    /*public void touchSense(){
        if(!lift.getTouch()){
            intake.intakeLeft.setPower(0);
            intake.intakeRight.setPower(0);
            intake.intakeMotor.setPower(0);
        }
    }*/
}
