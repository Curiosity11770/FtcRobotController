package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;
    public Intake intake;
    public Extension extension;
    public Lift lift;
    public Robot(LinearOpMode opmode){
        myOpMode = opmode;
    }
    public void init() {
        drivetrain = new Drivetrain(myOpMode);
        intake = new Intake(myOpMode);
        lift = new Lift(myOpMode);
        extension = new Extension (myOpMode);

        drivetrain.init();
        intake.init();
        lift.init();
        extension.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    public void teleOp() {
        drivetrain.teleOp();
        lift.teleOp();
        extension.teleOp();
        intake.teleOp();
    }


}
