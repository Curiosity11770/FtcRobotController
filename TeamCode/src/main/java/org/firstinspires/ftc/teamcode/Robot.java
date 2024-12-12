package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;

public class Robot {

    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;
    public Intake intake;
    public Extension extension;
    public Lift lift;
    public Scoring scoring;

    public Robot(LinearOpMode opmode){
        myOpMode = opmode;
    }
    public void init() {
        drivetrain = new Drivetrain(myOpMode);
        intake = new Intake(myOpMode);
        lift = new Lift(myOpMode);
        extension = new Extension (myOpMode);
        scoring = new Scoring (myOpMode);

        drivetrain.init();
        intake.init();
        lift.init();
        extension.init();
        scoring.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    public void teleOp() {
        drivetrain.teleOp();
        lift.teleOp();
        extension.teleOp();
        intake.teleOp();
        scoring.teleOp();
    }

    public void update(){
        drivetrain.update();
    }

}
