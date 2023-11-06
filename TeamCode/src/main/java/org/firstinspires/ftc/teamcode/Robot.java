package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {
    private LinearOpMode myOpMode= null;

    public Drivetrain drivetrain;

    //public Lift lift;

    //public Intake intake;

    //public Scoring scoring;

    public Robot(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init(){
        drivetrain = new Drivetrain(myOpMode);
        /*lift = new Lift(myOpMode);
        intake = new Intake(myOpMode);
        scoring = new Scoring(myOpMode);

        drivetrain.init();
        lift.init();
        intake.init();
        scoring.init();*/

        drivetrain.init();
    }

    public void teleOp(){
        drivetrain.teleOp();
        //lift.teleOp();
        //intake.teleOp();
        //scoring.teleOp();
    }
}
