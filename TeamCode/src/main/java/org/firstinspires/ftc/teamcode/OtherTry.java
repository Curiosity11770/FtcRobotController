package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OtherTry", group = "Linear Opmode")
public class OtherTry extends LinearOpMode {
    Robot robot = new Robot(this);
    Camera camera = new Camera(this);

    OpenCv.OpenCvPosition position = OpenCv.OpenCvPosition.LEFT;

    public void runOpMode(){
        robot.init();
        camera.init();
       /* //position = camera.pipeline.getAnalysis();

        if(position == OpenCv.OpenCvPosition.RIGHT){
            robot.drivetrain.strafeRight(0.7, 20);
            robot.drivetrain.driveForwards(0.7, 20);
            //robot.intake.outtake(0.7, 3);

        } else if (position == OpenCv.OpenCvPosition.LEFT) {
            robot.drivetrain.strafeLeft(0.7,20);
            robot.drivetrain.driveForwards(0.7, 20);
            //robot.intake.outtake(0.7, 3);

        } else {
            robot.drivetrain.driveForwards(0.7, 20);
            //robot.intake.outtake(0.7, 3);

        }*/
        waitForStart();

        //robot.drivetrain.strafeLeft(0.7, 40);
        //robot.drivetrain.driveForwards(0.7, 20);
        //robot.drivetrain.stopMotors();

    }

}
