package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CameraScanningTestAuto", group = "Linear OpMode")
public class CameraScanningAuto extends LinearOpMode {
    Robot robot = new Robot(this);
    Camera camera = new Camera(this);

   OpenCv.OpenCvPosition position = OpenCv.OpenCvPosition.LEFT;

    public void runOpMode(){
        robot.init();
        camera.initAuto();
        position = camera.pipeline.getAnalysis();

        if (position == OpenCv.OpenCvPosition.RIGHT){
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

        }
        robot.drivetrain.stopMotors();
        waitForStart();

        //robot.drivetrain.strafeRight(0.7, 40);
        //robot.drivetrain.driveForwards(0.7, 20);
        //robot.drivetrain.stopMotors();

    }

}