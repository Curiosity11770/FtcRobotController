/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "CameraScanningBlueRight", group = "Linear OpMode")
public class CameraScanningBlueRight extends LinearOpMode {
    Robot robot = new Robot(this);
    Camera camera = new Camera(this);

    OpenCvBlue.OpenCvPosition position = OpenCvBlue.OpenCvPosition.LEFT;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){
        robot.init();
        camera.initAutoBlue();
        position = camera.pipeline2.getAnalysis();
        waitForStart();
        runtime.reset();

        if(opModeIsActive()) {
            telemetry.addData("Pipeline", position);
            telemetry.update();

            if (position == OpenCvBlue.OpenCvPosition.RIGHT) {
                robot.drivetrain.driveForwards(0.7, 20);
                robot.drivetrain.strafeRight(0.7, 5);
                robot.drivetrain.driveBackwards(0.7, 20);
                robot.drivetrain.strafeLeft(0.7, 60);
                //robot.intake.outtake(0.7, 3);

            } else if (position == OpenCvBlue.OpenCvPosition.LEFT) {
                robot.drivetrain.driveForwards(0.7, 20);
                robot.drivetrain.strafeLeft(0.7, 5);
                robot.drivetrain.driveBackwards(0.7, 5);
                robot.drivetrain.strafeLeft(0.7, 60);
                //robot.intake.outtake(0.7, 3);

            } else {
                robot.drivetrain.driveForwards(0.7, 20);
                robot.drivetrain.driveBackwards(0.7, 5);
                robot.drivetrain.strafeLeft(0.7, 60);
                //robot.intake.outtake(0.7, 3);

            }
            robot.drivetrain.stopMotors();
            waitForStart();
        }

        //robot.drivetrain.strafeRight(0.7, 40);
        //robot.drivetrain.driveForwards(0.7, 20);
        //robot.drivetrain.stopMotors();

    }

}*/
