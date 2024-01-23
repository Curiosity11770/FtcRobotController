package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.SimpleVisionProcessor;

@Autonomous(name = "CameraScanningRedLeft", group = "Linear OpMode")
public class CameraScanningRedLeft extends LinearOpMode {
    Robot robot = new Robot(this);
    //Camera camera = new Camera(this);
    SimpleVisionProcessor.Selected position;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init();
        //camera.init();
        //waitForStart();
        runtime.reset();
        telemetry.addData("DIRECTION", position);
        if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE || robot.camera.returnSelection() == SimpleVisionProcessor.Selected.RIGHT || robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT) {
            position = robot.camera.returnSelection();
            //robot.camera.stopColorStreaming();
        }
        while (!isStarted()) {
            telemetry.addData("DIRECTION", position);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        waitForStart();


        if (opModeIsActive()) {

            if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE){
                robot.drivetrain.driveToPose(34, 0, 0);
                runtime.reset();
                while(runtime.seconds() < 3) {
                    robot.intake.outtake(-0.7);
                }
                robot.drivetrain.driveToPose(5, 0, 0);
                robot.drivetrain.driveToPose(5, -90, 0);
            } else if (robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT){
                //robot.drivetrain.driveToPose(34, 0, 0);
                robot.drivetrain.driveToPose(2, 0, 0);
                robot.drivetrain.driveToPose(2, -90, 0);
                /*runtime.reset();
                while(runtime.seconds() < 3) {
                    robot.intake.outtake(-0.7);
                }
                robot.drivetrain.driveToPose(5, 7, 0);
                robot.drivetrain.driveToPose(5, -70, 0);*/
            } else {
                robot.drivetrain.driveToPose(2, 0, 0);
                robot.drivetrain.driveToPose(2, -90, 0);
                /*runtime.reset();
                while(runtime.seconds() < 3) {
                    robot.intake.outtake(-0.7);
                }
                robot.drivetrain.driveToPose(5, -7, 0);
                robot.drivetrain.driveToPose(30, -40, 0);*/
            }
            //telemetry.addData("POSITION", robot.drivetrain.driveFrontRight.getCurrentPosition());

            robot.drivetrain.stopMotors();

            telemetry.update();

        }
    }
}