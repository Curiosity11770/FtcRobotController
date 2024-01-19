package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.SimpleVisionProcessor;

@Autonomous(name = "CameraScanningBlueLeft", group = "Linear OpMode")
public class CameraScanningBlueLeft extends LinearOpMode {
    Robot robot = new Robot(this);
    Camera camera = new Camera(this);
    SimpleVisionProcessor.Selected position;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init();
        camera.init();
        waitForStart();
        runtime.reset();
        telemetry.addData("DIRECTION", position);
        position = camera.returnSelection();
        if(position == SimpleVisionProcessor.Selected.RIGHT){;
        } else if (position == SimpleVisionProcessor.Selected.LEFT){
            robot.drivetrain.driveToPose(10, 10, 90);
            robot.drivetrain.driveToPose(20, 20, 90);
        } else {
            robot.drivetrain.driveToPose(30, 30, 90);
        }
        telemetry.addData("POSITION", robot.drivetrain.driveFrontRight.getCurrentPosition());

        robot.drivetrain.stopMotors();

        if (opModeIsActive()) {
            telemetry.update();
        }
    }
}
