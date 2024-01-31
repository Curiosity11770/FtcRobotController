package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous (name="Drive Function Test", group = "Concept")
public class DriveFunctionTest extends LinearOpMode {
    private Robot robot;

    enum State{
        TARGET,
        ORIGIN,
        MANUAL
    }

    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        robot.drivetrain.localizer.setCoordinates(0, 0, 0);

        while (!isStarted()) {
            robot.camera.scanAprilTag(5);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        // robot.camera.stopColorProcessor();

        waitForStart();

        robot.drivetrain.driveStraightProfiledPID(24);
        //try drive to pose
        //try drive to april tag
    }

}
