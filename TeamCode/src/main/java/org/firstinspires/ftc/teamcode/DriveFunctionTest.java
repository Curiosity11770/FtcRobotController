package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous (name="Drive Function Test", group = "Concept")
public class DriveFunctionTest extends LinearOpMode {
    private Robot robot;

    public static double xTarget = 24;
    public static double yTarget = 0;
    public static double thetaTarget = 0;

    public static double oX = 0;
    public static double oY = 0;
    public static double oT = 0;

    enum State{
        TARGET,
        ORIGIN,
        MANUAL
    }

    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        robot.drivetrain.localizer.setCoordinates(oX, oY, oT);

        while (!isStarted()) {
            robot.drivetrain.localizer.update();
            robot.drivetrain.localizer.updateDashboard();
            robot.camera.scanAprilTag(5);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        // robot.camera.stopColorProcessor();

        waitForStart();

        while(opModeIsActive()) {
            robot.drivetrain.driveToPose(xTarget, yTarget, thetaTarget);
            sleep(500);
            robot.drivetrain.driveToPose(0,0,0);
            sleep(500);

        }

        //try drive to pose
        //try drive to april tag
    }

}
