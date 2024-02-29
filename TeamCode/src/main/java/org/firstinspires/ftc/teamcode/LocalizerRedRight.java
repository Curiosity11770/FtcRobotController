package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LocalizerRedRight extends LinearOpMode {
    private Robot robot;

    public static double oX = 0;
    public static double oY = 0;
    public static double oT = 0;


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
        robot.camera.visionPortalFront.stopStreaming();

        waitForStart();
        //Drives to Spike Mark
        if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE) {
            //Drive to Spike Mark
            robot.drivetrain.driveToPose(41, 6, -90, 3);
            //Outtake and go to pixel stack
            robot.driveStraightTime(-0.2, 1.5);
            robot.drivetrain.driveToPose(53, 15, 90, 3);
            robot.driveStraightIntake(0.2, 3);
            //Intake Pixels
            //robot.intake.outtake(-0.7, 3);
            //robot.drivetrain.driveStraightTime(-0.2, 2);
            //Stage Door
            //robot.drivetrain.driveToPose(55, 15, -90, 3);
            //Forward
            //robot.drivetrain.driveToPose(55, 15, -90, 3);
            //robot.drivetrain.driveToPose(55, -45, -90, 3);
            //ToAprilTags
            //robot.drivetrain.driveToPose(40, 45, -90, 3);
            //robot.driveToAprilTag(5, 5);
            //
        }

        //robot.drivetrain.driveStraightTime(-0.3, 2);
    }

}
