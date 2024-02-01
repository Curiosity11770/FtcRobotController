package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (name="RedRightAuto", group = "Concept")
public class RedRightAuto extends LinearOpMode {
    private Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        robot.drivetrain.localizer.setCoordinates(12, 66, Math.PI/2);

        while (!isStarted()) {
            robot.camera.scanAprilTag(5);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        robot.camera.visionPortalFront.stopStreaming();
        // robot.camera.stopColorProcessor();

        waitForStart();

        if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE){
            robot.drivetrain.driveStraightPID(24, 3);
            runtime.reset();
            while(runtime.seconds() > 3){
                robot.intake.outtake(0.8);
            }
            robot.drivetrain.encoderTurn(4000, 3);
            robot.driveToAprilTag(2, 3);
           // robot.l
        }

        robot.drivetrain.encoderTurn(-5000, 5);

        //try drive to pose
        //try drive to april tag
    }

}
