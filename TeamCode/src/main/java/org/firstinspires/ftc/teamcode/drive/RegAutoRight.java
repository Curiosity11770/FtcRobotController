package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RegAutoRight extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        waitForStart();


        while(opModeIsActive()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            double targetX = 24;
            double targetY = 0;

            double xError = targetX - poseEstimate.getX();
            double yError = targetY - poseEstimate.getY();
            double theta = Math.atan2(yError,xError);
            // 0 is the reference because we want the distance to go to 0
            double distance = Math.hypot(xError, yError);
            double f = drive.drivePID.calculate(0,-distance);
            double t = drive.headingPID.calculate(theta, poseEstimate.getHeading());
            double left_power = f + t;
            double right_power = f - t;

            drive.leftFront.setPower(left_power);
            drive.leftRear.setPower(left_power);
            drive.rightFront.setPower(right_power);
            drive.rightRear.setPower(right_power);

            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }

    }
}
