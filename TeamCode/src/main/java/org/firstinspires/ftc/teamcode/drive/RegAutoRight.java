package org.firstinspires.ftc.teamcode.drive;


import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Config
@Autonomous
public class RegAutoRight extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleTankDrive robot = new SampleTankDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        MotionProfile path1 = new MotionProfile(robot, 36, 10, true);
        timer.reset();
        while(timer.seconds() < path1.totalTime() + 0.5){
            robot.followPath(path1, timer.seconds());

            telemetry.addData("last theta", robot.lastTheta);

            telemetry.addData("theta", robot.theta);
            //telemetry.addData("angle to target", Math.abs(robot.lastTheta - robot.theta));
            telemetry.addData("forward", path1.forward);
            telemetry.addData("wrapped heading", robot.angleWrap(robot.getPoseEstimate().getHeading()));
            telemetry.addData("f mult", Math.cos(Range.clip(robot.headingPID.error, -PI/2, PI/2)));
            telemetry.addData("heading error", robot.headingPID.error);
            telemetry.addData("distance", path1.distance);
            telemetry.addData("current distance", robot.currentDistance);
            telemetry.addData("f", robot.f);
            telemetry.addData("t", robot.t);
            telemetry.addData("seconds", timer.seconds());
            telemetry.addData("total time", path1.totalTime());
            telemetry.addData("target", -path1.distance+path1.calculate(time));
            telemetry.update();
        }

        /*
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

        */


    }
}
