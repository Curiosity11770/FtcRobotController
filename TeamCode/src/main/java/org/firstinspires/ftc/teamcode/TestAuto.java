package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "TestAuto", group = "Concept")

public class TestAuto extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(20, 20, Math.toRadians(90)))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(30, 50), Math.toRadians(90))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(35, 35, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(20, 30, Math.toRadians(90)))
                .build();

        waitForStart();

        if (opModeIsActive()) {
            drive.followTrajectorySequence(traj);
        }
    }
}
