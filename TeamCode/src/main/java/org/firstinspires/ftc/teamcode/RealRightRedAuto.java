package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Config
@Autonomous(group = "advanced")
public class RealRightRedAuto extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        PURPLEPIXEL,
        YELLOWPIXEL,
        SCORING,
        TOTRUSS,
        UNDERTRUSS,
        TOSTACK,
        CYCLING,
        INTAKESTACK,
        BACKUP,
        INTAKESTACK2,
        TOTRUSSBACK,
        UNDERTRUSSBACK,
        TODROP,
        DROP,
        UP,
        PARK,
        STRAFE,
        IDLE
    }



    // We define the current state we're on
    // Default to IDLE
    State currentState = State.PURPLEPIXEL;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our robot
        Robot robot = new Robot(this);

        robot.init();

        int placement = 0;
        int timesCycled = 0;

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory purplePixelCenter = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(23, -24, Math.toRadians(180)))
                .addDisplacementMarker(50, () -> {robot.intake.state = Intake.IntakeMode.OUTTAKE;})
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory yellowPixelCenter = drive.trajectoryBuilder(purplePixelCenter.end())
                .lineToConstantHeading(new Vector2d(48, -36))
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.LOW;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
                .build();

        // Let's define our trajectories
        Trajectory purplePixelRight = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(24, -38, Math.toRadians(90)))
                .addDisplacementMarker(50, () -> {robot.intake.state = Intake.IntakeMode.OUTTAKE;})
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one

        Trajectory purplePixelLeft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(9, -28, Math.toRadians(180)))
                .addDisplacementMarker(40, () -> {robot.intake.state = Intake.IntakeMode.OUTTAKE;})
                .build();

        Trajectory yellowPixelLeft = drive.trajectoryBuilder(purplePixelLeft.end())
                .lineToSplineHeading(new Pose2d(48, -28, Math.toRadians(180)))
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.LOW;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
                .build();

        Trajectory purplePixelRight2 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, -22, Math.toRadians(180)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory yellowPixelRight = drive.trajectoryBuilder(purplePixelRight.end())
                .lineToSplineHeading(new Pose2d(48, -42, Math.toRadians(180)))
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.LOW;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
                .build();

        /*Trajectory cycling = drive.trajectoryBuilder(purplePixelCenter.end())
                .splineToConstantHeading(new Vector2d(32, 61), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-36, 61))
                .splineTo(new Vector2d(-56, 40),  Math.toRadians(180))
                .build();*/

        Trajectory scoringTime1  = drive.trajectoryBuilder(yellowPixelCenter.end())
                .back(6.5)
                .addDisplacementMarker(10, ()-> {robot.scoring.gatesUp();})
                .build();
        Trajectory scoringTime2  =drive.trajectoryBuilder(yellowPixelLeft.end())
                .back(6.5)
                .addDisplacementMarker(10, ()-> {robot.scoring.gatesUp();})
                .build();
        Trajectory scoringTime3  =drive.trajectoryBuilder(yellowPixelRight.end())
                .back(6.5)
                .addDisplacementMarker(10, ()-> {robot.scoring.gatesUp();})
                .build();

        Trajectory toTrussCenter = drive.trajectoryBuilder(scoringTime1.end())
                .lineToSplineHeading(new Pose2d(32, -59+timesCycled*3, Math.toRadians(180)))
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = org.firstinspires.ftc.teamcode.Lift.LiftMode.INTAKE;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);})
                .build();

        Trajectory underTruss = drive.trajectoryBuilder(toTrussCenter.end())
                .lineToSplineHeading(new Pose2d(-52, -59+timesCycled*3, Math.toRadians(180)))
                .build();

        Trajectory toStack = drive.trajectoryBuilder(underTruss.end())
                .strafeTo(new Vector2d(-55.5, -34))
                .build();

        Trajectory intakeStack = drive.trajectoryBuilder(toStack.end())

                .forward(12,
                        SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0, () -> {robot.intake.state = Intake.IntakeMode.INTAKE;})
                .addTemporalMarker(1, () -> {sleep(1000);})
                .addDisplacementMarker(12, () -> {robot.intake.state = Intake.IntakeMode.OUTTAKE;})
                .build();
        Trajectory backUp = drive.trajectoryBuilder(intakeStack.end())
                .back(12)
                .build();
        /*Trajectory intakeStack2 = drive.trajectoryBuilder(toStack.end())

                .forward(6,
                        SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(0, () -> {robot.intake.state = Intake.IntakeMode.INTAKE;})
                .addDisplacementMarker(14, () -> {robot.intake.state = Intake.IntakeMode.OUTTAKE;})
                .build();*/
        Trajectory toTrussBack = drive.trajectoryBuilder(backUp.end())
                .addDisplacementMarker(0, () -> {robot.scoring.gatesDown();})
                .addDisplacementMarker(5, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .lineToSplineHeading(new Pose2d(-52, -59+timesCycled*3, Math.toRadians(180)))
                .build();

        Trajectory underTrussBack= drive.trajectoryBuilder(toTrussBack.end())
                .lineToSplineHeading(new Pose2d(32, -60.5+timesCycled*3, Math.toRadians(180)))
                .build();

        Trajectory toDrop = drive.trajectoryBuilder(underTrussBack.end())
                .lineToConstantHeading(new Vector2d(48, -36))
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.LOW;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
                .build();

        Trajectory dropNow = drive.trajectoryBuilder(toDrop.end())
                .back(6.5, SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.MIDDLE;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
                .addDisplacementMarker(10, ()-> {robot.scoring.gatesUp();})
                .build();
        Trajectory up = drive.trajectoryBuilder(dropNow.end())
                .strafeLeft(5,
                        SampleMecanumDrive.getVelocityConstraint(6, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(1, () -> {robot.lift.liftMode = Lift.LiftMode.HIGH;})
                .build();
        Trajectory back =  drive.trajectoryBuilder(up.end())
                .forward(2)
                .build();
        Trajectory strafe1 = drive.trajectoryBuilder(yellowPixelCenter.end())
                .strafeRight(30)
                .build();
        Trajectory strafe2 = drive.trajectoryBuilder(yellowPixelCenter.end())
                .strafeLeft(30)
                .build();
        Trajectory strafe3 = drive.trajectoryBuilder(yellowPixelCenter.end())
                .strafeRight(30)
                .build();
        Trajectory park = drive.trajectoryBuilder(strafe2.end())
                .back(15)
                .build();

        // Define the angle to turn at
        //double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        /*Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
                .lineToConstantHeading(new Vector2d(-15, 0))
                .build();*/

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();


        // Define the angle for turn 2
        //double turnAngle2 = Math.toRadians(720);

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        if (robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE) {
            placement = 2;
            drive.followTrajectoryAsync(purplePixelCenter);
        } else if (robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT){
            placement = 1;
            drive.followTrajectoryAsync(purplePixelLeft);
        } else {
            placement = 3;
            drive.followTrajectoryAsync(purplePixelRight);

        }

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case PURPLEPIXEL:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        //robot.intake.outtake(-0.7);
                        currentState = State.YELLOWPIXEL;
                        if(placement == 2) {
                            drive.followTrajectoryAsync(yellowPixelCenter);
                        } else if(placement == 1){
                            drive.followTrajectoryAsync(yellowPixelLeft);
                        } else {
                            drive.followTrajectoryAsync(yellowPixelRight);
                        }
                    }
                    break;
                case YELLOWPIXEL:
                    if (!drive.isBusy()) {
                        currentState = State.SCORING;
                        if(placement == 2){
                            drive.followTrajectoryAsync(scoringTime1);
                        } else if (placement == 1){
                            drive.followTrajectoryAsync(scoringTime2);
                        } else {
                            drive.followTrajectoryAsync(scoringTime3);
                        }
                        //drive.followTrajectoryAsync(toTrussCenter);
                    }
                    break;
                case SCORING:
                    if (!drive.isBusy()) {
                        currentState = State.TOTRUSS;
                        drive.followTrajectoryAsync(toTrussCenter);
                        /*currentState = State.STRAFE;
                        if(placement == 2){
                            drive.followTrajectoryAsync(strafe1);
                        } else if (placement == 1){
                            drive.followTrajectoryAsync(strafe2);
                        } else {
                            drive.followTrajectoryAsync(strafe3);
                        }*/
                    }
                    break;

                case TOTRUSS:
                    if (!drive.isBusy()) {
                        currentState = State.UNDERTRUSS;
                        drive.followTrajectoryAsync(underTruss);
                    }
                    break;

                case UNDERTRUSS:
                    if (!drive.isBusy()) {
                        currentState = State.TOSTACK;
                        drive.followTrajectoryAsync(toStack);
                    }
                    break;

                case TOSTACK:
                    if (!drive.isBusy()) {
                        currentState = State.INTAKESTACK;
                        drive.followTrajectoryAsync(intakeStack);
                    }
                    break;
                case INTAKESTACK:
                    if (robot.intake.state == Intake.IntakeMode.OUTTAKE) {
                        currentState = State.BACKUP;
                        drive.followTrajectoryAsync(backUp);
                    }
                    break;
                case BACKUP:
                    if (!drive.isBusy()) {
                        currentState = State.TOTRUSSBACK;
                        drive.followTrajectoryAsync(toTrussBack);
                    }
                /*case INTAKESTACK2:
                    if (robot.intake.state == Intake.IntakeMode.OUTTAKE) {
                        currentState = State.TOTRUSSBACK;
                        drive.followTrajectoryAsync(toTrussBack);
                    }
                    break;*/
                case TOTRUSSBACK:
                    if (!drive.isBusy()) {
                        currentState = State.UNDERTRUSSBACK;
                        drive.followTrajectoryAsync(underTrussBack);
                    }
                    break;
                case UNDERTRUSSBACK:
                    if (!drive.isBusy()) {
                        currentState = State.TODROP;
                        drive.followTrajectoryAsync(toDrop);
                    }
                    break;
                case TODROP:
                    if (!drive.isBusy()) {
                        currentState = State.DROP;
                        drive.followTrajectoryAsync(dropNow);
                    }
                    break;
                case DROP:
                    if (!drive.isBusy()) {
                        //if(timesCycled == 1) {
                        currentState = State.STRAFE;
                        drive.followTrajectoryAsync(strafe2);
                        //} else {
                        timesCycled = 1;
                        //currentState = State.TOTRUSS;
                        //drive.followTrajectoryAsync(toTrussCenter);
                        // }

                    }
                    break;
                case STRAFE:
                    if (!drive.isBusy()) {
                        //if(timesCycled == 1) {
                        currentState = State.PARK;
                        drive.followTrajectoryAsync(park);
                    }
                case PARK:
                    if (!drive.isBusy()) {
                        //if(timesCycled == 1) {
                        currentState = State.IDLE;
                        //drive.followTrajectoryAsync(back);
                    }
                case IDLE :

            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            //lift.update();
            robot.lift.update();

            robot.intake.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("state", currentState);
            telemetry.update();
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
}
