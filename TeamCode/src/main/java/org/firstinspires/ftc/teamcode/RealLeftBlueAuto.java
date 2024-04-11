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
public class RealLeftBlueAuto extends LinearOpMode {

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
        TOTRUSSBACK,
        UNDERTRUSSBACK,
        TODROP,
        DROP,
        IDLE
    }



    // We define the current state we're on
    // Default to IDLE
    State currentState = State.PURPLEPIXEL;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(12, 63.5, Math.toRadians(270));

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
                .lineToSplineHeading(new Pose2d(21, 24, Math.toRadians(180)))
                .addTemporalMarker(2, () -> {robot.intake.state = Intake.IntakeMode.OUTTAKE;})
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory yellowPixelCenter = drive.trajectoryBuilder(purplePixelCenter.end())
                .lineToConstantHeading(new Vector2d(48, 36))
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.LOW;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
                .build();

        // Let's define our trajectories
        Trajectory scoringTime = drive.trajectoryBuilder(yellowPixelCenter.end())
                .back(6.5)
                .addDisplacementMarker(10, ()-> {robot.scoring.gatesUp();})
                .build();
        Trajectory purplePixelLeft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(24, 35, Math.toRadians(270)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory yellowPixelLeft = drive.trajectoryBuilder(purplePixelLeft.end())
                .lineToConstantHeading(new Vector2d(48, 36))
                .build();
        Trajectory purplePixelRight = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(15, 24, Math.toRadians(180)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory yellowPixelRight = drive.trajectoryBuilder(purplePixelRight.end())
                .lineToConstantHeading(new Vector2d(48, 36))
                .build();

        /*Trajectory cycling = drive.trajectoryBuilder(purplePixelCenter.end())
                .splineToConstantHeading(new Vector2d(32, 61), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-36, 61))
                .splineTo(new Vector2d(-56, 40),  Math.toRadians(180))
                .build();*/

        Trajectory toTrussCenter = drive.trajectoryBuilder(yellowPixelCenter.end())
                .lineToSplineHeading(new Pose2d(32, 59, Math.toRadians(180)))
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = org.firstinspires.ftc.teamcode.Lift.LiftMode.INTAKE;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);})
                .build();

        Trajectory underTruss = drive.trajectoryBuilder(toTrussCenter.end())
                .lineToSplineHeading(new Pose2d(-52, 59, Math.toRadians(180)))
                .build();

        Trajectory toStack = drive.trajectoryBuilder(underTruss.end())
                .strafeTo(new Vector2d(-56, 33.5))
                .build();

        Trajectory intakeStack = drive.trajectoryBuilder(toStack.end())

                .forward(4,
                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.1, () -> {robot.intake.state = Intake.IntakeMode.INTAKE;})
                    .addTemporalMarker(2, () -> {robot.intake.state = Intake.IntakeMode.OUTTAKE;})
                .build();
        Trajectory toTrussBack = drive.trajectoryBuilder(intakeStack.end())
                .addDisplacementMarker(5, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .lineToSplineHeading(new Pose2d(-52, 59, Math.toRadians(180)))
                .build();

        Trajectory underTrussBack= drive.trajectoryBuilder(toTrussBack.end())
                .lineToSplineHeading(new Pose2d(32, 59, Math.toRadians(180)))
                .build();

        Trajectory toDrop = drive.trajectoryBuilder(underTrussBack.end())
                .lineToConstantHeading(new Vector2d(48, 36))
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.LOW;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
                .build();

        Trajectory dropNow = drive.trajectoryBuilder(toDrop.end())
                .back(6.5)
                .addDisplacementMarker(10, () -> {robot.intake.state = Intake.IntakeMode.OFF;})
                .addDisplacementMarker(10, () -> {robot.lift.liftMode = Lift.LiftMode.MIDDLE;})
                .addDisplacementMarker(10, () -> {robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);})
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
            drive.followTrajectoryAsync(purplePixelCenter);
            placement = 2;
        } else if (robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT){
            drive.followTrajectoryAsync(purplePixelLeft);
            placement = 1;
        } else {
            drive.followTrajectoryAsync(purplePixelRight);
            placement = 3;

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
                        drive.followTrajectoryAsync(scoringTime);
                        //drive.followTrajectoryAsync(toTrussCenter);
                    }
                    break;
                case SCORING:
                    if (!drive.isBusy()) {
                        currentState = State.TOTRUSS;
                        drive.followTrajectoryAsync(toTrussCenter);
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
                        currentState = State.TOTRUSSBACK;
                        drive.followTrajectoryAsync(toTrussBack);
                    }
                    break;
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
                        if(timesCycled == 1) {
                            currentState = State.IDLE;
                        } else {
                            timesCycled = 1;
                            currentState = State.TOTRUSS;
                            drive.followTrajectoryAsync(toTrussCenter);
                        }

                    }
                    break;

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
