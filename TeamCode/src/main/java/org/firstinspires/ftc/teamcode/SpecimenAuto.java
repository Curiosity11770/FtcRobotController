package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name="SpecimenAuto", group="Linear OpMode")
@Config
public class SpecimenAuto extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_CHAMBER,
        LIFT,
        DRIVE_SLOWLY,
        SCORE,
        OPEN,
        BACK,
        FORWARD,
        PICKUP,
        DRIVE_BACK,
        LIFT2,
        DRIVE_SLOWLY2,
        SCORE2,
        OPEN2,
        PARK,
        IDLE

    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_CHAMBER;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

    // Define our target
    public static double targetX = 24;
    public static double targetY = 5;
    public static double targetHeading = 0;
    Pose2D targetPose = new Pose2D(DistanceUnit.INCH, targetX,targetY, AngleUnit.DEGREES, targetHeading);

    @Override
    public void runOpMode() {
        //calling constructor
        robot = new Robot(this);


        //calling init function
        robot.init();

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        // robot.drivetrain.localizer.myOtos.setPosition(startPose);

        //Set the drivetrain's first target
        robot.drivetrain.setTargetPose(targetPose);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case DRIVE_TO_CHAMBER:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached){
                        currentState = State.LIFT;
                        timer.reset();
                    }
                    break;
                case LIFT:
                    robot.lift.liftToPositionPIDClass(1400);
                    robot.scoring.scoringPivot.setPosition(0.2);
                    //robot.scoring.clawWrist.setPosition(robot.scoring.CLAW_DOWN);
                    if(timer.seconds() > 2.0){
                        currentState = State.DRIVE_SLOWLY;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 28, 5, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawWrist.setPosition(0.88);
                    if(robot.drivetrain.targetReached || timer.seconds() > 3){
                        currentState = State.SCORE;
                        timer.reset();
                    }
                    break;
                case SCORE:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(600);
                    robot.scoring.scoringPivot.setPosition(0.2);
                    //robot.lift.liftToPositionPIDClass(400);
                    //robot.scoring.clawServo.setPosition(0.8);
                    if(timer.seconds() > 4.5){
                        currentState = State.OPEN;
                        timer.reset();
                    }
                    break;
                case OPEN:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.lift.liftToPositionPIDClass(600);
                    robot.scoring.clawServo.setPosition(0.8);
                    if(timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -32, AngleUnit.DEGREES, 180));
                        currentState = State.BACK;
                        timer.reset();
                    }
                    break;
                case BACK:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.clawServo.setPosition(0.8);
                    robot.lift.liftToPositionPIDClass(0);
                    robot.scoring.scoringPivot.setPosition(0.47);
                    //robot.scoring.clawWrist.setPosition(0.5);
                    if(timer.seconds() > 2.0){
                        currentState = State.FORWARD;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1, -32, AngleUnit.DEGREES, 180));
                        timer.reset();
                    }
                    break;
                case FORWARD:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.clawServo.setPosition(0.8);
                    //robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.clawWrist.setPosition(0.5);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        currentState = State.PICKUP;
                        timer.reset();
                    }
                    break;
                case PICKUP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(0.32);
                    if(timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 24, 12, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_BACK;
                        timer.reset();
                    }
                    break;
                case DRIVE_BACK:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(1600);
                    robot.scoring.scoringPivot.setPosition(0.2);
                    robot.scoring.clawServo.setPosition(0.32);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 29.5, 12, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY2;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(1200);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        currentState = State.SCORE2;
                        timer.reset();
                    }
                    break;
                case SCORE2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(700);
                    robot.scoring.scoringPivot.setPosition(0.2);
                    if(timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 29.5, 12, AngleUnit.DEGREES, 0));
                        currentState = State.OPEN;
                        timer.reset();
                    }
                    break;
                case OPEN2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.lift.liftToPositionPIDClass(600);
                    robot.scoring.clawServo.setPosition(0.8);
                    if(timer.seconds() > 2.0){
                        //robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -32, AngleUnit.DEGREES, 180));
                        currentState = State.BACK;
                        timer.reset();
                    }
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();

            telemetry.addData("state", currentState);
            telemetry.update();

        }
    }

}
