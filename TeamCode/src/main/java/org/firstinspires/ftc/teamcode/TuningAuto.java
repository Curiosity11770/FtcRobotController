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


@Autonomous(name="TuningAuto", group="Linear OpMode")
@Config
public class TuningAuto extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_TARGET,
        IDLE_ONE,
        DRIVE_TO_START,
        IDLE_TWO
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_TARGET;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

    // Define our target
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 90;
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
                case DRIVE_TO_TARGET:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached){
                        currentState = State.IDLE_ONE;
                        timer.reset();
                    }
                    break;
                case IDLE_ONE:
                    if(timer.seconds() > 2.0){
                        currentState = State.DRIVE_TO_START;
                        robot.drivetrain.setTargetPose(startPose);
                    }
                    break;
                case DRIVE_TO_START:
                    if(robot.drivetrain.targetReached){
                        currentState = State.IDLE_TWO;
                        timer.reset();
                    }
                    break;
                case IDLE_TWO:
                    if(timer.seconds() > 2){
                        currentState = State.DRIVE_TO_TARGET;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.DEGREES,targetHeading));
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