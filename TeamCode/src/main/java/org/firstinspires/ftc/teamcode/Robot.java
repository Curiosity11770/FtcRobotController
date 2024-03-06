package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    public boolean passed, notPassed;
    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;

    public Lift lift;

    public Intake intake;

    public Scoring scoring;

    public Drone drone;

    public DualPortalCameras camera;

    //Variables for AprilTag Motion --> eventually reconcile these with drive constants in Drivetrain class
    final double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;   //  Clip the turn speed to this max value (adjust for your robot)

    public Robot(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init(){
        drivetrain = new Drivetrain(myOpMode);
        lift = new Lift(myOpMode);
        intake = new Intake(myOpMode);
        scoring = new Scoring(myOpMode);
        drone = new Drone(myOpMode);
        camera = new DualPortalCameras(myOpMode);

        myOpMode.telemetry.addData("IsWorking", drone);

        drivetrain.init();
        lift.init();
        intake.init();
        scoring.init();
        drone.init();
        camera.init();
    }
    public void initTeleOp(){
        drivetrain = new Drivetrain(myOpMode);
        lift = new Lift(myOpMode);
        intake = new Intake(myOpMode);
        scoring = new Scoring(myOpMode);
        drone = new Drone(myOpMode);
        camera = new DualPortalCameras(myOpMode);

        myOpMode.telemetry.addData("IsWorking", drone);

        drivetrain.init();
        lift.init();
        intake.init();
        scoring.init();
        drone.init();
        camera.initTeleOp();

    }

    public void teleOp(){
        drivetrain.teleOp();
        lift.teleOp();
        intake.teleOp();
        scoring.teleOp(passed, notPassed);
        drone.teleOp();

        if(drivetrain.state == Drivetrain.DriveMode.APRILTAGS){
            driveToAprilTagTeleOp(drivetrain.AprilTagTarget, 8);
        }

        if(intake.frontPixel && intake.backPixel){
           scoring.rightGateServo.setPosition(scoring.GATE_DOWN_RIGHT);
            scoring.leftGateServo.setPosition(scoring.GATE_DOWN_LEFT);
        }

        if(myOpMode.gamepad2.dpad_right){
            lift.liftMode = Lift.LiftMode.LOW;
            scoring.state = Scoring.ScoringMode.SCORING;
            scoring.timer.reset();
            if(scoring.timer.seconds() > 1 && scoring.state == Scoring.ScoringMode.SCORING){
                scoring.boxServo.setPosition(scoring.BOX_OUT);
            }

        }

        if(myOpMode.gamepad2.dpad_down){
            lift.liftMode = Lift.LiftMode.INTAKE;
        }

    }

    void driveToAprilTag(int targetTag, double targetDistance) {
        double rangeError = 100;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

        while (rangeError > 1 && myOpMode.opModeIsActive()) {
            camera.scanAprilTag(targetTag);
            if (camera.targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError = (camera.desiredTag.ftcPose.range - targetDistance);
                double headingError = camera.desiredTag.ftcPose.bearing;
                double yawError = camera.desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)*-1;
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // Calculate wheel powers.
                double leftFrontPower = drive - strafe - turn;
                double rightFrontPower = drive + strafe + turn;
                double leftBackPower = drive + strafe - turn;
                double rightBackPower = drive - strafe + turn;

                // Normalize wheel powers to be less than 1.0
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Send powers to the wheels.
                drivetrain.driveFrontLeft.setPower(leftFrontPower);
                drivetrain.driveFrontRight.setPower(rightFrontPower);
                drivetrain.driveBackLeft.setPower(leftBackPower);
                drivetrain.driveBackRight.setPower(rightBackPower);
            }else{
                drivetrain.stopMotors();
            }
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.update();
        }

        drivetrain.stopMotors();
    }

    public void driveStraightOuttake(double power, double time){
        ElapsedTime t = new ElapsedTime();
        intake.intakeRight.setPower(-0.7);
        intake.intakeLeft.setPower(0.7);
        //myOpMode.sleep(500);
        drivetrain.driveFrontLeft.setPower(power);
        drivetrain.driveFrontRight.setPower(power);
        drivetrain.driveBackLeft.setPower(power);
        drivetrain.driveBackRight.setPower(power);
        t.reset();
        while(myOpMode.opModeIsActive() && t.seconds() < time){
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.addData("Motor Power", power);
            myOpMode.telemetry.addData("Time Target", time);
            myOpMode.telemetry.addData("Time Elapsed", t.seconds());
        }
        intake.intakeRight.setPower(0);
        intake.intakeLeft.setPower(0);
        drivetrain.driveFrontLeft.setPower(0);
        drivetrain.driveFrontRight.setPower(0);
        drivetrain.driveBackLeft.setPower(0);
        drivetrain.driveBackRight.setPower(0);
    }
    public void driveStraightTime(double power, double time){
        ElapsedTime t = new ElapsedTime();
        drivetrain.driveFrontLeft.setPower(power);
        drivetrain.driveFrontRight.setPower(power);
        drivetrain.driveBackLeft.setPower(power);
        drivetrain.driveBackRight.setPower(power);
        t.reset();
        while(myOpMode.opModeIsActive() && t.seconds() < time){
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.addData("Motor Power", power);
            myOpMode.telemetry.addData("Time Target", time);
            myOpMode.telemetry.addData("Time Elapsed", t.seconds());
        }
        intake.intakeRight.setPower(0);
        intake.intakeLeft.setPower(0);
        drivetrain.driveFrontLeft.setPower(0);
        drivetrain.driveFrontRight.setPower(0);
        drivetrain.driveBackLeft.setPower(0);
        drivetrain.driveBackRight.setPower(0);
    }

    public void driveStraightStrafe(double power, double time){
        ElapsedTime t = new ElapsedTime();
        //myOpMode.sleep(500);
        drivetrain.driveFrontLeft.setPower(-power);
        drivetrain.driveFrontRight.setPower(power);
        drivetrain.driveBackLeft.setPower(power);
        drivetrain.driveBackRight.setPower(-power);
        t.reset();
        while(myOpMode.opModeIsActive() && t.seconds() < time){
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.addData("Motor Power", power);
            myOpMode.telemetry.addData("Time Target", time);
            myOpMode.telemetry.addData("Time Elapsed", t.seconds());
        }
        drivetrain.driveFrontLeft.setPower(0);
        drivetrain.driveFrontRight.setPower(0);
        drivetrain.driveBackLeft.setPower(0);
        drivetrain.driveBackRight.setPower(0);
    }

    public void driveStraightIntake(double power, double time){

        ElapsedTime t = new ElapsedTime();
        intake.intakeRight.setPower(0.7);
        intake.intakeLeft.setPower(-0.7);
        intake.intakeMotor.setPower(0.7);
        myOpMode.sleep(500);
        drivetrain.driveFrontLeft.setPower(power);
        drivetrain.driveFrontRight.setPower(power);
        drivetrain.driveBackLeft.setPower(power);
        drivetrain.driveBackRight.setPower(power);
        t.reset();
        scoring.leftGateServo.setPosition(scoring.GATE_UP_LEFT);
        scoring.rightGateServo.setPosition(scoring.GATE_UP_RIGHT);
        while(myOpMode.opModeIsActive() && t.seconds() < time && (!intake.frontPixel || !intake.backPixel)){
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.addData("Motor Power", power);
            myOpMode.telemetry.addData("Time Target", time);
            myOpMode.telemetry.addData("Time Elapsed", t.seconds());

            myOpMode.telemetry.addData("frontPixel", intake.frontPixel);
            myOpMode.telemetry.addData("frontPixel", intake.backPixel);
            myOpMode.telemetry.addData("colorFront", ((DistanceSensor) intake.colorFront).getDistance(DistanceUnit.CM));
            myOpMode.telemetry.addData("colorBack", ((DistanceSensor) intake.colorBack).getDistance(DistanceUnit.CM));
            if(((DistanceSensor) intake.colorFront).getDistance(DistanceUnit.CM) <= 4){
                intake.frontPixel = true;
            }else{
                intake.frontPixel = false;

            }
            if(((DistanceSensor) intake.colorBack).getDistance(DistanceUnit.CM) <= 2){
                intake.backPixel = true;
            }else{
                intake.backPixel = false;
            }
            if(intake.frontPixel && intake.backPixel){
                scoring.rightGateServo.setPosition(scoring.GATE_DOWN_RIGHT);
                scoring.leftGateServo.setPosition(scoring.GATE_DOWN_LEFT);
            }
        }
        t.reset();
        intake.intakeRight.setPower(-0.7);
        intake.intakeLeft.setPower(0.7);
        intake.intakeMotor.setPower(-0.7);
        drivetrain.driveFrontLeft.setPower(-power);
        drivetrain.driveFrontRight.setPower(-power);
        drivetrain.driveBackLeft.setPower(-power);
        drivetrain.driveBackRight.setPower(-power);
        while(myOpMode.opModeIsActive() && t.seconds() < time-1){
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.addData("Motor Power", power);
            myOpMode.telemetry.addData("Time Target", time);
            myOpMode.telemetry.addData("Time Elapsed", t.seconds());
        }
        intake.intakeRight.setPower(0);
        intake.intakeLeft.setPower(0);
        intake.intakeMotor.setPower(0);
        drivetrain.driveFrontLeft.setPower(0);
        drivetrain.driveFrontRight.setPower(0);
        drivetrain.driveBackLeft.setPower(0);
        drivetrain.driveBackRight.setPower(0);
    }

    void driveToAprilTagTeleOp(int targetTag, double targetDistance) {
        double rangeError = 0;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

            camera.scanAprilTag(targetTag);
            if (camera.targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError = (camera.desiredTag.ftcPose.range - targetDistance);
                double headingError = camera.desiredTag.ftcPose.bearing;
                double yawError = camera.desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)*-1;
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // Calculate wheel powers.
                double leftFrontPower = drive - strafe - turn;
                double rightFrontPower = drive + strafe + turn;
                double leftBackPower = drive + strafe - turn;
                double rightBackPower = drive - strafe + turn;

                // Normalize wheel powers to be less than 1.0
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Send powers to the wheels.
                drivetrain.driveFrontLeft.setPower(leftFrontPower);
                drivetrain.driveFrontRight.setPower(rightFrontPower);
                drivetrain.driveBackLeft.setPower(leftBackPower);
                drivetrain.driveBackRight.setPower(rightBackPower);
            }else{
                drivetrain.stopMotors();
            }
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.update();

       // drivetrain.stopMotors();
    }

    /*public void touchSense(){
        if(!lift.getTouch()){
            intake.intakeLeft.setPower(0);
            intake.intakeRight.setPower(0);
            intake.intakeMotor.setPower(0);
        }
    }*/
}
