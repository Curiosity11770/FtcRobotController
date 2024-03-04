package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Drivetrain {
    private LinearOpMode myOpMode = null;

   public Localizer localizer;

    public DcMotor driveFrontLeft = null;
    public DcMotor driveFrontRight = null;
    public DcMotor driveBackLeft = null;
    public DcMotor driveBackRight = null;

    public static double HEADING_KP = 0.9;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static double DRIVE_KP = 0.07;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.01;//0.0003;g
    public static double DRIVE_MAX_ACC = 200;
    public static double DRIVE_MAX_VEL = 350;
    public static double DRIVE_MAX_OUT = 0.5;

    public int AprilTagTarget = 1;

    PIDController xPID;
    PIDController yPID;
    PIDController headingPID;

    public DriveMode state = DriveMode.MANUAL;

    public enum DriveMode{
        MANUAL,
        APRILTAGS
    }

    public Drivetrain(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init(){

        xPID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        yPID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

        xPID.maxOut = DRIVE_MAX_OUT;
        yPID.maxOut = DRIVE_MAX_OUT;
        headingPID.maxOut = DRIVE_MAX_OUT;

        driveFrontLeft = myOpMode.hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = myOpMode.hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackLeft = myOpMode.hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackRight = myOpMode.hardwareMap.get(DcMotor.class, "driveBackRight");

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);


        localizer = new Localizer(myOpMode);

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders(){
        localizer.centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localizer.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localizer.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders(){
        localizer.centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        localizer.leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        localizer.rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleOp() {

        if(Math.abs(myOpMode.gamepad1.left_stick_y) > 0.2||
                Math.abs(myOpMode.gamepad1.right_stick_x) > 0.2 ||
                Math.abs(myOpMode.gamepad1.left_stick_x) > 0.2||
                Math.abs(myOpMode.gamepad1.right_stick_y) > 0.2) {
            state = DriveMode.MANUAL;
        }else if(myOpMode.gamepad1.dpad_left){
            state = DriveMode.APRILTAGS;
            AprilTagTarget = 1;
        }else if(myOpMode.gamepad1.dpad_up){
            state = DriveMode.APRILTAGS;
            AprilTagTarget = 2;
        } else if(myOpMode.gamepad1.dpad_right){
            state = DriveMode.APRILTAGS;
            AprilTagTarget = 3;
        }


        if(state == DriveMode.MANUAL) {
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            double timesFactor;

            double drive = -myOpMode.gamepad1.left_stick_y;
            double turn = myOpMode.gamepad1.right_stick_x;
            double strafe = -myOpMode.gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 2);

            frontLeftPower = (drive + turn - strafe) / denominator;
            frontRightPower = (drive - turn + strafe) / denominator;
            backLeftPower = (drive + turn + strafe) / denominator;
            backRightPower = (drive - turn - strafe) / denominator;

            driveFrontLeft.setPower(frontLeftPower);
            driveFrontRight.setPower(frontRightPower);
            driveBackLeft.setPower(backLeftPower);
            driveBackRight.setPower(backRightPower);
            if (myOpMode.gamepad1.left_trigger > 0.2) {
            driveFrontLeft.setPower(frontLeftPower * 1.25);
            driveFrontRight.setPower(frontRightPower * 1.25);
            driveBackLeft.setPower(backLeftPower * 1.25);
            driveBackRight.setPower(backRightPower * 1.25);
        }
        else if (myOpMode.gamepad1.right_trigger > 0.2){
            driveFrontLeft.setPower(frontLeftPower/2);
            driveFrontRight.setPower(frontRightPower/2);
            driveBackLeft.setPower(backLeftPower/2);
            driveBackRight.setPower(backRightPower/2);
         }

            //if
            myOpMode.telemetry.addData("power", frontLeftPower);
            //myOpMode.telemetry.addData(frontLeftPower*1.25);
        } else if(state == DriveMode.APRILTAGS){

        }

       myOpMode.telemetry.addData("Counts", localizer.leftEncoder.getCurrentPosition()/localizer.COUNTS_PER_INCH);
        myOpMode.telemetry.addData("Counts", localizer.rightEncoder.getCurrentPosition()/localizer.COUNTS_PER_INCH);
        myOpMode.telemetry.addData("Counts", localizer.centerEncoder.getCurrentPosition()/localizer.TRACK_WIDTH);

    }

    public void fieldTeleOp(){
        if(Math.abs(myOpMode.gamepad1.left_stick_y) > 0.2||
                Math.abs(myOpMode.gamepad1.right_stick_x) > 0.2 ||
                Math.abs(myOpMode.gamepad1.left_stick_x) > 0.2||
                Math.abs(myOpMode.gamepad1.right_stick_y) > 0.2) {
            state = DriveMode.MANUAL;
        }else if(myOpMode.gamepad1.dpad_left){
            state = DriveMode.APRILTAGS;
            AprilTagTarget = 1;
        }else if(myOpMode.gamepad1.dpad_up){
            state = DriveMode.APRILTAGS;
            AprilTagTarget = 2;
        } else if(myOpMode.gamepad1.dpad_right){
            state = DriveMode.APRILTAGS;
            AprilTagTarget = 3;
        }


        if(state == DriveMode.MANUAL) {
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            double drive = -myOpMode.gamepad1.left_stick_y;
            double turn = myOpMode.gamepad1.right_stick_x;
            double strafe = -myOpMode.gamepad1.left_stick_x;


            double x_rotated = drive * Math.cos(localizer.heading) - strafe * Math.sin(localizer.heading);
            double y_rotated = drive * Math.sin(localizer.heading) + strafe * Math.cos(localizer.heading);

            frontLeftPower = x_rotated + turn - y_rotated;
            frontRightPower = x_rotated - turn + y_rotated;
            backLeftPower = x_rotated + turn + y_rotated;
            backRightPower = x_rotated - turn - y_rotated;

            driveFrontLeft.setPower(frontLeftPower);
            driveFrontRight.setPower(frontRightPower);
            driveBackLeft.setPower(backLeftPower);
            driveBackRight.setPower(backRightPower);
        } else if(state == DriveMode.APRILTAGS){

        }
    }

    public void driveToPose(double xTarget, double yTarget, double degreeTarget, double timeOutSeconds){
        double thetaTarget = Math.toRadians(degreeTarget);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(myOpMode.opModeIsActive() && timer.seconds() < timeOutSeconds &&
                ((Math.abs(localizer.x - xTarget) > 1 || Math.abs(localizer.y - yTarget) > 1||
                Math.abs(angleWrap(localizer.heading - thetaTarget)) > Math.toRadians(2)))){
            //Use PIDs to calculate motor powers based on error to targets
            double xPower = xPID.calculate(xTarget, localizer.x);
            double yPower = yPID.calculate(yTarget, localizer.y);

            double wrappedAngle = angleWrap(thetaTarget - localizer.heading);
            double tPower = headingPID.calculate(wrappedAngle);

            //rotate the motor powers based on robot heading
            double xPower_rotated = xPower * Math.cos(-localizer.heading) - yPower * Math.sin(-localizer.heading);
            double yPower_rotated = xPower * Math.sin(-localizer.heading) + yPower * Math.cos(-localizer.heading);

            // x, y, theta input mixing
            driveFrontLeft.setPower(xPower_rotated - yPower_rotated - tPower);
            driveBackLeft.setPower(xPower_rotated + yPower_rotated - tPower);
            driveFrontRight.setPower(xPower_rotated + yPower_rotated + tPower);
            driveBackRight.setPower(xPower_rotated - yPower_rotated + tPower);

            localizer.update();
            localizer.updateDashboard();
            localizer.telemetry();
            myOpMode.telemetry.update();
        }
        stopMotors();
    }



    public void driveStraightProfiledPID(float distance) {
        double targetCounts = distance * localizer.COUNTS_PER_INCH;
        double leftInitial = localizer.leftEncoder.getCurrentPosition();
        double rightInitial = localizer.rightEncoder.getCurrentPosition();
        float direction = -1;
        if (distance < 0) {
            direction = 1;
        }

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (myOpMode.opModeIsActive() &&
                time.seconds() < 0.5 + MotionProfile.motionProfileTime(DRIVE_MAX_ACC, DRIVE_MAX_VEL, targetCounts, time.seconds())) {
            double flPower = xPID.calculate(MotionProfile.motionProfile(DRIVE_MAX_ACC, DRIVE_MAX_VEL, targetCounts, time.seconds()), direction * localizer.leftEncoder.getCurrentPosition()-leftInitial);
            double frPower = xPID.calculate(MotionProfile.motionProfile(DRIVE_MAX_ACC, DRIVE_MAX_VEL, targetCounts, time.seconds()), direction * localizer.rightEncoder.getCurrentPosition()-rightInitial);
            double blPower = xPID.calculate(MotionProfile.motionProfile(DRIVE_MAX_ACC, DRIVE_MAX_VEL, targetCounts, time.seconds()), direction * localizer.leftEncoder.getCurrentPosition()-leftInitial);
            double brPower = xPID.calculate(MotionProfile.motionProfile(DRIVE_MAX_ACC, DRIVE_MAX_VEL, targetCounts, time.seconds()), direction * localizer.rightEncoder.getCurrentPosition()-rightInitial);

            driveFrontLeft.setPower(flPower);
            driveFrontRight.setPower(frPower);
            driveBackLeft.setPower(blPower);
            driveBackRight.setPower(brPower);

            myOpMode.telemetry.addData("flPower", flPower);
            myOpMode.telemetry.addData("instantTarget", MotionProfile.motionProfile(DRIVE_MAX_ACC, DRIVE_MAX_VEL, targetCounts, time.seconds()));
            myOpMode.telemetry.addData("profileTime", MotionProfile.motionProfileTime(DRIVE_MAX_ACC, DRIVE_MAX_VEL, targetCounts, time.seconds()));
            myOpMode.telemetry.addData("left", localizer.leftEncoder.getCurrentPosition());
            myOpMode.telemetry.addData("right", localizer.rightEncoder.getCurrentPosition());
            myOpMode.telemetry.update();
            localizer.update();
            localizer.updateDashboard();

        }

        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);
    }

    public void driveStraightPID(float distance, float timeOutSeconds) {
        double targetCounts = distance * localizer.COUNTS_PER_INCH;
        double leftInitial = localizer.leftEncoder.getCurrentPosition();
        double rightInitial = localizer.rightEncoder.getCurrentPosition();
        double leftTrueTarget = leftInitial + targetCounts;
        double rightTrueTarget = rightInitial+targetCounts;
        double leftError = leftTrueTarget - leftInitial;
        double rightError = rightTrueTarget- rightInitial;
        float direction = -1;
        if (distance < 0) {
            //direction = 1;
        }

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (myOpMode.opModeIsActive() && (Math.abs(leftError) > 10 || Math.abs(rightError) > 10) && time.seconds() < timeOutSeconds) {
            //update error
            leftError = leftTrueTarget - localizer.leftEncoder.getCurrentPosition();
            rightError = rightTrueTarget - localizer.rightEncoder.getCurrentPosition();

            double flPower = xPID.calculate(leftTrueTarget, direction*localizer.leftEncoder.getCurrentPosition());
            double frPower = xPID.calculate(rightTrueTarget, direction*localizer.rightEncoder.getCurrentPosition());
            double blPower = xPID.calculate(leftTrueTarget, direction*localizer.leftEncoder.getCurrentPosition());
            double brPower = xPID.calculate(rightTrueTarget, direction*localizer.rightEncoder.getCurrentPosition());

            driveFrontLeft.setPower(flPower);
            driveFrontRight.setPower(frPower);
            driveBackLeft.setPower(blPower);
            driveBackRight.setPower(brPower);

            myOpMode.telemetry.addData("flPower", flPower);
            myOpMode.telemetry.addData("targetCounts", targetCounts);
            myOpMode.telemetry.addData("left", localizer.leftEncoder.getCurrentPosition());
            myOpMode.telemetry.addData("right", localizer.rightEncoder.getCurrentPosition());
            myOpMode.telemetry.addData("leftError", leftError);
            myOpMode.telemetry.addData("rightError", leftError);
            myOpMode.telemetry.update();
            localizer.update();
            localizer.updateDashboard();
        }

        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);
    }

    public void driveSidePID(float distance, float timeOutSeconds) {
        double targetCounts = distance * localizer.COUNTS_PER_INCH;
        //double leftInitial = localizer.leftEncoder.getCurrentPosition();
        //double rightInitial = localizer.rightEncoder.getCurrentPosition();
        double centerInitial = localizer.centerEncoder.getCurrentPosition();
        //double leftError = targetCounts - leftInitial;
        //double rightError = targetCounts - rightInitial;
        double trueTarget = centerInitial + targetCounts;
        double centerError = trueTarget - centerInitial;

        double currentHeading = localizer.heading;

        float direction = -1;
        if (distance < 0) {
            //direction = 1;
        }

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (myOpMode.opModeIsActive() && (Math.abs(centerError) > 10) && time.seconds() < timeOutSeconds) {
            //update error
            centerError = trueTarget - localizer.centerEncoder.getCurrentPosition();
            double strafePower = xPID.calculate(trueTarget, direction*localizer.centerEncoder.getCurrentPosition());

            double turnPower = headingPID.calculate(currentHeading, localizer.heading)*-1;
            driveFrontLeft.setPower(-strafePower + turnPower);
            driveFrontRight.setPower(strafePower - turnPower);
            driveBackLeft.setPower(strafePower + turnPower);
            driveBackRight.setPower(-strafePower - turnPower);

            myOpMode.telemetry.addData("strafePower", strafePower);
            myOpMode.telemetry.addData("turnPower", turnPower);
            myOpMode.telemetry.addData("CurrentHeading", currentHeading);
            myOpMode.telemetry.addData("Heading",localizer.heading);
            myOpMode.telemetry.addData("targetCounts", targetCounts);
            myOpMode.telemetry.addData("center", localizer.centerEncoder.getCurrentPosition());
            myOpMode.telemetry.addData("centerError", centerError);
            myOpMode.telemetry.update();
            localizer.update();
            localizer.updateDashboard();
        }

        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);
    }

    public void encoderTurn(float counts, float timeOutSeconds) {
        double ORIGINAL_LEFT = localizer.leftEncoder.getCurrentPosition();
        double TARGET_COUNTS = ORIGINAL_LEFT + counts;
        double turnError = TARGET_COUNTS - ORIGINAL_LEFT;

        ElapsedTime time = new ElapsedTime();
        time.reset();
        //final double WHEEL_DIAMETER = 4;
        //final double COUNTS_PER_INCH = 537.6 / (Math.PI * WHEEL_DIAMETER);
        //final int STRAIGHT_COUNTS = (int) (COUNTS_PER_INCH * inches * -1);
        while (Math.abs(turnError) > 10 && myOpMode.opModeIsActive() && time.seconds() < timeOutSeconds) {
            turnError = TARGET_COUNTS - localizer.leftEncoder.getCurrentPosition();
            double turnPower = xPID.calculate(TARGET_COUNTS, localizer.leftEncoder.getCurrentPosition()) *-1 ;
            driveFrontLeft.setPower(turnPower);
            driveFrontRight.setPower(- turnPower);
            driveBackLeft.setPower(turnPower);
            driveBackRight.setPower(- turnPower);

            myOpMode.telemetry.addData("turnPower", turnPower);
            myOpMode.telemetry.addData("left", localizer.leftEncoder.getCurrentPosition());
            myOpMode.telemetry.addData("TARGET Counts", TARGET_COUNTS);

            myOpMode.telemetry.update();
            localizer.update();
            localizer.updateDashboard();
        }
        stopMotors();

        return;

    }

    /*public void driveToPose(double xTarget, double yTarget, double thetaTarget){

        //localizer.update();
        while(myOpMode.opModeIsActive() && ((Math.abs(localizer.x - xTarget) > 1 || Math.abs(localizer.y - yTarget) > 1
        Math.abs(localizer.heading - thetaTarget) < Math.PI/10))) {
            //Use PIDs to calculate motor powers based on error to targets
            double xPower = xPID.calculate(xTarget, localizer.x);
            double yPower = yPID.calculate(yTarget, localizer.y);

            double wrappedAngle = angleWrap(thetaTarget - localizer.heading);
            double tPower = headingPID.calculate(wrappedAngle);

            //rotate the motor powers based on robot heading
            double xPower_rotated = xPower * Math.cos(-localizer.heading) - yPower * Math.sin(-localizer.heading);
            double yPower_rotated = xPower * Math.sin(-localizer.heading) + yPower * Math.cos(-localizer.heading);

            // x, y, theta input mixing
            driveFrontLeft.setPower((-xPower_rotated + yPower_rotated + tPower) * -1);
            driveBackLeft.setPower((-xPower_rotated - yPower_rotated + tPower) * -1);
            driveFrontRight.setPower((-xPower_rotated - yPower_rotated - tPower) * -1);
            driveBackRight.setPower((-xPower_rotated + yPower_rotated - tPower) * -1);

            localizer.update();
            localizer.telemetry();
            myOpMode.telemetry.update();
        }
        stopMotors();
    }*/

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }




    public void stopMotors(){
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

        }



        public void turn(double power){
            driveFrontLeft.setPower(power);
            driveFrontRight.setPower(-power);
            driveBackLeft.setPower(power);
            driveBackRight.setPower(-power);
        }

        public void driveStraightTime(double power, double time){
            ElapsedTime t = new ElapsedTime();
            driveFrontLeft.setPower(power);
            driveFrontRight.setPower(power);
            driveBackLeft.setPower(power);
            driveBackRight.setPower(power);
            t.reset();
            while(myOpMode.opModeIsActive() && t.seconds() < time){
                localizer.update();
                localizer.updateDashboard();
                myOpMode.telemetry.addData("Motor Power", power);
                myOpMode.telemetry.addData("Time Target", time);
                myOpMode.telemetry.addData("Time Elapsed", t.seconds());
            }
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);
        }
    }
