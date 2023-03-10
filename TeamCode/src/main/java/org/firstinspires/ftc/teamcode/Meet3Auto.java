/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Meet3Auto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final double countspercm = 19.3566666;

    private int liftTarget;

    Orientation angles;
    Acceleration gravity;

    BNO055IMU imu;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    //havent sensed any tags yet
    AprilTagDetection tagOfInterest = null;

    //motor stuff
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //private DcMotor verticalLeft = null;
    //private DcMotor horizontal = null;

    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;

    private double pPos;

    private Servo pivot = null;

    private CRServo intake = null;

    private String height = "LOW";

    //state/control stuff
    private ElapsedTime runtime = new ElapsedTime();
    public State stateRunning;

    private enum State {
        SCAN,
        FORWARDONE,
        FORWARDTWO,
        FORWARDTHREE,
        STOP;
    }

    @Override
    public void runOpMode()
    {

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //motor stuff
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //verticalLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        //horizontal = hardwareMap.get(DcMotor.class, "backRight");

        liftLeft = hardwareMap.get(DcMotor.class, "leftSlides");
        liftRight = hardwareMap.get(DcMotor.class, "rightSlides");

        pivot = hardwareMap.get(Servo.class, "pivot");

        intake = hardwareMap.get(CRServo.class, "intakeServo");

        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pivot.setDirection(DcMotor.Direction.REVERSE);
        //pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetLiftEncoders();
        //resetPivotEncoders();
        pivot.setPosition(0.5);

        //reset encoders

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        setStateRunning(State.SCAN);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE  || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */

        pPos = pivot.getPosition();

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
            driveForwards(0.7, 66);
        }

        if(tagOfInterest.id == LEFT){
            driveForwards(0.7, 10);
            turn90CCW(0.6);
            driveForwards(0.6, 74);
            //resetAngle();
            //turn90CW(0.6);
            //turnCW(0.6, 270);
            resetIMUCW();
            driveForwards(0.6, 55);
            telemetry.addData("In forward one", "right now");
            telemetry.update();
        }else if(tagOfInterest.id == MIDDLE){
            driveForwards(0.6, 66);
            telemetry.addData("In forward two", "right now");
            telemetry.update();
        }
        else{
            driveForwards(0.7, 6);
            turn90CW(0.5);
            driveForwards(0.6, 70);
            //resetAngle();
            //turn90CW(0.6);
            resetIMUCCW();
            driveForwards(0.6, 60);
            telemetry.addData("In forward three", "right now");
            telemetry.update();

        }

        //DELETE THIS LATER
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void driveForwards(double motorPower, double distance){
        resetDriveEncoders();
        double counts = distance*countspercm;

        while(opModeIsActive() && frontLeft.getCurrentPosition() < counts){
            backLeft.setPower(motorPower);
            backRight.setPower(motorPower);
            frontLeft.setPower(motorPower);
            frontRight.setPower(motorPower);

            telemetry.addData("Motor position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        driveStop();
    }

    private void driveBackwards(double motorPower, double distance){
        resetDriveEncoders();
        double counts = distance*countspercm;

        while(opModeIsActive() && frontLeft.getCurrentPosition() < counts){
            backLeft.setPower(-motorPower);
            backRight.setPower(-motorPower);
            frontLeft.setPower(-motorPower);
            frontRight.setPower(-motorPower);

            telemetry.addData("Motor position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        driveStop();
    }

    private void turnCW(double motorPower, double degrees){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while(degrees > Math.abs(angles.firstAngle)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            backLeft.setPower(motorPower);
            backRight.setPower(-motorPower);
            frontLeft.setPower(motorPower);
            frontRight.setPower(-motorPower);

            telemetry.addData("imu: ", angles.firstAngle);
            telemetry.addData("motor position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        driveStop();
    }

    private void turn90CW(double motorPower){
        turnCW(motorPower, 60);
    }

    private void turn90CCW(double motorPower){
        turnCCW(motorPower, 60);
    }

    private void turnCCW(double motorPower, double degrees){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while(degrees > Math.abs(angles.firstAngle)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            backLeft.setPower(-motorPower);
            backRight.setPower(motorPower);
            frontLeft.setPower(-motorPower);
            frontRight.setPower(motorPower);

            telemetry.addData("imu: ", angles.firstAngle);
            telemetry.addData("motor position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        driveStop();
    }

    private void resetIMUCW(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while(Math.abs(angles.firstAngle) > 1){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            backLeft.setPower(0.3);
            frontLeft.setPower(0.3);
            backRight.setPower(-0.3);
            frontRight.setPower(-0.3);
        }
        driveStop();
    }

    private void liftPosition(double motorPower, String height){
        if(height.equals("MANUAL")){
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //add touch sensor lower bound (if touch sensor not pressed)
            if(gamepad2.right_stick_y > 0.2 && Math.abs(liftLeft.getCurrentPosition()) < 3000 && Math.abs(liftRight.getCurrentPosition()) < 3000){
                liftLeft.setPower(-motorPower);
                liftRight.setPower(-motorPower);
                //make proportional to right_stick_y? (like drivetrain)
            } else if(gamepad2.right_stick_y < -0.2){
                liftLeft.setPower(motorPower);
                liftRight.setPower(motorPower);
            }
            else
            {
                liftLeft.setPower(0.1);
                liftRight.setPower(0.1);
            }
        }
        else if(height.equals("GROUND")){     //GROUND
            liftTarget = 0;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("left", liftLeft.getTargetPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }

        } else if (height.equals("LOW")){    //LOW
            liftTarget = 1400;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("left", liftLeft.getTargetPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if (height.equals("MIDDLE")){    //MIDDLE
            liftTarget = 2200;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if(height.equals("HIGH")){                    //HIGH
            liftTarget = 2900;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else {
            height = "MANUAL";
        }
    }

    private void resetIMUCCW(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while(Math.abs(angles.firstAngle) > 1){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            backLeft.setPower(-0.3);
            frontLeft.setPower(-0.3);
            backRight.setPower(0.3);
            frontRight.setPower(0.3);
        }
        driveStop();
    }

    private void wait(double time){
        resetRuntime();
        driveStop();
        while(opModeIsActive() && runtime.seconds() < time){

        }
    }

    private void pivotPosition(double pivotTarget){
        if (pPos > pivotTarget) {
            pivot.setPosition(pPos - 0.005);
        } else if (pPos < pivotTarget) {
            pivot.setPosition(pPos + 0.005);
        }
    }

    private void intakeIn(){
        intake.setPower(0.7);
    }
    private void intakeOut(){
        intake.setPower(-0.7);
    }
    private void intakeStop(){
        intake.setPower(0.0);
    }

    private void driveStop(){
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    private void resetDriveEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void resetLiftEncoders(){
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*private void resetPivotEncoders(){
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }*/

    private void setStateRunning(State state){
        //reset encoders
        stateRunning = state;
        runtime.reset();
    }
}