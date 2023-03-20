package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.code2023.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.code2023.DriveConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.code2023.SampleTankDrive;

import java.util.ArrayList;

//@Autonomous
public class TrajectoryAutoRight extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    ElapsedTime timer;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    //havent sensed any tags yet
    AprilTagDetection tagOfInterest = null;

    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;

    private Servo pivot = null;

    private CRServo intake = null;

    private int liftTarget;

    private double pPos = 0;
    private double pivotTarget = 0.54;

    private double liftPower = 0.5;
    private double intakePower = 0.7;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        liftLeft = hardwareMap.get(DcMotor.class, "leftSlides");
        liftRight = hardwareMap.get(DcMotor.class, "rightSlides");

        //pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot = hardwareMap.get(Servo.class, "pivot");

        intake = hardwareMap.get(CRServo.class, "intakeServo");

        //liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pivot.setDirection(DcMotor.Direction.REVERSE);
        //pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //resetLift();
        resetLiftEncoders();
        //resetPivotEncoders();

        timer = new ElapsedTime();


        pivot.setPosition(0.54);
        //manual = true;


        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                //38
                .splineTo(new Vector2d(37, 0), Math.toRadians(0))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(53, 0), Math.toRadians(0))
                .build();
        Trajectory traj3A = drive.trajectoryBuilder(traj2.end())
                //84
                .splineTo(new Vector2d(79, 0), Math.toRadians(0))
                .build();
        Trajectory traj3B = drive.trajectoryBuilder(traj2.end())
                //84
                .splineTo(new Vector2d(80, 0), Math.toRadians(0),
                        SampleTankDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleTankDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3B.end(), true)
                .splineTo(new Vector2d(47, 0), Math.toRadians(0))
                .build();
        Trajectory left = drive.trajectoryBuilder(traj4.end(), true)   //pos 1
                .splineTo(new Vector2d(40, 0), Math.toRadians(0))
                .build();
        Trajectory middle = drive.trajectoryBuilder(traj4.end())                 //pos 2
                .splineTo(new Vector2d(60, 0), Math.toRadians(0))
                .build();
        Trajectory right = drive.trajectoryBuilder(traj4.end())                 //pos 3
                .splineTo(new Vector2d(77, 0), Math.toRadians(0))
                .build();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE  || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }


        if(isStopRequested()) return;
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        intakeIn();
        liftPosition(0.65, "MIDDLE", 1800);
        drive.followTrajectory(traj1); //this drives in front of the middle junction

        //pivotPosition(0.8);
        pivot.setPosition(0.85);
        wait(1.25);

        //intake out for 1 second, then stop
        intakeOut();
        wait(1.0);
        intakeStop();
        //pivot center
        pivot.setPosition(0.54);
        wait(0.5);
        timer.reset();

        while(timer.seconds() < 1.5 && !isStopRequested()){
            liftPosition(0.65, "CONE1", 500);
            telemetry.update();
        }
        //wait(2.0);
        drive.followTrajectory(traj2); //drives to align with cone stack
        //wait(0.5);
        drive.turn(Math.toRadians(-97)); //turns
        //lift to cone height

        //liftPosition(0.65, "CONE1");

        intakeIn();
        wait(0.2);
        drive.followTrajectory(traj3A); //drives into cone stack
        wait(0.1);
        //drive.followTrajectory(traj3B);
        //cone should be in robot
        //lift to above the cone stack
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            liftPosition(0.65, "ABOVESTACK", 1000);
        }
        drive.followTrajectory(traj4); //backs up to high junction
        //lift to 3
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
            liftPosition(0.65, "HIGH", 2900);
        }
        //pivot left
        pivot.setPosition(0.8);
        wait(1.25);
        intakeOut();
        wait(1.0);
        pivot.setPosition(0.54);
        wait(0.2);
        //intake out for 1 second then stop
        //pivot center
        //lift to ground
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
            liftPosition(0.65, "GROUND", 0);
        }

        //need a last trajectory to park

        if(tagOfInterest.id == LEFT){
            drive.followTrajectory(left);
            //this trajectory would go forwards/backwards
        }
        else if(tagOfInterest.id == RIGHT){
            drive.followTrajectory(right);
        }
        else{
            drive.followTrajectory(middle);
        }


    }

    private void wait(double seconds){
        resetRuntime();
        while(getRuntime() < seconds){}
    }

    private void pivotPosition(double pivotTarget){

            if(Math.abs(pPos-pivotTarget) >= 0.01) {
                if (pPos > pivotTarget) {
                    pivot.setPosition(pPos - 0.005);
                } else if (pPos < pivotTarget) {
                    pivot.setPosition(pPos + 0.005);
                }
            }
    }
    //positive encoder values and motor powers are UP
    private void liftPosition(double motorPower, String height, int counts) {
        if (height.equals("GROUND")) {     //GROUND
            //resetLift();
            liftTarget = 0;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //pivotPosition(0.54);

            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if (opModeIsActive() && liftLeft.isBusy()) {
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("left", liftLeft.getTargetPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if (height.equals("CONE1")) {    //LOW
            liftTarget = 450;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if (opModeIsActive() && liftLeft.isBusy()) {
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("left", liftLeft.getTargetPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if (height.equals("MIDDLE")) {    //MIDDLE
            liftTarget = 1750;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if (opModeIsActive() && liftLeft.isBusy()) {
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if (height.equals("HIGH")) {                    //HIGH
            liftTarget = 2800;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if (opModeIsActive() && liftLeft.isBusy()) {
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else {
            liftTarget = counts;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if (opModeIsActive() && liftLeft.isBusy()) {
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        }
    }

    private void intakeIn(){
        intake.setPower(-intakePower);
    }
    private void intakeOut(){
        intake.setPower(intakePower);
    }
    private void intakeStop(){
        intake.setPower(0);
    }


    private void resetLiftEncoders(){
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

}
