package org.firstinspires.ftc.teamcode.code2023;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.util.Path;

import java.util.ArrayList;

@Config
@Autonomous
public class RegAutoRight extends LinearOpMode {

    public static double tx = 36;
    public static double ty = 0;
    public static boolean tf = true;

    public static double tx2 = 53;
    public static double ty2 = 0;
    public static boolean tf2 = true;

    public static double tx3 = 53;
    public static double ty3 = -22.5;
    public static boolean tf3 = true;

    public static double tx4 = 53;
    public static double ty4 = 16;
    public static boolean tf4 = false;

    SampleTankDrive robot;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleTankDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        robot.aprilTagDetectionPipeline = new AprilTagDetectionPipeline(robot.tagsize, robot.fx, robot.fy, robot.cx, robot.cy);

        robot.camera.setPipeline(robot.aprilTagDetectionPipeline);
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robot.camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(50);
        timer = new ElapsedTime();

        //create the path object
        Path path1 = new Path(robot, tx, ty, tf);
        Path path2 = new Path(robot, tx2, ty2, tf2);
        Path path3 = new Path(robot, tx3, ty3, tf3);
        Path path4 = new Path(robot, tx4, ty4, tf4);

        Path path5 = new Path(robot, tx3, ty3+0.25, tf3);
        Path path6 = new Path(robot, tx4, ty4, tf4);

        Path right = new Path(robot, 52, -20, true);
        Path middle = new Path(robot, 52, 3, true);
        Path left = new Path(robot, 52, 26, false);

        robot.resetLiftEncoders();

        robot.pivot.setPosition(0.54);
        robot.alignBack();

        //waitForStart();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = robot.aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE  || tag.id == RIGHT) {
                        robot.tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(robot.tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(robot.tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(robot.tagOfInterest);
                    }
                }
            }
            else {
                telemetry.addLine("Don't see tag of interest :(");

                if(robot.tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(robot.tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }


        if(isStopRequested()) return;
        if(robot.tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(robot.tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        robot.intakeIn(robot.intakePower);
        robot.liftPosition(0.7, "MIDDLE");
        //while the path is incomplete
        path1.time.reset();
        while (!path1.targetReached) {
            //follow the path!
            path1.followPath();

            pathTele(path1);
        }


        robot.pivot.setPosition(0.85);
        wait(1.5);
        robot.intakeOut(robot.intakePower);
        wait(0.5);
        robot.intakeStop();
        robot.pivot.setPosition(0.54);

        wait(0.5);

        path2.time.reset();
        while (!path2.targetReached){
            path2.followPath();
            robot.liftPosition(0.8, "CONE1");

            pathTele(path2);

        }

        robot.intakeIn(robot.intakePower);
        path3.time.reset();
        while (!path3.targetReached){
            path3.followPath();
            pathTele(path3);

        }
        wait(1.5);      // waiting for intaking

        timer.reset();      // needs to be own loop or will knock over stack
        while(timer.seconds() < 1.0 && !isStopRequested()){
            robot.liftPosition(0.7, "ABOVESTACK");
        }

        path4.time.reset();
        while (!path4.targetReached){
            path4.followPath();
            pathTele(path4);
            robot.liftPosition(0.65, "HIGHAUTO");
        }

        robot.alignRight();
        robot.pivot.setPosition(0.83);
        wait(1.5);
        robot.intakeOut(robot.intakePower);     // make this slower? current = 0.7
        wait(0.25);
        robot.pivot.setPosition(0.54);
        wait(0.2);

        robot.intakeIn(robot.intakePower);

        path5.time.reset();
        while (!path5.targetReached){
            path5.followPath();
            pathTele(path5);
            robot.liftPosition(0.8, "CONE2");
        }
        wait(1.5);      // waiting for intaking

        timer.reset();
        while(timer.seconds() < 1.0 && !isStopRequested()){
            robot.liftPosition(0.7, "ABOVESTACK");
        }
        path6.time.reset();
        while (!path6.targetReached){
            path6.followPath();
            pathTele(path6);
            robot.liftPosition(0.65, "HIGHAUTO");

        }

        robot.pivot.setPosition(0.83);
        wait(1.5);
        robot.intakeOut(robot.intakePower);     // make this slower? current = 0.7
        wait(0.25);
        robot.pivot.setPosition(0.54);
        wait(0.2);

        robot.intakeStop();

        //intake out for 1 second then stop
        //pivot center
        //lift to ground
        if(robot.tagOfInterest.id == LEFT){
            left.time.reset();
            while (!isStopRequested() && (!left.targetReached || left.time.seconds() < 10.0)){
                left.followPath();
                pathTele(left);
                robot.liftPosition(0.8, "GROUND");
            }
        }
        else if(robot.tagOfInterest.id == RIGHT){
            right.time.reset();
            while (!isStopRequested() && (!right.targetReached  || right.time.seconds() < 10.0)){
                right.followPath();
                pathTele(right);
                robot.liftPosition(0.8, "GROUND");
            }
        }
        else{
            middle.time.reset();
            while (!isStopRequested() && (!middle.targetReached || middle.time.seconds() < 10.0)){

                middle.followPath();
                pathTele(middle);
                robot.liftPosition(0.8, "GROUND");

            }
        }


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x* robot.FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y* robot.FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*robot.FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void pathTele(Path path2){
        telemetry.addData("path state", path2.state);
        //telemetry to check agreement of angle formats
        telemetry.addData("robot Heading", path2.currentHeading);

        telemetry.addData("angle to target", path2.theta);
        //telemetry to check distance
        telemetry.addData("distance to target", path2.currentDistance);
        //telemetry to check outputs
        telemetry.addData("f", path2.f);
        telemetry.addData("t", path2.t);
        telemetry.update();
    }

    void wait(double seconds){
        resetRuntime();
        while(getRuntime() < seconds){}
    }
}
