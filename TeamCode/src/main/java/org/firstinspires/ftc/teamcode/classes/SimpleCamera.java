package org.firstinspires.ftc.teamcode.classes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class SimpleCamera {
    private LinearOpMode myOpMode = null;

    public WebcamName webcam1, webcam2;
    public VisionPortal visionPortal;
    private SimpleVisionProcessor colorVisionProcessor;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public AprilTagProcessor aprilTagProcessor;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound;

    public SimpleCamera(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init(){
        webcam1 = myOpMode.hardwareMap.get(WebcamName.class, "frontCamera");
        webcam2 = myOpMode.hardwareMap.get(WebcamName.class, "backCamera");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        colorVisionProcessor = new SimpleVisionProcessor();
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        //set decimation of AprilTag processor?

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(1280, 720))
                //.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(colorVisionProcessor)
                .build();

        targetFound     = false;    // Set to true when an AprilTag target is detected
        //setActiveCamera(webcam1);
    }


    public SimpleVisionProcessor.Selected returnSelection(){
        return colorVisionProcessor.getSelection();
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

    public void stopColorProcessor(){visionPortal.setProcessorEnabled(colorVisionProcessor, false);}

    public void setActiveCamera(WebcamName webCam){
        visionPortal.setActiveCamera(webCam);
    }

    void scanAprilTag() {
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags
        // and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            myOpMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            myOpMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            myOpMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            myOpMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        }
    }
}