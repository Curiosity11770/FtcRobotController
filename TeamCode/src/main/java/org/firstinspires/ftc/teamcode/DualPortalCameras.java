package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class DualPortalCameras {
    private LinearOpMode myOpMode = null;

    public WebcamName frontCamera, backCamera;
    public VisionPortal visionPortal;

    public VisionPortal visionPortalFront;
    private SimpleVisionProcessor colorVisionProcessor;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public AprilTagProcessor aprilTagProcessor;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound;

    public DualPortalCameras(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        frontCamera = myOpMode.hardwareMap.get(WebcamName.class, "frontCamera");
        backCamera = myOpMode.hardwareMap.get(WebcamName.class, "backCamera");

        colorVisionProcessor = new SimpleVisionProcessor();
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        //set decimation of AprilTag processor?
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        int[] portalList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        int portalOneID = portalList[0];
        int portalTwoID = portalList[1];

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(backCamera)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .setLiveViewContainerId(portalOneID)
                .build();

        // Create the vision portal by using a builder.
        visionPortalFront = new VisionPortal.Builder()
                .setCamera(frontCamera)
                //.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(colorVisionProcessor)
                .setLiveViewContainerId(portalTwoID)
                .build();


        setManualExposure(4, 255);
        targetFound = false;    // Set to true when an AprilTag target is detected
        //setActiveCamera(webcam1);
    }


    public SimpleVisionProcessor.Selected returnSelection() {
        return colorVisionProcessor.getSelection();
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    //public void stopColorProcessor(){visionPortal.setProcessorEnabled(colorVisionProcessor, false);}

    void scanAprilTag(int targetTag) {
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags
        // and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == targetTag || detection.id == targetTag+3) {
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

    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                myOpMode.sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }
}