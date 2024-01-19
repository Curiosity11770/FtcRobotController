package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.classes.SimpleVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Camera {

    private LinearOpMode myOpMode = null;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private SimpleVisionProcessor visionProcessor;

    public static final boolean USE_WEBCAM = true;


    public Camera(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void init() {
        visionProcessor = new SimpleVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"),
                visionProcessor);
    }

    public SimpleVisionProcessor.Selected returnSelection(){
        return visionProcessor.getSelection();
    }

    public void stopColorStreaming(){
        visionPortal.stopStreaming();
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .addProcessor(aprilTag)
                    .setCameraResolution(new Size(640,480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
}