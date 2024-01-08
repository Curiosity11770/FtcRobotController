package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {

    private LinearOpMode myOpMode = null;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    OpenCv pipeline;

    OpenCvBlue pipeline2;

    OpenCvWebcam webcam1;

    //OpenCvWebcam webcam2;



    public Camera(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void init(){

    }

    public void initAutoBlue() {
        pipeline2 = new OpenCvBlue();

        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId2", "id", myOpMode.hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam1.setPipeline(pipeline2);

        webcam1.setMillisecondsPermissionTimeout(5000);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public void initAutoRed(){
        /*aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
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
        }*/

        pipeline = new OpenCv();

        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId2", "id", myOpMode.hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam1.setPipeline(pipeline);

        webcam1.setMillisecondsPermissionTimeout(5000);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
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