package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class SimpleCamera {
    private LinearOpMode myOpMode = null;

    private SimpleVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    public SimpleCamera(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init(){
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
}