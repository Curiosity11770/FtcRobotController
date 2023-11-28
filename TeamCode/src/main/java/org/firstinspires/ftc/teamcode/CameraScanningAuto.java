package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CameraScanningTestAuto", group = "Linear Opmode")
public class CameraScanningAuto extends LinearOpMode {
    Robot robot = new Robot(this);
    Camera camera = new Camera(this);

   OpenCv.OpenCvPosition position = OpenCv.OpenCvPosition.LEFT;

    public void runOpMode(){
        robot.init();
        camera.init();
        position = camera.pipeline.getAnalysis();

        if(position == OpenCv.OpenCvPosition.RIGHT){

        } else if (position == OpenCv.OpenCvPosition.LEFT) {

        } else {

        }

    }

}