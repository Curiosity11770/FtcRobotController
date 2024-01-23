package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.SimpleVisionProcessor;

@Autonomous public class SimpleVisionTest extends LinearOpMode {

    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.init();
        while (!isStarted()) {
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }
        waitForStart();

        robot.camera.stopColorStreaming();

        if (opModeIsActive()) {

            //now you can move your robot based on the value of the 'selection' stored in the vision processor
            if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT){
                //strafe left
                //drive straight
            }else if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.RIGHT){
                //strafe right
                //drive straight
            }else{
                //drive straight ex;
                //robot.encoderDrive(0.5,24,5);
            }

            //You could also do other stuff afterwards...
            //turn right 90
            //robot.turnCW(0.5,90);

        }
    }
}
