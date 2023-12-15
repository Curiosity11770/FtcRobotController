package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Meet2TeleOp", group = "TeleOp")
public class Meet2TeleOp extends LinearOpMode {
    Robot robot = new Robot(this);

    //private ElapsedTime = new ElapsedTime();

    public void runOpMode(){
        robot.init();
        telemetry.addData("Status", "Intialized");
        waitForStart();
        //runtime.reset();

        while(opModeIsActive()){
            robot.teleOp();
            //robot.touchSense();
            telemetry.update();
        }
    }


}
