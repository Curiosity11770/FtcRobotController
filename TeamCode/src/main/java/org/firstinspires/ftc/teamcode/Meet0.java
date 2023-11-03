package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Meet0 extends LinearOpMode {
    Robot robot = new Robot(this);

    //private ElapsedTime = new ElapsedTime();

    public void runOpMode(){
        robot.init();
        telemetry.addData("Status", "Intialized");
        waitForStart();
        //runtime.reset();

        while(opModeIsActive()){
            robot.teleOp();
        }
    }


}
