package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class Drone {
    private LinearOpMode myOpMode = null;
    public CRServo droneServo = null;

    public Drone(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init(){
        droneServo = myOpMode.hardwareMap.get(CRServo.class, "droneServo");
    }

    public void teleOp(){
        if(myOpMode.gamepad1.x){
            droneServo.setPower(0.7);
        } else {
            droneServo.setPower(0);
        }
    }
}
