package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    private LinearOpMode myOpMode = null;
    public Servo droneServo = null;

    public Drone(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init(){
        droneServo = myOpMode.hardwareMap.get(Servo.class, "droneServo");
    }

    public void teleOp(){
        if(myOpMode.gamepad1.x) {
            droneServo.setPosition(1.5);
        }
    }
}
