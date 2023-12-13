package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    private LinearOpMode myOpMode = null;
    public Servo droneServo = null;
    public Servo droneServo2 = null;

    public Drone(LinearOpMode opmode){
        myOpMode = opmode;
    }


    public void init(){
        droneServo = myOpMode.hardwareMap.get(Servo.class, "droneServo");
        droneServo2 = myOpMode.hardwareMap.get(Servo.class, "droneServo2");

        droneServo.setPosition(0);
        droneServo2.setPosition(0.5);
    }

    public void teleOp(){
        if(myOpMode.gamepad1.x) {
            droneServo.setPosition(0.5);
        } else {
            droneServo.setPosition(0);
        }
        if(myOpMode.gamepad1.y) {
            droneServo2.setPosition(0.4);
        }
    }
}
