package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    private LinearOpMode myOpMode = null;
    public CRServo droneServo = null;
    public Servo droneServo2 = null;

    public final double DRONE_ANGLE = 0.6;
    public final double START_DRONE = 1;

    public Drone(LinearOpMode opmode){
        myOpMode = opmode;
    }


    public void init(){
        droneServo = myOpMode.hardwareMap.get(CRServo.class, "droneServo");
        droneServo2 = myOpMode.hardwareMap.get(Servo.class, "droneServo2");

        droneServo.setPower(0);
        droneServo2.setPosition(START_DRONE);
    }

    public void teleOp(){
        if(myOpMode.gamepad1.b) {
            droneServo.setPower(1);
        }

        if(myOpMode.gamepad1.y) {
            droneServo2.setPosition(DRONE_ANGLE);
        }
    }
}
