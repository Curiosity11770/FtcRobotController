package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    private LinearOpMode myOpMode = null;

    public Servo leftArmServo = null;

    public Servo rightArmServo = null;

    public Servo boxServo = null;

    public Servo leftGateServo;
    public Servo rightGateServo;

    public boolean is_open_left;

    public boolean is_open_right;

    public Scoring(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void init(){
        leftArmServo = myOpMode.hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = myOpMode.hardwareMap.get(Servo.class, "rightArmServo");
        boxServo = myOpMode.hardwareMap.get(Servo.class, "boxServo");
        leftGateServo = myOpMode.hardwareMap.get(Servo.class, "leftGateServo");
        rightGateServo = myOpMode.hardwareMap.get(Servo.class, "rightGateServo");

        leftArmServo.setPosition(0);
        rightArmServo.setPosition(0);
        boxServo.setPosition(0);
        leftGateServo.setPosition(0);
        rightGateServo.setPosition(0);

        is_open_left = false;
        is_open_right = false;


    }

    public void teleOp() {
        if (!is_open_left) {
            if (myOpMode.gamepad2.x) {
                leftGateServo.setPosition(-0.7);
                is_open_left = true;
            }
        } else {
            if (myOpMode.gamepad2.x) {
                leftGateServo.setPosition(0);
                is_open_left = false;
            }
        }

        if (!is_open_right) {
            if (myOpMode.gamepad2.y) {
                rightGateServo.setPosition(-0.7);
                is_open_right = true;
            }
        } else {
            if (myOpMode.gamepad2.y) {
                rightGateServo.setPosition(0);
                is_open_right = false;
            }
        }

    }

    public void deliver(double power, double time) {
        
    }

}
