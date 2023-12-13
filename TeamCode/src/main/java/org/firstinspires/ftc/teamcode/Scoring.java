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

    public Scoring(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        leftArmServo = myOpMode.hardwareMap.get(Servo.class, "armServoLeft");
        rightArmServo = myOpMode.hardwareMap.get(Servo.class, "armServoRight");
        boxServo = myOpMode.hardwareMap.get(Servo.class, "boxServo");
        leftGateServo = myOpMode.hardwareMap.get(Servo.class, "gateServoLeft");
        rightGateServo = myOpMode.hardwareMap.get(Servo.class, "gateServoRight");

        leftArmServo.setPosition(0.02);
        rightArmServo.setPosition(0.98);
        boxServo.setPosition(0.93);
        leftGateServo.setPosition(0.5);
        rightGateServo.setPosition(0);

        is_open_left = false;
        is_open_right = false;


    }

    public void teleOp() {

        if (myOpMode.gamepad2.x) {
            leftGateServo.setPosition(0);
            is_open_left = true;
        } else {
            leftGateServo.setPosition(0.5);
            is_open_left = false;
        }
        //myOpMode.telemetry.addData("isWorking");
        if (myOpMode.gamepad2.y) {
            rightGateServo.setPosition(0.7);
            is_open_right = true;
        } else {
            rightGateServo.setPosition(0);
            is_open_right = false;
        }
        if(myOpMode.gamepad2.dpad_up) {
            leftArmServo.setPosition(0.3);
            rightArmServo.setPosition(0.7);
        }
        if(myOpMode.gamepad2.dpad_down) {
            leftArmServo.setPosition(0.02);
            rightArmServo.setPosition(0.98);
        }

        if(myOpMode.gamepad2.right_bumper) {
            boxServo.setPosition(0.93);

        } else if(myOpMode.gamepad2.left_bumper) {
            boxServo.setPosition(0.58);
        }
    }


    public void deliver(double power, double time) {
        
    }

}
