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

    public Scoring(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void init(){
        leftArmServo = myOpMode.hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = myOpMode.hardwareMap.get(Servo.class, "rightArmServo");
        boxServo = myOpMode.hardwareMap.get(Servo.class, "boxServo");
        leftGateServo = myOpMode.hardwareMap.get(Servo.class, "leftGateServo");
        rightGateServo = myOpMode.hardwareMap.get(Servo.class, "rightGateServo");
    }

    public void teleOp() {

    }

}
