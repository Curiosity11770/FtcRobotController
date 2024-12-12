package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {

    private LinearOpMode myOpMode = null;
    public Servo clawWrist = null;
    public Servo clawServo = null;
    public Servo scoringPivot = null;

    public static final double CLAW_UP = 0.88;
    public static final double CLAW_DOWN = 0.2;
    public static final double CLAW_OPEN = 0.8;
    public static final double CLAW_CLOSED = 0.32;
    public static final double SCORING_UP = 0;
    public static final double SCORING_DOWN = 0.45;

    public double clawPosition;
    public double clawRotation;
    public double scoringPosition;

    public Scoring(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        scoringPivot = myOpMode.hardwareMap.get(Servo.class, "scoringPivot");
        clawWrist = myOpMode.hardwareMap.get(Servo.class, "clawWrist");
        clawServo = myOpMode.hardwareMap.get(Servo.class, "clawServo");
        scoringPivot.setPosition(SCORING_DOWN);
                //analogInput.getVoltage() / 3.3 * 360;
        clawWrist.setPosition(CLAW_UP);
        clawServo.setPosition(CLAW_CLOSED);
        clawPosition = CLAW_CLOSED;
        clawRotation = CLAW_UP;
        scoringPosition = SCORING_UP;
    }

    public void teleOp() {
        clawWrist.setPosition(clawRotation);
        clawServo.setPosition(clawPosition);
        scoringPivot.setPosition(scoringPosition);
        //position = scoringPivot.getVoltage();
        if (myOpMode.gamepad2.left_trigger > 0.7) {
            clawPosition = CLAW_OPEN;
        } else if (myOpMode.gamepad2.right_trigger > 0.7) {
            clawPosition = CLAW_CLOSED;
        }
        if(myOpMode.gamepad2.x){
            clawRotation = CLAW_UP;
        } else if (myOpMode.gamepad2.y){
            clawRotation = CLAW_DOWN;
        }
        if (myOpMode.gamepad2.dpad_up){
            scoringPosition = SCORING_UP;
        }else if(myOpMode.gamepad2.dpad_down){
            scoringPosition = SCORING_DOWN;
        } else {
        }

    }
}
