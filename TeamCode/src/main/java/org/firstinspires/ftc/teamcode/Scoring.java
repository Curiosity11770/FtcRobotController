package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Scoring {
    private LinearOpMode myOpMode = null;

    public Servo leftArmServo = null;

    public Servo rightArmServo = null;

    public Servo boxServo = null;

    public Servo leftGateServo;
    public Servo rightGateServo;

    public boolean is_open_left;

    public boolean is_open_right;

    public boolean isUp;

    public final double GATE_UP_LEFT = 0.5;
    public final double GATE_DOWN_LEFT = 0;
    public final double GATE_UP_RIGHT = 0;
    public final double GATE_DOWN_RIGHT = 0.5;

    public final double ARM_UP_LEFT = 0.39;
    public final double ARM_UP_RIGHT = 0.61;
    public final double ARM_DOWN_LEFT = 0.1275;
    public final double ARM_DOWN_RIGHT = 0.8725;

    public final double BOX_OUT = 0.4;
    public final double BOX_IN = 0.74;

    public ElapsedTime timer = new ElapsedTime();

    public enum ScoringMode {
        SCORING,
        INTAKE
    }

    public ScoringMode state = ScoringMode.INTAKE;

    public Scoring(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        leftArmServo = myOpMode.hardwareMap.get(Servo.class, "armServoLeft");
        rightArmServo = myOpMode.hardwareMap.get(Servo.class, "armServoRight");
        boxServo = myOpMode.hardwareMap.get(Servo.class, "boxServo");
        leftGateServo = myOpMode.hardwareMap.get(Servo.class, "gateServoLeft");
        rightGateServo = myOpMode.hardwareMap.get(Servo.class, "gateServoRight");

        leftArmServo.setPosition(ARM_DOWN_LEFT);
        rightArmServo.setPosition(ARM_DOWN_RIGHT);
        boxServo.setPosition(BOX_IN);
        leftGateServo.setPosition(GATE_DOWN_LEFT);
        rightGateServo.setPosition(GATE_DOWN_RIGHT);

        is_open_left = false;
        is_open_right = false;
        isUp = false;
    }

    public void teleOp(boolean firstBreak, boolean secondBreak) {
        if(state == ScoringMode.INTAKE) {

            boxServo.setPosition(BOX_IN);
            timer.reset();
            if(timer.seconds() > 0.8) {
                leftArmServo.setPosition(ARM_DOWN_LEFT);
                rightArmServo.setPosition(ARM_DOWN_RIGHT);
            }

        } else if (state == ScoringMode.SCORING){

            leftArmServo.setPosition(ARM_UP_LEFT);
            rightArmServo.setPosition(ARM_UP_RIGHT);

            if(myOpMode.gamepad2.right_bumper) {
                boxServo.setPosition(BOX_OUT);
            } else if (myOpMode.gamepad2.left_bumper){
                boxServo.setPosition(BOX_IN);
            }

        }

        if(myOpMode.gamepad2.dpad_up){
            state = ScoringMode.SCORING;
        } else if(myOpMode.gamepad2.dpad_down){
            state = ScoringMode.INTAKE;
        }
        if (myOpMode.gamepad2.y) {
            rightGateServo.setPosition(GATE_DOWN_RIGHT);
            is_open_left = true;
        } else if (myOpMode.gamepad2.x){
            rightGateServo.setPosition(GATE_UP_RIGHT);
        }
        //myOpMode.telemetry.addData("isWorking");
        if (myOpMode.gamepad2.b) {
            leftGateServo.setPosition(GATE_DOWN_LEFT);
            is_open_right = true;
        }
            else if(myOpMode.gamepad2.a){
                leftGateServo.setPosition(GATE_UP_LEFT);
            }
        /*} else if (secondBreak){
            rightGateServo.setPosition(0);
            is_open_right = true;*/
    }


    public void deliver(double power, double time) {
        
    }

}
