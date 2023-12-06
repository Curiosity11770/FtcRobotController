package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    private LinearOpMode myOpMode = null;

    public DcMotor liftLeft = null;
    public DcMotor liftRight = null;

    //public TouchSensor touch = null;

    public enum LiftMode {
        MANUAL
    }
    public LiftMode liftMode = LiftMode.MANUAL;
    public Lift(LinearOpMode opMode){
        myOpMode = opMode;
    }
    public void init(){
        liftLeft = myOpMode.hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = myOpMode.hardwareMap.get(DcMotor.class, "liftRight");

        //touch = myOpMode.hardwareMap.get(TouchSensor.class, "touch");

        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void teleOp(){
        if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.3){
            liftMode = LiftMode.MANUAL;
        }
        //myOpMode.telemetry.addData("Touch", "Pressed:" + touch.isPressed());

        /*if(touch.isPressed()){
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/

            myOpMode.telemetry.addData("working", liftLeft.getCurrentPosition());
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.3) {
                liftLeft.setPower(-myOpMode.gamepad2.left_stick_y);
                liftRight.setPower(-myOpMode.gamepad2.left_stick_y);
            } else {
                liftLeft.setPower(0);
                liftRight.setPower(0);
            }
    }
}
