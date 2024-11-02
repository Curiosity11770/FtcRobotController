package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift {
    private LinearOpMode myOpMode = null;

    //lift motors
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;

    //touch sensor
    //public TouchSensor touch = null;

    public enum LiftMode {
        MANUAL,
        HIGH,
        MEDIUM,
        LOW,
        GROUND
    }

    public LiftMode liftMode = LiftMode.MANUAL;

    public Lift(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "leftLift");

        //touch = myOpMode.hardwareMap.get(TouchSensor.class, "touch");

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        // brake and encoders
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Lift Initialized");
    }

    public void teleOp() {
        //gamepad control specific to lift
        if (Math.abs(myOpMode.gamepad2.right_stick_y) > 0.8) {
            liftMode = LiftMode.MANUAL;
        }

       // myOpMode.telemetry.addData("touch", "Pressed: " + touch.isPressed());

        //code defining behavior of lift in each state
        if (liftMode == LiftMode.MANUAL) {
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (Math.abs(myOpMode.gamepad2.right_stick_y) > 0.1) {
                //robot.liftLeft.setPower(-0.8);
                //robot.liftRight.setPower(-0.8);
                leftLift.setPower(-myOpMode.gamepad2.right_stick_y);
                rightLift.setPower(-myOpMode.gamepad2.right_stick_y);
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }
        }
    }

}