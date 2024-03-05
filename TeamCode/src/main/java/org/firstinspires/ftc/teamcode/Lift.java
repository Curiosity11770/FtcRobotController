package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift {
    private LinearOpMode myOpMode = null;

    public DcMotor liftLeft = null;
    public DcMotor liftRight = null;

    public TouchSensor touch = null;

    public enum LiftMode {
        MANUAL,
        LOW,
        MIDDLE,
        HIGH,
        INTAKE,
        HANG
    }
    public LiftMode liftMode = LiftMode.MANUAL;

    PIDController liftLeftPID;
    PIDController liftRightPID;

    public static final double LIFT_KP = 0.005;
    public static final double LIFT_KI = 0;
    public static final double LIFT_KD = 0.0;

    public boolean isTrue = false;

    public Lift(LinearOpMode opMode){
        myOpMode = opMode;
    }
    public void init(){

        liftLeftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        liftRightPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        liftLeftPID.maxOut = 0.9;
        liftRightPID.maxOut = 0.9;


        liftLeft = myOpMode.hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = myOpMode.hardwareMap.get(DcMotor.class, "liftRight");

        touch = myOpMode.hardwareMap.get(TouchSensor.class, "touch");

        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void teleOp() {
        if (Math.abs(myOpMode.gamepad2.left_stick_y) > 0.3) {
            liftMode = LiftMode.MANUAL;
        }
        //myOpMode.telemetry.addData("Touch", "Pressed:" + touch.isPressed());

        myOpMode.telemetry.addData("TelemetryLeft", liftLeft.getCurrentPosition());
        myOpMode.telemetry.addData("TelemetryRight", liftRight.getCurrentPosition());


        /*if(isTrue = true)
            {
                liftLeft.setPower(-0.9);
                liftRight.setPower(-0.9);
            } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }*/

        /*if(touch.isPressed()){
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/
        if(myOpMode.gamepad2.right_stick_y > 0.1) {
            liftMode = LiftMode.HANG;
        }

        myOpMode.telemetry.addData("working", liftLeft.getCurrentPosition());
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(liftMode == LiftMode.MANUAL) {
            if (myOpMode.gamepad2.left_stick_y > 0.1) {
                liftLeft.setPower(-0.9);
                liftRight.setPower(-0.9);
            } else if (myOpMode.gamepad2.left_stick_y < -0.1 && liftLeft.getCurrentPosition() > -100) {
                liftLeft.setPower(0.9);
                liftRight.setPower(0.9);
            } else {
                liftLeft.setPower(0.1);
                liftRight.setPower(0.1);
            }
        } else if (liftMode == LiftMode.HIGH) {
            liftToPositionPIDClass(300);
        } else if (liftMode == LiftMode.MIDDLE) {
            liftToPositionPIDClass(200);
        } else if (liftMode == LiftMode.LOW) {
            liftToPositionPIDClass(700);
        } else if (liftMode == LiftMode.INTAKE) {
            liftToPositionPIDClass(0);
        } else if(liftMode == LiftMode.HANG){
            liftLeft.setPower(-1);
            liftRight.setPower(-1);
        }

        if(touch.isPressed()){
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
    public void liftToPositionPIDClass(double targetPosition) {
        double outLeft = liftLeftPID.calculate(targetPosition, liftLeft.getCurrentPosition());
        double outRight = liftRightPID.calculate(targetPosition, liftRight.getCurrentPosition());

        liftLeft.setPower(outLeft);
        liftRight.setPower(outRight);

        myOpMode.telemetry.addData("LiftLeftPower: ", outLeft);
        myOpMode.telemetry.addData("LiftRightPower: ", outRight);
    }

    public void resetLift(double speed){
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(touch.isPressed() == false){
           liftLeft.setPower(speed);
           liftRight.setPower(speed);
        } else {
            liftLeft.setPower(speed);
            liftRight.setPower(speed);
        }
    }
}
