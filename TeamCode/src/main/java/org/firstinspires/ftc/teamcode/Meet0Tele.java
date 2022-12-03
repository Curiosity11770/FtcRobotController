package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.collection.CircularArray;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp //used to say "extends LinearOpMode but that got error
public class Meet0Tele extends LinearOpMode {

    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor verticalLeft = null;
    private DcMotor horizontal = null;


    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;

    private DcMotor pivot = null;

    private CRServo intake = null;

    private int liftTarget;
    private int pivotTarget;

    private String height = "GROUND";

    private double drivePower;

    //@Override         doesn't like override?
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Test", "Good");
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        verticalLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        horizontal = hardwareMap.get(DcMotor.class, "backRight");

        liftLeft = hardwareMap.get(DcMotor.class, "leftSlides");
        liftRight = hardwareMap.get(DcMotor.class, "rightSlides");

        pivot = hardwareMap.get(DcMotor.class, "pivot");

        intake = hardwareMap.get(CRServo.class, "intakeServo");


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot.setDirection(DcMotor.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetLiftEncoders();
        resetPivotEncoders();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            double rightDrivePower = 0.5;
            double leftDrivePower = 0.5;
            double liftPower = 0.5;
            double pivotPower = 0.4;
            double intakePower = 0.5;

            //boolean intakeIn = false;

            telemetry.addData("LR ticks", liftRight.getCurrentPosition());
            telemetry.addData("LL ticks", liftLeft.getCurrentPosition());
            telemetry.addData("P ticks", pivot.getCurrentPosition());
           // telemetry.update();

            //do we want it to be set up this way? we can also do
            //frontLeft.setPower(-gamepad1.left_stick_y);
            //backLeft.setPower(-gamepad1.left_stick_y);

            //frontRight.setPower(-gamepad1.right_stick_y);
            //backRight.setPower(-gamepad1.right_stick_y);

            if(gamepad1.left_stick_y > 0.2){
                frontLeft.setPower(-leftDrivePower);
                backLeft.setPower(-leftDrivePower);
            } else if(gamepad1.left_stick_y < -0.2){
                frontLeft.setPower(leftDrivePower);
                backLeft.setPower(leftDrivePower);
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
            }

            if(gamepad1.right_stick_y > 0.2){
                frontRight.setPower(-rightDrivePower);
                backRight.setPower(-rightDrivePower);
            } else if(gamepad1.right_stick_y < -0.2){
                frontRight.setPower(rightDrivePower);
                backRight.setPower(rightDrivePower);
            } else {
                frontRight.setPower(0);
                backRight.setPower(0);
            }

            if(gamepad2.right_stick_y > 0.2 && Math.abs(liftLeft.getCurrentPosition()) < 3000 && Math.abs(liftRight.getCurrentPosition()) < 3000){
                liftLeft.setPower(liftPower);
                liftRight.setPower(liftPower);
            } else if(gamepad2.right_stick_y < -0.2){
                liftLeft.setPower(-liftPower);
                liftRight.setPower(-liftPower);
            }
            else
            {
                liftLeft.setPower(0.0);
                liftRight.setPower(0.0);
            }
            /*
             if(gamepad2.y) { height = "GROUND"; }
            else if(gamepad2.x){ height = "LOW"; }
            else if(gamepad2.b){ height = "MIDDLE"; }
            else if(gamepad2.a){ height = "HIGH"; }

            liftPosition(0.3, height);
            */

            //if(pivot.getCurrentPosition() < 100){
               // pivot.setPower(0);
            //} else {
                if (gamepad2.left_stick_x > 0.2) {
                    pivot.setPower(pivotPower);
                } else if (gamepad2.left_stick_x < -0.2) {
                    pivot.setPower(-pivotPower);
                } /* else if (gamepad2.DpadLeft) { pivotPosition(0.1, "LEFT")}
            else if(gamepad2.DpadUp) { pivotPosition(0.1, "CENTER")}
            else if(gamepad2.DpadRight) { pivotPosition(0.1, "RIGHT")}*/
                else {
                    pivot.setPower(0);
                }

            if(gamepad2.right_bumper) {
                intake.setPower(intakePower);
            } else if(gamepad2.left_bumper){
                intake.setPower(-intakePower);
            } else {
                intake.setPower(0);
            }
        }
    }

    private void pivotPosition(double motorPower, String side){
        if(side.equals("LEFT")){     //LEFT
            pivotTarget = 0;

        } else if (side.equals("RIGHT")){    //RIGHT
            pivotTarget = 10;
        } else if (side.equals("CENTER")){    //CENTER
            pivotTarget = 20;
        }

        pivot.setTargetPosition(pivotTarget);

        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorPower = 0.1;
        pivot.setPower(motorPower);

        if(opModeIsActive() && pivot.isBusy()){
            telemetry.addData("pivot", pivot.getCurrentPosition());
            telemetry.addData("Target", pivotTarget);
            telemetry.update();
        }

        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void liftPosition(double motorPower, String height){
        if(height.equals("GROUND")){     //GROUND
            liftTarget = 0;

        } else if (height.equals("LOW")){    //LOW
            liftTarget = 100;
        } else if (height.equals("MIDDLE")){    //MIDDLE
            liftTarget = 2200;
        } else {                    //HIGH
            liftTarget = 2900;
        }

        liftLeft.setTargetPosition(-liftTarget);
        liftRight.setTargetPosition(-liftTarget);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //motorPower = 0.1;
        liftLeft.setPower(motorPower);
        liftRight.setPower(motorPower);

        if(opModeIsActive() && liftLeft.isBusy()){
            telemetry.addData("Right", liftRight.getCurrentPosition());
            telemetry.addData("left", liftLeft.getCurrentPosition());
            telemetry.addData("Target", liftTarget);
            telemetry.update();
        }
        /*
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

    }

    private void resetLiftEncoders(){
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetPivotEncoders(){
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
