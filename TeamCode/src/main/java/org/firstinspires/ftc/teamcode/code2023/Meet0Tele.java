package org.firstinspires.ftc.teamcode.code2023;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.collection.CircularArray;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

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

    private Servo pivot = null;

    private CRServo intake = null;

    private Servo align = null;

    private Servo barRight = null;
    private Servo barLeft = null;

    private int liftTarget;

    private TouchSensor touch = null;
    private String height = "MANUAL";

    private double drivePower;

    private double pivotAdjustment = 0.005;
    private boolean tankDrive = false;

    private boolean turbo = false;
    private boolean barDeployed = false;
    private boolean barOverride = false;

    private boolean reset = false;
    private boolean sweeperManual = false;

    private double pPos = 0;
    private double pivotTarget = 0.54;
    private boolean manual;

    private double rightPower;
    private double leftPower;
    private double liftPower = 0.5;
    private double pivotPower = 0.3;
    private double intakePower = 0.7;

    //@Override         doesn't like override?
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Test", "Good");
        ;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        verticalLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        horizontal = hardwareMap.get(DcMotor.class, "backRight");


        liftLeft = hardwareMap.get(DcMotor.class, "leftSlides");
        liftRight = hardwareMap.get(DcMotor.class, "rightSlides");

        //pivot = hardwareMap.get(DcMotor.class, "pivot");
        pivot = hardwareMap.get(Servo.class, "pivot");

        intake = hardwareMap.get(CRServo.class, "intakeServo");

        align = hardwareMap.get(Servo.class, "sweep");

        barRight = hardwareMap.get(Servo.class, "wbRight");
        barLeft = hardwareMap.get(Servo.class, "wbLeft");

        touch = hardwareMap.get(TouchSensor.class, "touchSensor");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pivot.setDirection(DcMotor.Direction.REVERSE);
        //pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //resetLift();
        resetLiftEncoders();
        //resetPivotEncoders();

        //barLeft.setPosition(0.0); //0.0 is upright
        //barRight.setPosition(1.0); //1.0 is upright
        barIn();

        pivot.setPosition(0.54);
        //manual = true;

        align.setPosition(1.0);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            //boolean intakeIn = false;

            telemetry.addData("LR ticks", liftRight.getCurrentPosition());
            telemetry.addData("LL ticks", liftLeft.getCurrentPosition());
            telemetry.addData("P ticks", pivot.getPosition());
           // telemetry.update();

            pPos = pivot.getPosition();

            //do we want it to be set up this way? we can also do
            //frontLeft.setPower(-gamepad1.left_stick_y);
            //backLeft.setPower(-gamepad1.left_stick_y);

            //frontRight.setPower(-gamepad1.right_stick_y);
            //backRight.setPower(-gamepad1.right_stick_y);

            if(gamepad1.right_trigger > 0.2){
                tankDrive = !tankDrive;
            }


            //if(gamepad1.left_bumper){
                //turbo = !turbo;
            //}

            if(tankDrive) {

                leftPower = -gamepad1.left_stick_y/2;
                rightPower = -gamepad1.right_stick_y/2;
            } else if (!tankDrive){
                leftPower = -gamepad1.left_stick_y/2 + gamepad1.right_stick_x/2;
                rightPower = -gamepad1.left_stick_y/2 - gamepad1.right_stick_x/2;
            }
            if(gamepad1.right_bumper){
                leftPower = leftPower*1.95;
                rightPower = rightPower*1.95;
            } else if(gamepad1.left_bumper){
                leftPower = leftPower/2;
                rightPower = rightPower/2;
            }

            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);


            if(gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2){
                height = "MANUAL";
            }
            else if(gamepad2.y){ height = "GROUND"; }
            else if(gamepad2.x){ height = "MIDDLE"; }
            //else if(gamepad2.b){ height = "MIDDLE"; }
            else if(gamepad2.a){ height = "HIGH"; }

            if(gamepad2.b){
                pivotAdjustment = 0.009;
            }
            else{
                pivotAdjustment = 0.005;
            }

            //height = "MANUAL"; //set manual at the very beginning
            liftPosition(0.65, height);

            barPosition();
            //piv
            //if(pivot.getCurrentPosition() < 100){
               // pivot.setPower(0);
            //} else {

                //manual
               if (gamepad2.left_stick_x > 0.2 || gamepad2.left_stick_x < -0.2) {
                   //pivot.setPower(gamepad2.left_stick_x/2.5);
                   manual = true;
                   //pivot.setPosition(pPos+0.005);
               }
               //set positions

               if (gamepad2.left_bumper) { pivotTarget = 0.2; manual = false; }
               //else if(gamepad2.dpad_up) { pivotTarget = 0.5; manual = false; }
               else if(gamepad2.right_bumper) { pivotTarget = 0.8; manual = false; }
               //else {
                 //  pivot.setPower(0);
               //}

            if(gamepad2.y){
                reset = true;
            } else if(liftRight.getCurrentPosition() < 300 && align.getPosition() == 1.0){
                reset = false;
            }

            pivotPosition(manual, pivotTarget);
            setSweeper(reset);

            //pivot.setPower(gamepad
            // 2.left_stick_x/2.5);

            if(gamepad2.right_trigger > 0.2) {
                intake.setPower(intakePower);
            } else if(gamepad2.left_trigger > 0.2){
                intake.setPower(-intakePower);
            } else {
                intake.setPower(0);
            }
            telemetry.update();
        }
    }

    private void pivotPosition(boolean manual, double pivotTarget){

        if(manual) {
            //0.005
            if (gamepad2.left_stick_x > 0.2) {
                pivot.setPosition(pPos - pivotAdjustment);
            } else if (gamepad2.left_stick_x < -0.2) {
                pivot.setPosition(pPos + pivotAdjustment);
            }
        }
        else {
            if(Math.abs(pPos-pivotTarget) >= 0.02) {
                if (pPos > pivotTarget) {
                    pivot.setPosition(pPos - 0.02);
                } else if (pPos < pivotTarget) {
                    pivot.setPosition(pPos + 0.02);
                }
            }
        }
    }

    private void barOut(){
        double distance = 0.55;
        barLeft.setPosition(distance);
        barRight.setPosition(1.0-0.52);

    }

    private void barIn(){
        barLeft.setPosition(0.05);
        barRight.setPosition(0.98);
    }

    /*private void pivotPositionOLD(double motorPower, String side){
        if(side.equals("LEFT")){     //LEFT
            pivotTarget = -663;

        } else if (side.equals("RIGHT")){    //RIGHT
            pivotTarget = 717;
        } else if (side.equals("CENTER")){    //CENTER
            pivotTarget = 0;
        }

        pivot.setTargetPosition(pivotTarget);

        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //motorPower = 0.1;
        pivot.setPower(motorPower);

        if(opModeIsActive() && pivot.isBusy()){
            telemetry.addData("pivot", pivot.getCurrentPosition());
            telemetry.addData("Target", pivotTarget);
            //telemetry.update();
        }

        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }*/
    //positive encoder values and motor powers are UP
    private void barPosition(){

        if(gamepad1.dpad_up){
            barDeployed = false;
            barOverride = true;
        } else if(gamepad1.dpad_down){
            barDeployed = true;
            barOverride = false;
        } else if(liftRight.getCurrentPosition() > 600 && !barOverride){
            barDeployed = true;
        } else if(liftRight.getCurrentPosition() < 600 && !barOverride){
            barDeployed = false;
        }

        //liftRight.getCurrentPosition()
        if (barDeployed){
            barOut();
        }
        else {
            barIn();
        }
    }

    private void setSweeper(boolean reset) {
        //left further, right less
        //swinging to right is lower
        //pivot to the left
        if(gamepad2.dpad_up){
            sweeperManual = true;
        } else if(gamepad2.dpad_down){
            sweeperManual = false;
        }

        if(!sweeperManual) {
            if (!reset) {
                if (pPos > 0.75) {
                    //left side
                    align.setPosition(0.32);
                } else if (pPos < 0.25) {
                    //right side
                    align.setPosition(0.38);
                } else {
                    align.setPosition(1.0);
                }
            }
        } else if(sweeperManual){
            align.setPosition(1.0);
        }
    }

    private void liftPosition(double motorPower, String height){
        if(height.equals("MANUAL")){
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //add touch sensor lower bound (if touch sensor not pressed)
            if(gamepad2.right_stick_y > 0.2 && Math.abs(liftLeft.getCurrentPosition()) < 3000 && Math.abs(liftRight.getCurrentPosition()) < 3000){
                liftLeft.setPower(-motorPower);
                liftRight.setPower(-motorPower);
                //make proportional to right_stick_y? (like drivetrain)
            } else if(gamepad2.right_stick_y < -0.2){
                liftLeft.setPower(motorPower);
                liftRight.setPower(motorPower);
            }
            else
            {
                liftLeft.setPower(0.1);
                liftRight.setPower(0.1);
            }
        }
        else if(height.equals("GROUND")){     //GROUND
            //resetLift();
            liftTarget = 0;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            align.setPosition(1.0);
            pivotPosition(false, 0.54);

            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("left", liftLeft.getTargetPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if (height.equals("LOW")){    //LOW
            liftTarget = 600;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("left", liftLeft.getTargetPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if (height.equals("MIDDLE")){    //MIDDLE
            liftTarget = 2000;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else if(height.equals("HIGH")){                    //HIGH
            liftTarget = 2600;

            liftLeft.setTargetPosition(liftTarget);
            liftRight.setTargetPosition(liftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //motorPower = 0.1;
            liftLeft.setPower(-motorPower);
            liftRight.setPower(motorPower);

            if(opModeIsActive() && liftLeft.isBusy()){
                telemetry.addData("Right", liftRight.getCurrentPosition());
                telemetry.addData("left", liftLeft.getCurrentPosition());
                telemetry.addData("Target", liftTarget);
                //telemetry.update();
            }
        } else {
            height = "MANUAL";
        }
    }
    /*
    private void resetLift(){
        if(!touch.isPressed()){
            liftLeft.setPower(-0.5);
            liftRight.setPower(-0.5);
        } else if(touch.isPressed()){
            liftLeft.setPower(0);
            liftRight.setPower(0);
            resetLiftEncoders();
        }
    }

    */

    private void resetLiftEncoders(){
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*private void resetPivotEncoders(){
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }*/
}
