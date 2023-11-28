package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    private LinearOpMode myOpMode = null;

    static final double countspercm = 19.3566666;

    public DcMotor driveFrontLeft = null;
    public DcMotor driveFrontRight = null;
    public DcMotor driveBackLeft = null;
    public DcMotor driveBackRight = null;

    public Drivetrain(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init(){
        driveFrontLeft = myOpMode.hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = myOpMode.hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackLeft = myOpMode.hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackRight = myOpMode.hardwareMap.get(DcMotor.class, "driveBackRight");

        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void driveForwards(double motorPower, double distance){
        resetEncoders();
        double counts = distance*countspercm;
        while(driveFrontLeft.getCurrentPosition() < counts && myOpMode.opModeIsActive()){
            driveBackLeft.setPower(motorPower);
            //driveBackRight.setPower(motorPower);
            driveFrontLeft.setPower(motorPower);
            driveFrontRight.setPower(motorPower);

        }
        stopMotors();
    }

    public void resetEncoders(){
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders(){
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleOp() {
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 2);

        frontLeftPower = (drive + turn - strafe) / denominator;
        frontRightPower = (drive - turn + strafe) / denominator;
        backLeftPower = (drive + turn + strafe) / denominator;
        backRightPower = (drive - turn - strafe) / denominator;

        driveFrontLeft.setPower(frontLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackLeft.setPower(backLeftPower);
        driveBackRight.setPower(backRightPower);

    }
        public void driveForward(double time, double power){
            while(myOpMode.time < time){
                driveFrontLeft.setPower(power);
                driveFrontRight.setPower(power);
                driveBackLeft.setPower(power);
                driveBackRight.setPower(power);
            }
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void strafeRight(double time, double power){
            while(myOpMode.time < time){
                driveFrontLeft.setPower(-power);
                driveFrontRight.setPower(power);
                driveBackLeft.setPower(power);
                driveBackRight.setPower(-power);
            }
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
        public void strafeLeft(double time, double power){
            while(myOpMode.time < time){
                driveFrontLeft.setPower(power);
                driveFrontRight.setPower(-power);
                driveBackLeft.setPower(-power);
                driveBackRight.setPower(power);
            }
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        public void stopMotors(){
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

        }

        public void turn(double power){
            driveFrontLeft.setPower(power);
            driveFrontRight.setPower(-power);
            driveBackLeft.setPower(power);
            driveBackRight.setPower(-power);
        }
    }
