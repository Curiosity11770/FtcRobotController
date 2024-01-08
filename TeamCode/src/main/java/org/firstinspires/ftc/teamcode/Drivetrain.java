package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    private LinearOpMode myOpMode = null;

   public Localizer localizer;

    public DcMotor driveFrontLeft = null;
    public DcMotor driveFrontRight = null;
    public DcMotor driveBackLeft = null;
    public DcMotor driveBackRight = null;

    public double counts2 = 19.35666666;

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

        localizer = new Localizer(myOpMode);
        useEncoders();
    }

    public void resetEncoders(){
        localizer.centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localizer.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localizer.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders(){
        localizer.centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        localizer.leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        localizer.rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

       myOpMode.telemetry.addData("Counts", localizer.leftEncoder.getCurrentPosition());
        myOpMode.telemetry.addData("Counts", localizer.rightEncoder.getCurrentPosition());

    }
        public void driveForwards(double motorPower, double distance){
            resetEncoders();
            useEncoders();
            double counts = distance*20;
            while(localizer.rightEncoder.getCurrentPosition()*-1 < counts && myOpMode.opModeIsActive()){
                myOpMode.telemetry.addData("Right", localizer.rightEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Left", localizer.leftEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Center", localizer.centerEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Counts", counts);
                myOpMode.telemetry.update();
                driveBackLeft.setPower(motorPower);
                driveBackRight.setPower(motorPower);
                driveFrontLeft.setPower(motorPower);
                driveFrontRight.setPower(motorPower);
            }
            stopMotors();
        }

        public void driveBackwards(double motorPower, double distance){
            resetEncoders();
            useEncoders();
            double counts = distance*20;
            while(localizer.rightEncoder.getCurrentPosition() < counts && myOpMode.opModeIsActive()){
                myOpMode.telemetry.addData("Right", localizer.rightEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Left", localizer.leftEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Center", localizer.centerEncoder.getCurrentPosition());
                myOpMode.telemetry.update();
                driveBackLeft.setPower(motorPower);
                driveBackRight.setPower(motorPower);
                driveFrontLeft.setPower(motorPower);
                driveFrontRight.setPower(motorPower);
            }
            stopMotors();
        }

        public void strafeRight(double motorPower, double distance){
            resetEncoders();
            useEncoders();
            double counts = distance;
            while(localizer.rightEncoder.getCurrentPosition() < counts && myOpMode.opModeIsActive()){
                myOpMode.telemetry.addData("Right", localizer.rightEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Left", localizer.leftEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Center", localizer.centerEncoder.getCurrentPosition());
                myOpMode.telemetry.update();
                driveBackLeft.setPower(-motorPower);
                driveBackRight.setPower(motorPower);
                driveFrontLeft.setPower(motorPower);
                driveFrontRight.setPower(-motorPower);
            }
            stopMotors();
        }
        public void strafeLeft(double motorPower, double distance) {
            resetEncoders();
            useEncoders();
            double counts = distance*20;
            while (localizer.rightEncoder.getCurrentPosition() * -1 < counts && myOpMode.opModeIsActive()) {
                myOpMode.telemetry.addData("Right", localizer.rightEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Left", localizer.leftEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Center", localizer.centerEncoder.getCurrentPosition());
                myOpMode.telemetry.addData("Counts", counts);
                myOpMode.telemetry.update();
                driveBackLeft.setPower(motorPower);
                driveBackRight.setPower(-motorPower);
                driveFrontLeft.setPower(-motorPower);
                driveFrontRight.setPower(motorPower);
            }
            stopMotors();
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
