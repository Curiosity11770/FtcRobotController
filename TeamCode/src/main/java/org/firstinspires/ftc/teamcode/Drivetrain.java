package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime time = new ElapsedTime();

    //private ElapsedTime timeout = null;

    //drivetrain
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    public MecanumDrive mecanumDrive = null;

    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){

        time.reset();
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }
    public void teleOp() {
        //drive train
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

        if (myOpMode.gamepad1.right_bumper) {
            leftFrontDrive.setPower(frontLeftPower / 7);
            rightBackDrive.setPower(frontRightPower / 7);
            leftBackDrive.setPower(backLeftPower / 7);
            rightBackDrive.setPower(backRightPower / 7);
        } else if (myOpMode.gamepad1.left_bumper) {
            leftFrontDrive.setPower(2 * frontLeftPower);
            rightFrontDrive.setPower(2 * frontRightPower);
            leftBackDrive.setPower(2 * backLeftPower);
            rightBackDrive.setPower(2 * backRightPower);
        } else {
            leftBackDrive.setPower(frontLeftPower);
            leftFrontDrive.setPower(frontRightPower);
            rightFrontDrive.setPower(backLeftPower);
            rightBackDrive.setPower(backRightPower);
        }
    }

    public void driveTime(double speed, double seconds) {
        time.reset();
        while(myOpMode.opModeIsActive() && seconds < time.seconds()){
            leftBackDrive.setPower(speed);
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightBackDrive.setPower(speed);
        }
    }
    public void strafeTime(double speed, double seconds) {
        time.reset();
        while(myOpMode.opModeIsActive() && seconds < time.seconds()){
            leftBackDrive.setPower(-speed);
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(-speed);
            rightBackDrive.setPower(speed);
        }
    }



}
