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

    public static double HEADING_KP = 0.7;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static double DRIVE_KP = 0.07;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0;//0.0003;g
    public static double DRIVE_MAX_ACC = 2000;
    public static double DRIVE_MAX_VEL = 3500;
    public static double DRIVE_MAX_OUT = 0.8;

    PIDController xPID;
    PIDController yPID;
    PIDController headingPID;

    public Drivetrain(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init(){

        xPID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        yPID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

        xPID.maxOut = DRIVE_MAX_OUT;
        yPID.maxOut = DRIVE_MAX_OUT;
        headingPID.maxOut = DRIVE_MAX_OUT;

        driveFrontLeft = myOpMode.hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = myOpMode.hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackLeft = myOpMode.hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackRight = myOpMode.hardwareMap.get(DcMotor.class, "driveBackRight");

        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);


        localizer = new Localizer(myOpMode);

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void driveToPose(double xTarget, double yTarget, double thetaTarget){
        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xPID.calculate(xTarget, localizer.x);
        double yPower = yPID.calculate(yTarget, localizer.y);

        double wrappedAngle = angleWrap(Math.toRadians(thetaTarget)- localizer.heading);
        double tPower = headingPID.calculate(wrappedAngle);

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-localizer.heading) - yPower * Math.sin(-localizer.heading);
        double yPower_rotated = xPower * Math.sin(-localizer.heading) + yPower * Math.cos(-localizer.heading);

        // x, y, theta input mixing
        driveFrontLeft.setPower(-xPower_rotated + yPower_rotated + tPower);
        driveBackLeft.setPower(-xPower_rotated - yPower_rotated + tPower);
        driveFrontRight.setPower(-xPower_rotated - yPower_rotated - tPower);
        driveBackLeft.setPower(-xPower_rotated + yPower_rotated - tPower);
    }

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
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
