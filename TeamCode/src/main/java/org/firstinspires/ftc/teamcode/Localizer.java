package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Localizer {

    //Declare Constants
    //ticks per inch
    public static final double COUNTS_PER_INCH = 272;

    //track width - distance between odometry wheels
    public static final double TRACK_WIDTH = 13.68;

    //center wheel offset - distance from left and right wheel; '-' is behind, '+' is in front
    public static final double CENTER_OFFSET = 5;

    //Declare pose variables
    double x = 0;
    double y = 0;
    double heading = 0;

    double lastLeftPosition = 0;

    double lastRightPosition = 0;

    double lastCenterPosition = 0;

    //Declare OpMode Members
    //encoders
    DcMotorEx leftEncoder, rightEncoder, centerEncoder;
    //opMode
    LinearOpMode myOpMode;

    //Constructor with parameter for OpMode
    Localizer(LinearOpMode opMode){
        myOpMode = opMode;

        leftEncoder = myOpMode.hardwareMap.get(DcMotorEx.class,"leftBackDrive");
        rightEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        centerEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFrontDrive");

        //reset the encoder counts (stop and reset encoders)
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Methods

    //checkCounts
    void telemetry(){
        myOpMode.telemetry.addData("leftCounts:", leftEncoder.getCurrentPosition());
        myOpMode.telemetry.addData("rightCounts:", rightEncoder.getCurrentPosition());
        myOpMode.telemetry.addData("centerCounts:", centerEncoder.getCurrentPosition());
        myOpMode.telemetry.addData("X:", x);
        myOpMode.telemetry.addData("Y:", y);
        myOpMode.telemetry.addData("Heading:", heading);
    }

    void update(){
        double deltaLeftPosition = leftEncoder.getCurrentPosition() - lastLeftPosition;
        double deltaRightPosition = rightEncoder.getCurrentPosition() - lastRightPosition;
        double deltaCenterPosition = centerEncoder.getCurrentPosition() - lastCenterPosition;

        deltaLeftPosition = deltaLeftPosition / COUNTS_PER_INCH;
        deltaRightPosition = deltaRightPosition / COUNTS_PER_INCH;
        deltaCenterPosition = deltaCenterPosition / COUNTS_PER_INCH;


        double phi = (deltaLeftPosition - deltaRightPosition) / TRACK_WIDTH;
        double deltaMiddlePosition = (deltaLeftPosition + deltaRightPosition) / 2;
        double deltaPerpPosition = deltaCenterPosition -  CENTER_OFFSET * phi;

        double deltaX = deltaMiddlePosition * Math.cos(heading) - deltaPerpPosition * Math.sin(heading);
        double deltaY = deltaMiddlePosition * Math.sin(heading) + deltaPerpPosition * Math.cos(heading);

        x += deltaX;
        y += deltaY;
        heading += phi;

        lastLeftPosition = leftEncoder.getCurrentPosition();
        lastRightPosition = rightEncoder.getCurrentPosition();
        lastCenterPosition = centerEncoder.getCurrentPosition();

        myOpMode.telemetry.addData("Track Width", (deltaLeftPosition - deltaRightPosition) / (20 * Math.PI));
    }


    //update pose

    //return x

    //return y

    //return heading

}
