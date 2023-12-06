package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private LinearOpMode myOpMode = null;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    public DcMotor intakeMotor = null;

    public Intake(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void init(){
        intakeLeft = myOpMode.hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = myOpMode.hardwareMap.get(CRServo.class, "intakeRight");
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void teleOp(){
        if(myOpMode.gamepad2.right_trigger > 0.2){
           intakeLeft.setPower(0.7);
           intakeRight.setPower(-0.7);
           intakeMotor.setPower(-0.7);
        } else if(myOpMode.gamepad2.left_trigger > 0.2){
            intakeLeft.setPower(-0.7);
            intakeRight.setPower(0.7);
            intakeMotor.setPower(0.7);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            intakeMotor.setPower(0);
        }
    }

    /*public void outtake(double power, double time){
        while(myOpMode.time < time){
            intakeLeft.setPower(power);
            intakeRight.setPower(power);
            intakeMotor.setPower(power);
        }
    }*/


}
