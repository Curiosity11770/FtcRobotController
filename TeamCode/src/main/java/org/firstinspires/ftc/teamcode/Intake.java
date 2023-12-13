package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Intake {

    private LinearOpMode myOpMode = null;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    public DcMotor intakeMotor = null;
    public boolean inAction = false;
    public DigitalChannel beamBreaker1;
    public DigitalChannel beamBreaker2;

    public Intake(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public void init(){
        intakeLeft = myOpMode.hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = myOpMode.hardwareMap.get(CRServo.class, "intakeRight");
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");

        beamBreaker1 = myOpMode.hardwareMap.digitalChannel.get("switch");
        beamBreaker2 = myOpMode.hardwareMap.digitalChannel.get("switch");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void teleOp(){
        boolean passed1 = beamBreaker1.getState();

        boolean switchState1;
        if (passed1) {
            switchState1 = false;
        } else {
            switchState1 = true;
        }
        myOpMode.telemetry.addData("state", ":  " + switchState1);

        boolean passed2 = beamBreaker2.getState();

        Boolean switchState2;
        if (passed2) {
            switchState2 = false;
        } else {
            switchState2 = true;
        }
        myOpMode.telemetry.addData("state", ":  " + switchState2);
        if (switchState1 & switchState2){
            if (myOpMode.gamepad2.right_trigger > 0.2) {
                inAction = true;
                intakeLeft.setPower(-0.7);
                intakeRight.setPower(0.7);
                intakeMotor.setPower(0.7);
            } else if (myOpMode.gamepad2.left_trigger > 0.2) {
                inAction = true;
                intakeLeft.setPower(-0.7);
                intakeRight.setPower(0.7);
                intakeMotor.setPower(0.7);
            } else {
                inAction = false;
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeMotor.setPower(0);
            }
        }
        else {
            if (myOpMode.gamepad2.right_trigger > 0.2) {
                inAction = true;
                intakeLeft.setPower(0.7);
                intakeRight.setPower(-0.7);
                intakeMotor.setPower(-0.7);
            } else if (myOpMode.gamepad2.left_trigger > 0.2) {
                inAction = true;
                intakeLeft.setPower(-0.7);
                intakeRight.setPower(0.7);
                intakeMotor.setPower(0.7);
            } else {
                inAction = false;
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeMotor.setPower(0);
            }
        }
    }
}
