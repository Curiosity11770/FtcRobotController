package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    private LinearOpMode myOpMode = null;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    public DcMotor intakeMotor = null;
    public boolean inAction = false;
    public NormalizedColorSensor colorFront;
    public NormalizedColorSensor colorBack;

    public boolean frontPixel, backPixel;

    View relativeLayout;

    public enum IntakeMode {
        INTAKE,
        OUTTAKE,
        OFF
    }

    public IntakeMode state = IntakeMode.OFF;

    public ElapsedTime timer = new ElapsedTime();

    public Intake(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        intakeLeft = myOpMode.hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = myOpMode.hardwareMap.get(CRServo.class, "intakeRight");
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");


        colorFront = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorFront");
        colorBack = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorBack");

        int relativeLayoutId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", myOpMode.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) myOpMode.hardwareMap.appContext).findViewById(relativeLayoutId);


        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void teleOp() {

        myOpMode.telemetry.addData("Distance Left (cm)", "%.3f", ((DistanceSensor) colorFront).getDistance(DistanceUnit.CM));
        myOpMode.telemetry.addData("Distance Right (cm)", "%.3f", ((DistanceSensor) colorBack).getDistance(DistanceUnit.CM));

        if(myOpMode.gamepad2.a || myOpMode.gamepad2.x) {

            backPixel = false;
        }
            else if(((DistanceSensor) colorFront).getDistance(DistanceUnit.CM) <= 4){
            myOpMode.gamepad2.rumble(.25,0,500);
            frontPixel = true;
        }else{
            frontPixel = false;

        }
        if(myOpMode.gamepad2.a || myOpMode.gamepad2.x){
            backPixel = false;
        }
        else if(((DistanceSensor) colorBack).getDistance(DistanceUnit.CM) <= 2){
            myOpMode.gamepad2.rumble(0,0.5,500);
            backPixel = true;
        }else{
            backPixel = false;
        }

        if(state == IntakeMode.INTAKE) {
            intakeLeft.setPower(0.7);
            intakeRight.setPower(-0.7);
            intakeMotor.setPower(-0.7);

            if(frontPixel && backPixel){
                state = IntakeMode.OUTTAKE;
            }
        } else if (state == IntakeMode.OUTTAKE){
            intakeLeft.setPower(-0.7);
            intakeRight.setPower(0.7);
            intakeMotor.setPower(0.7);

        } else if(state == IntakeMode.OFF){
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            intakeMotor.setPower(0);
        }
        if (myOpMode.gamepad2.right_trigger > 0.2) {
            state = IntakeMode.INTAKE;

        } else if (myOpMode.gamepad2.left_trigger > 0.2) {
            state = IntakeMode.OUTTAKE;
            timer.reset();
        } else if(myOpMode.gamepad2.dpad_left){
            state = IntakeMode.OFF;
        } else{
            state = IntakeMode.OFF;
        }
        // }
    }

    public void outtake(double motorPower) {
        timer.reset();
        //while(timer.seconds()) {
            intakeLeft.setPower(-motorPower);
            intakeRight.setPower(motorPower);
            intakeMotor.setPower(motorPower);
        //
        // }

    }
    public void outtake2(double motorPower, double timeOut) {
        timer.reset();
        while(timer.seconds() < timeOut) {
        intakeLeft.setPower(-motorPower);
        intakeRight.setPower(motorPower);
        intakeMotor.setPower(motorPower);

         }
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        intakeMotor.setPower(0);

    }
    public void update(){
        if(((DistanceSensor) colorFront).getDistance(DistanceUnit.CM) <= 4){
            myOpMode.gamepad2.rumble(.25,0,500);
            frontPixel = true;
        }else{
            frontPixel = false;
        }
        if(((DistanceSensor) colorBack).getDistance(DistanceUnit.CM) <= 2){
            myOpMode.gamepad2.rumble(0,0.5,500);
            backPixel = true;
        }else {
            backPixel = false;
        }

        if(state == IntakeMode.INTAKE) {
            intakeLeft.setPower(0.7);
            intakeRight.setPower(-0.7);
            intakeMotor.setPower(-0.7);

        } else if (state == IntakeMode.OUTTAKE){
            intakeLeft.setPower(-0.7);
            intakeRight.setPower(0.7);
            intakeMotor.setPower(0.7);

        } else if(state == IntakeMode.OFF){
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            intakeMotor.setPower(0);
        }

    }
    public void reset(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        intakeMotor.setPower(0);
    }
}