package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Touch Sensor Test", group = "Sensor")
public class TouchSensorTest extends LinearOpMode
{

    TouchSensor touch;

    @Override
    public void runOpMode(){
        touch = hardwareMap.get(TouchSensor.class, "touch");
        waitForStart();

        while(opModeIsActive()){
            /*
            telemetry.addData("Touch", "Pressed:" + touch.isPressed());
            telemetry.update();*/
            if(touch.isPressed()){
                telemetry.addData("touch", "pressed");
            }
            else{
                telemetry.addData("touch", "not pressed");
            }
            telemetry.update();
        }

    }

}
