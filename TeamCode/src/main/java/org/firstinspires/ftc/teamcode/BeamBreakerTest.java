package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Beam Breaker Test", group = "Sensor")
//@Disabled
public class BeamBreakerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the hardware
         */

        DigitalChannel beamBreaker;

        beamBreaker = hardwareMap.digitalChannel.get("switch");

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            // is it on or off?

            boolean passed = beamBreaker.getState();

            String switchState;
            if (passed) {
                switchState = "No pixel";
            } else {
                switchState = "Has pixel";
            }
            telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
            telemetry.addData("state", ":  " + switchState);
            telemetry.update();
        }
    }

}
