//CHECKLIST

/*

AUTO
- Troubleshoot Intake
- Other Four Sides
TELEOP
- SWITCH STATE MANUAL FIELD
- TURBO/SLOW
- RED/BLUE TELEOP
- TEST TRANSITION BETWEEN AUTO AND TELEOP
 */
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RedTeleOp", group = "TeleOp")
public class RedTeleOp extends LinearOpMode {
    Robot robot = new Robot(this);

    //private ElapsedTime = new ElapsedTime();

    public void runOpMode(){
        robot.initTeleOp();
        robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
        robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
        telemetry.addData("Status", "Intialized");

        robot.drivetrain.localizer.setCoordinates(PoseStorage.currentPose.getX(),
                PoseStorage.currentPose.getY(), PoseStorage.currentPose.getHeading());

        waitForStart();
        //runtime.reset();

        while(opModeIsActive()){
            robot.teleOpRed();
            //robot.touchSense();
            robot.drivetrain.localizer.update();
            robot.drivetrain.localizer.telemetry();

            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard dashboard = FtcDashboard.getInstance();

            robot.drivetrain.localizer.drawRobot(packet.fieldOverlay());
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }


}

