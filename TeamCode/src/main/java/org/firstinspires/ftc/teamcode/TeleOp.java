package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode {
    Robot robot = new Robot(this);

    //private ElapsedTime = new ElapsedTime();

    public void runOpMode(){
        robot.initTeleOp();
        robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
        robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
        telemetry.addData("Status", "Intialized");

        robot.drivetrain.localizer.setCoordinates(-36,64, Math.toRadians(270));
        waitForStart();
        //runtime.reset();

        while(opModeIsActive()){
            robot.teleOp();
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
