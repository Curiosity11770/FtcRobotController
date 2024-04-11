package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous (name="LocalizerTest", group = "Concept")
public class LocalizerTest extends LinearOpMode {
    private Robot robot;

    public static double tX, tY, tHeading;
    public double oX,oY,oHeading;
    boolean following = false;

    public State state = State.MANUAL;

    enum State{
        TARGET,
        ORIGIN,
        MANUAL
    }

    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        tX = 24;
        tY = 0;
        tHeading = 0;

        oX = 0;
        oY = 0;
        oHeading = 0;

        robot.drivetrain.localizer.setCoordinates(0,0,0);

        waitForStart();

        while (opModeIsActive())
        {
            robot.drivetrain.localizer.telemetry();

            robot.drivetrain.localizer.update();

            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard dashboard = FtcDashboard.getInstance();

            robot.drivetrain.localizer.drawRobot(packet.fieldOverlay());
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();

            //robot.drivetrain.updateDriveConstants();

            if(gamepad1.a){
                state = State.TARGET;
            }else if(gamepad1.x){
                state = State.ORIGIN;
            }else if(gamepad1.b){
                state = State.MANUAL;
            }

            if(state == State.TARGET) {
                robot.drivetrain.driveToPose(tX, tY, tHeading, 3);
            }else if(state == State.ORIGIN) {
                robot.drivetrain.driveToPose(oX,oY,oHeading,3);
            }else if(state == State.MANUAL){
                robot.drivetrain.teleOpBlue();
            }
        }
    }
}