package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous (name="Omni Drive To AprilTag", group = "Concept")
public class AutoPIDError extends LinearOpMode {
    private Localizer localizer;

    private PIDController headingPID = new PIDController(2,2,2);

    private PIDController distancePID = new PIDController(2,2,2);;

    private DcMotor leftFrontDrive; //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive;  //  Used to control the right back drive wheel

    public void driveToPose(double targetX, double targetY, double targetHeading){
        //calculate error between current position and target position
        double xError = (localizer.x + targetX) / targetX;
        double yError = (localizer.y + targetY) / targetY;
        double headingError = (localizer.heading + targetHeading) / targetHeading;

        //get PID output
        double outputHeading = headingPID.calculate(headingError);
        double outputDistanceX = distancePID.calculate(xError);
        double outputDistanceY = distancePID.calculate(yError);

        //translate outputs to motor powers


    }

    @Override public void runOpMode() {
        localizer = new Localizer(this);

        while (opModeIsActive())
        {

            localizer.telemetry();

            localizer.update();

            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard dashboard = FtcDashboard.getInstance();

            localizer.drawRobot(packet.fieldOverlay());
            dashboard.sendTelemetryPacket(packet);

        }
    }

}
