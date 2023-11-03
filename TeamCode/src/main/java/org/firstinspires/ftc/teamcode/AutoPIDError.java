package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous (name="Auto PID Error", group = "Concept")
public class AutoPIDError extends LinearOpMode {
    private Localizer localizer;

    private PIDController headingPID = new PIDController(0.01,0,0);

    private PIDController distancePID = new PIDController(0.01,0,0);;

    private DcMotor leftFrontDrive; //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive;  //  Used to control the right back drive wheel


    public void driveToPose(double targetX, double targetY, double targetHeading){

        double editedHeading = localizer.heading;

         if (editedHeading > 2 * Math.PI) {
             while (editedHeading > 2 * Math.PI){
                 editedHeading = editedHeading - 2 * Math.PI;
             }
         }
         
        //calculate error between current position and target position
        double xError = (localizer.x + targetX) / targetX;
        double yError = (localizer.y + targetY) / targetY;
        double headingError = (editedHeading + targetHeading) / targetHeading;

        //get PID output
        double outputHeading = headingPID.calculate(headingError);
        double outputDistanceX = distancePID.calculate(xError);
        double outputDistanceY = distancePID.calculate(yError);

        double outputDistancePID = distancePID.calculate(Math.sqrt((outputDistanceX * outputDistanceX) + (outputDistanceY * outputDistanceY)));

        double x_rotated = localizer.x * Math.cos(editedHeading) - localizer.y * Math.sin(editedHeading);
        double y_rotated = localizer.x * Math.sin(editedHeading) + localizer.y * Math.cos(editedHeading);
        //translate outputs to motor powers

        /*
        leftFrontDrive.setPower(localizer.x + localizer.y + outputDistancePID);
        leftBackDrive.setPower(localizer.x - localizer.y + outputDistancePID);
        rightFrontDrive.setPower(localizer.x - localizer.y - outputDistancePID);
        leftBackDrive.setPower(localizer. x + localizer.y - outputDistancePID);

         */
        leftFrontDrive.setPower(x_rotated + y_rotated + outputDistancePID);
        leftBackDrive.setPower(x_rotated - y_rotated + outputDistancePID);
        rightFrontDrive.setPower(x_rotated - y_rotated - outputDistancePID);
        leftBackDrive.setPower(x_rotated + y_rotated - outputDistancePID);

    }

    @Override public void runOpMode() {
        localizer = new Localizer(this);
        waitForStart();
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
