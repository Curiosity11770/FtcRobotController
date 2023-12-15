package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "CameraScanningBlueRight", group = "Linear OpMode")
public class CameraScanningBlueRight extends LinearOpMode {
    Robot robot = new Robot(this);
    Camera camera = new Camera(this);

    OpenCv.OpenCvPosition position = OpenCv.OpenCvPosition.LEFT;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){
        robot.init();
        camera.initAuto();
        position = camera.pipeline.getAnalysis();
        waitForStart();
        runtime.reset();

        if(opModeIsActive()) {
            telemetry.addData("Pipeline", position);
            telemetry.update();

            if (position == OpenCv.OpenCvPosition.RIGHT) {
                robot.drivetrain.driveForwards(0.7, 40);
                robot.drivetrain.strafeRight(0.7, 20);
                robot.drivetrain.strafeLeft(0.7, 20);
                //robot.intake.outtake(0.7, 3);

            } else if (position == OpenCv.OpenCvPosition.LEFT) {
                robot.drivetrain.driveForwards(0.7, 20);
                robot.drivetrain.strafeLeft(0.7, 20);
                //robot.intake.outtake(0.7, 3);

            } else {
                robot.drivetrain.driveForwards(0.7, 20);
                robot.drivetrain.strafeRight(0.7, 40);
                //robot.intake.outtake(0.7, 3);

            }
            robot.drivetrain.stopMotors();
            waitForStart();
        }

        //robot.drivetrain.strafeRight(0.7, 40);
        //robot.drivetrain.driveForwards(0.7, 20);
        //robot.drivetrain.stopMotors();

    }

}
