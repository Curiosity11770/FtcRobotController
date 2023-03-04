package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.util.Path;

@Config
@Autonomous
public class RegAutoTEST extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleTankDrive robot = new SampleTankDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        //create the path object
        Path path1 = new Path(robot, 36, 10, true);

        //while the path is incomplete
        while (!path1.targetReached) {
            //follow the path!
            path1.followPath();

            //telemetry to check state
            telemetry.addData("path state", path1.state);
            //telemetry to check agreement of angle formats
            telemetry.addData("robot Heading", path1.currentHeading);
            telemetry.addData("angle to target", path1.theta);
            //telemetry to check distance
            telemetry.addData("distance to target", path1.currentDistance);
            //telemetry to check outputs
            telemetry.addData("f", path1.f);
            telemetry.addData("t", path1.t);
            telemetry.update();
        }
    }
}
