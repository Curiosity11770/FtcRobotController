package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Meet0Auto", group = "Linear Opmode")
public class Meet0Auto extends LinearOpMode{
    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init();
        runtime.reset();

        waitForStart();

        robot.drivetrain.strafeTime(-0.7, 2);
        robot.drivetrain.strafeTime(0.7, 2);
        robot.drivetrain.driveTime(0.7, 2);
    }

}
