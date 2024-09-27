package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LocalizerBlueLeft extends LinearOpMode {
    private Robot robot;

    public static double oX = 0;
    public static double oY = 0;
    public static double oT = 0;

    private ElapsedTime runtime = new ElapsedTime();


    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        robot.drivetrain.localizer.setCoordinates(oX, oY, oT);

        while (!isStarted()) {
            robot.drivetrain.localizer.update();
            robot.drivetrain.localizer.updateDashboard();
            robot.camera.scanAprilTag(5);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        // robot.camera.stopColorProcessor();
        robot.camera.visionPortalFront.stopStreaming();

        waitForStart();
        //Drives to Spike Mark
        if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE) {
            //Drive to Spike Mark
            robot.drivetrain.driveToPose(41, 6, -90, 2);
            //Outtake and go to pixel stack
            robot.driveStraightOuttake(-0.2, 1.5);
            robot.drivetrain.driveToPose(32, 12, -90, 2);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_DOWN_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_DOWN_RIGHT);
            sleep(200);
            //Intake Pixels
            //robot.intake.outtake(-0.7, 3);
            //robot.drivetrain.driveStraightTime(-0.2, 2);
            //Stage Door

            robot.driveToAprilTag(2, 5, 3);
            robot.drivetrain.stopMotors();
            runtime.reset();

            while(opModeIsActive()&& runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(400);
            }
            robot.lift.liftLeft.setPower(0.1);
            robot.lift.liftRight.setPower(0.1);
            sleep(200);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);
            sleep(200);
            robot.driveStraightTime(-0.2, 1.25);
            robot.driveStraightStrafe(-0.3, 1);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_OUT);
            sleep(200);
            robot.driveStraightTime(-0.5, .25);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
            sleep(200);
            runtime.reset();
            robot.drivetrain.driveStraightTime(0.2,1);
            /*
            while(runtime.seconds() < 1){
                robot.drivetrain.driveFrontLeft.setPower(0.2);
                robot.drivetrain.driveFrontRight.setPower(0.2);
                robot.drivetrain.driveBackLeft.setPower(0.2);
                robot.drivetrain.driveBackRight.setPower(0.2);
            }
            robot.drivetrain.stopMotors();
            */
            sleep(200);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_IN);
            sleep(200);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);
            sleep(200);
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(0);
                robot.lift.liftToPositionPIDClass(0);
            }
            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            robot.driveStraightStrafe(-0.7, 1);
            robot.driveStraightTime(-0.3, 2);
            robot.drivetrain.stopMotors();
        } else if (robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT){
            //Drive to Spike Mark
            robot.drivetrain.driveToPose(35, 10.5, 180, 2);
            //Outtake and go to pixel stack
            robot.driveStraightOuttake(-0.2, 1.5);
            robot.drivetrain.driveToPose(39, 24, -90, 1);
            robot.drivetrain.driveToPose(25, 24, -90, 1);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_DOWN_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_DOWN_RIGHT);
            sleep(200);
            //Intake Pixels
            //robot.intake.outtake(-0.7, 3);
            //robot.drivetrain.driveStraightTime(-0.2, 2);
            //Stage Door

            robot.driveToAprilTag(1, 5, 3);
            robot.drivetrain.stopMotors();
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(500);
            }
            robot.lift.liftLeft.setPower(0.1);
            robot.lift.liftRight.setPower(0.1);
            sleep(200);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);
            sleep(200);
            robot.driveStraightStrafe(-0.3, .5);
            robot.driveStraightTime(-0.2, 1);
            robot.scoring.boxServo.setPosition(1);
            sleep(200);
            robot.driveStraightTime(-0.7, 1);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
            sleep(200);
            runtime.reset();
            robot.drivetrain.driveStraightTime(0.2,1);
            /*
            while(runtime.seconds() < 1){
                robot.drivetrain.driveFrontLeft.setPower(0.2);
                robot.drivetrain.driveFrontRight.setPower(0.2);
                robot.drivetrain.driveBackLeft.setPower(0.2);
                robot.drivetrain.driveBackRight.setPower(0.2);
            }
            robot.drivetrain.stopMotors();
            */
            sleep(200);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_IN);
            sleep(200);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);
            sleep(200);
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(0);
                robot.lift.liftToPositionPIDClass(0);
            }
            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            robot.driveStraightStrafe(-0.7, 1);
            robot.driveStraightTime(-0.3, 2);
            robot.drivetrain.stopMotors();
        }
        else {
            //Drive to Spike Mark
            robot.drivetrain.driveToPose(30, 0, 0, 1);
            robot.drivetrain.driveToPose(30, 0, -90, 1);
            robot.drivetrain.driveToPose(30, -7, -90, 1);
            //robot.drivetrain.driveToPose(30, 6, -90, 1);
            //Outtake and go to pixel stack
            robot.driveStraightOuttake(-0.2, 1.5);
            //robot.driveStraightStrafe(0.3, 1.5);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_DOWN_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_DOWN_RIGHT);
            sleep(200);
            //Intake Pixels
            //robot.intake.outtake(-0.7, 3);
            //robot.drivetrain.driveStraightTime(-0.2, 2);
            robot.drivetrain.driveToPose(33, 17, -90, 1);

            robot.driveToAprilTag(3, 5, 3);
            robot.drivetrain.stopMotors();
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(500);
            }
            robot.lift.liftLeft.setPower(0.1);
            robot.lift.liftRight.setPower(0.1);
            sleep(200);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);
            sleep(200);
            robot.driveStraightTime(-0.2, 1);
            robot.driveStraightStrafe(-0.3, 1);
            //robot.scoring.boxServo.setPosition(1);
            //sleep(200);
            robot.driveStraightTime(-0.65, 1);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
            sleep(200);
            runtime.reset();
            robot.drivetrain.driveStraightTime(0.2,1);
            /*
            while(runtime.seconds() < 1){
                robot.drivetrain.driveFrontLeft.setPower(0.2);
                robot.drivetrain.driveFrontRight.setPower(0.2);
                robot.drivetrain.driveBackLeft.setPower(0.2);
                robot.drivetrain.driveBackRight.setPower(0.2);
            }
            robot.drivetrain.stopMotors();
            */
            sleep(200);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_IN);
            sleep(200);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);
            sleep(200);
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(0);
                robot.lift.liftToPositionPIDClass(0);
            }
            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            //robot.driveStraightStrafe(0.5, 2);
            // robot.driveStraightTime(-0.7,2);

            robot.driveStraightStrafe(-0.7, 1);
            robot.driveStraightTime(-0.3, 2);

            robot.drivetrain.stopMotors();
        }

        //robot.drivetrain.driveStraightTime(-0.3, 2);
    }

}


