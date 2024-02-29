package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (name="BlueRightAuto", group = "Concept")
public class BlueRightAuto extends LinearOpMode {
    private Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        robot.drivetrain.localizer.setCoordinates(-12, 66, (Math.PI/2)*3);

        while (!isStarted()) {
            robot.camera.scanAprilTag(5);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        robot.camera.visionPortalFront.stopStreaming();
        // robot.camera.stopColorProcessor();

        waitForStart();

        if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE){
            robot.drivetrain.driveStraightPID(37, 3);
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 3){
                //robot.intake.outtake(0.8);
            }
            robot.intake.intakeLeft.setPower(0);
            robot.intake.intakeRight.setPower(0);
            //robot.drivetrain.driveStraightTime(0.1, .75);
            robot.drivetrain.encoderTurn(-3400, 3);
            robot.drivetrain.driveStraightTime(0.4, 2);
            robot.driveToAprilTag(2, 4);
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 2) {
                robot.lift.liftToPositionPIDClass(500);
                robot.lift.liftToPositionPIDClass(500);
                robot.drivetrain.driveFrontLeft.setPower(-0.2);
                robot.drivetrain.driveFrontRight.setPower(-0.2);
                robot.drivetrain.driveBackLeft.setPower(-0.2);
                robot.drivetrain.driveBackRight.setPower(-0.2);
                robot.drivetrain.localizer.update();
                robot.drivetrain.localizer.updateDashboard();
            }

            robot.lift.liftLeft.setPower(0.15);
            robot.lift.liftRight.setPower(0.15);
            robot.drivetrain.stopMotors();

            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);
            robot.scoring.rightArmServo.setPosition(robot.scoring.ARM_UP_RIGHT);
            sleep(500);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_OUT);
            sleep(500);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
            sleep(500);
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
            sleep(500);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_IN);
            sleep(500);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);
            robot.scoring.rightArmServo.setPosition(robot.scoring.ARM_DOWN_RIGHT);
            sleep(500);
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 2) {
                robot.lift.liftToPositionPIDClass(0);
                robot.lift.liftToPositionPIDClass(0);
            }
            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            robot.drivetrain.driveSidePID(35,5);

            robot.drivetrain.stopMotors();
            //robot.lift.liftToPositionPIDClass(100);
            //robot.scoring.(100);

            // robot.l

        } else if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT) {
            robot.drivetrain.driveStraightPID(35, 3);
            robot.drivetrain.driveSidePID(15, 2);
            runtime.reset();
            while(runtime.seconds() < 1.5){
                robot.intake.outtake(0.8);
            }
            runtime.reset();
            // robot.drivetrain.driveStraightTime(0.2,1);
            /*
            while(runtime.seconds() < 2){
                robot.drivetrain.driveFrontLeft.setPower(0.2);
                robot.drivetrain.driveFrontRight.setPower(0.2);
                robot.drivetrain.driveBackLeft.setPower(0.2);
                robot.drivetrain.driveBackRight.setPower(0.2);
            }
            robot.drivetrain.stopMotors();
            */
            robot.intake.intakeLeft.setPower(0);
            robot.intake.intakeRight.setPower(0);
            robot.drivetrain.driveStraightTime(0.14, 2);
            robot.drivetrain.driveSidePID(15, 2);
            robot.drivetrain.encoderTurn(-3400, 3);
            robot.drivetrain.driveStraightTime(0.4, 2.5);
            robot.drivetrain.driveSidePID(-10, 2);
            robot.driveToAprilTag(1, 3);
            runtime.reset();
            while(opModeIsActive()&&runtime.seconds() < 2) {
                robot.lift.liftToPositionPIDClass(500);
                robot.lift.liftToPositionPIDClass(500);
                robot.drivetrain.driveFrontLeft.setPower(-0.2);
                robot.drivetrain.driveFrontRight.setPower(-0.2);
                robot.drivetrain.driveBackLeft.setPower(-0.2);
                robot.drivetrain.driveBackRight.setPower(-0.2);
                robot.drivetrain.localizer.update();
                robot.drivetrain.localizer.updateDashboard();
            }

            robot.lift.liftLeft.setPower(0.15);
            robot.lift.liftRight.setPower(0.15);
            robot.drivetrain.stopMotors();

            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);
            robot.scoring.rightArmServo.setPosition(robot.scoring.ARM_UP_RIGHT);
            sleep(500);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_OUT);
            sleep(500);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
            sleep(500);
            runtime.reset();
            robot.drivetrain.driveStraightTime(-0.2,1);
            /*
            while(runtime.seconds() < 2){
                robot.drivetrain.driveFrontLeft.setPower(0.2);
                robot.drivetrain.driveFrontRight.setPower(0.2);
                robot.drivetrain.driveBackLeft.setPower(0.2);
                robot.drivetrain.driveBackRight.setPower(0.2);
            }
             robot.drivetrain.stopMotors();
             */

            sleep(500);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_IN);
            sleep(500);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);
            robot.scoring.rightArmServo.setPosition(robot.scoring.ARM_DOWN_RIGHT);
            sleep(500);
            runtime.reset();
            while(runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(0);
                robot.lift.liftToPositionPIDClass(0);
            }
            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            robot.drivetrain.driveSidePID(35,3);
            robot.drivetrain.driveStraightTime(0.2,3);

            robot.drivetrain.stopMotors();

        } else {
            robot.drivetrain.driveStraightPID(35, 3);
            robot.drivetrain.driveSidePID(-13, 2);
            runtime.reset();
            while(runtime.seconds() < 1.5){
                robot.intake.outtake(0.8);
            }
            runtime.reset();
            robot.drivetrain.driveStraightTime(0.2,1);
            /*
            while(runtime.seconds() < 2){
                robot.drivetrain.driveFrontLeft.setPower(0.2);
                robot.drivetrain.driveFrontRight.setPower(0.2);
                robot.drivetrain.driveBackLeft.setPower(0.2);
                robot.drivetrain.driveBackRight.setPower(0.2);
            }
            robot.drivetrain.stopMotors();
            */
            robot.intake.intakeLeft.setPower(0);
            robot.intake.intakeRight.setPower(0);
            robot.drivetrain.driveStraightTime(0.11, 1);
            robot.drivetrain.driveSidePID(15, 2);
            robot.drivetrain.encoderTurn(-3400, 3);
            robot.drivetrain.driveStraightTime(0.4, 3);
            robot.driveToAprilTag(3, 5);
            //robot.drivetrain.driveSidePID(5,1);
            runtime.reset();
            while(opModeIsActive()&&runtime.seconds() < 2) {
                robot.lift.liftToPositionPIDClass(500);
                robot.lift.liftToPositionPIDClass(500);
                robot.drivetrain.driveFrontLeft.setPower(-0.2);
                robot.drivetrain.driveFrontRight.setPower(-0.2);
                robot.drivetrain.driveBackLeft.setPower(-0.2);
                robot.drivetrain.driveBackRight.setPower(-0.2);
                robot.drivetrain.localizer.update();
                robot.drivetrain.localizer.updateDashboard();
            }

            robot.lift.liftLeft.setPower(0.15);
            robot.lift.liftRight.setPower(0.15);
            robot.drivetrain.stopMotors();

            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);
            robot.scoring.rightArmServo.setPosition(robot.scoring.ARM_UP_RIGHT);
            sleep(500);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_OUT);
            sleep(500);
            robot.scoring.leftGateServo.setPosition(robot.scoring.GATE_UP_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.GATE_UP_RIGHT);
            sleep(500);
            runtime.reset();
            robot.drivetrain.driveStraightTime(-0.2,1);
            /*
            while(runtime.seconds() < 2){
                robot.drivetrain.driveFrontLeft.setPower(0.2);
                robot.drivetrain.driveFrontRight.setPower(0.2);
                robot.drivetrain.driveBackLeft.setPower(0.2);
                robot.drivetrain.driveBackRight.setPower(0.2);
            }
             robot.drivetrain.stopMotors();
             */

            sleep(500);
            robot.scoring.boxServo.setPosition(robot.scoring.BOX_IN);
            sleep(500);
            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_DOWN_LEFT);
            robot.scoring.rightArmServo.setPosition(robot.scoring.ARM_DOWN_RIGHT);
            sleep(500);
            runtime.reset();
            while(runtime.seconds() < 1) {
                robot.lift.liftToPositionPIDClass(0);
                robot.lift.liftToPositionPIDClass(0);
            }
            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            robot.drivetrain.driveSidePID(35,3);
            robot.drivetrain.driveStraightTime(0.2,3);

            robot.drivetrain.stopMotors();
        }

        //try drive to pose
        //try drive to april tag
    }

}


