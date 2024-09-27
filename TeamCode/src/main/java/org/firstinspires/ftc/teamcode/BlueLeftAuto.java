package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (name="BlueLeftAuto", group = "Concept")
public class BlueLeftAuto extends LinearOpMode {
    private Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        robot.drivetrain.localizer.setCoordinates(12, -66, Math.PI/2*3);

        while (!isStarted()) {
            robot.camera.scanAprilTag(5);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        robot.camera.visionPortalFront.stopStreaming();
        // robot.camera.stopColorProcessor();

        waitForStart();

        if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE){
            robot.drivetrain.driveStraightPID(35, 3);
            runtime.reset();
            while(opModeIsActive()&& runtime.seconds() < 3){
                //robot.intake.outtake(0.8);
            }
            robot.intake.intakeLeft.setPower(0);
            robot.intake.intakeRight.setPower(0);
            robot.drivetrain.encoderTurn(-3400, 3);
            robot.driveToAprilTag(2, 4.5, 0);
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

            robot.drivetrain.driveSidePID(-35,5);

            robot.drivetrain.stopMotors();
            //robot.lift.liftToPositionPIDClass(100);
            //robot.scoring.(100);

            // robot.l

        } else if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT) {
            robot.drivetrain.driveStraightPID(35, 3);
            robot.drivetrain.driveSidePID(15, 3);
            runtime.reset();

            runtime.reset();
            while(runtime.seconds() < 3){
                //robot.intake.outtake(0.8);
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
            robot.drivetrain.encoderTurn(3400, 3);
            robot.driveToAprilTag(1, 6, 0);
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
            robot.drivetrain.driveStraightTime(-0.2,2);
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
            while(runtime.seconds() < 2) {
                robot.lift.liftToPositionPIDClass(0);
                robot.lift.liftToPositionPIDClass(0);
            }
            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            robot.drivetrain.driveSidePID(-100,5);

            robot.drivetrain.stopMotors();

        } else {

        }

        //try drive to pose
        //try drive to april tag
    }

}
