package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Set;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOpExample", group="Linear Opmode")

public class NewTeleBalanceTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    //theoritically, if we were using a robot class (which we use for our autonomous)
    //we wouldn't declare motors here, but rather just use the functions in the robot
    //class when buttons are pressed.
    //see the auto code for how to implement robot class (TankDrive) functions
    
    
    //drivetrain
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    
    //lift
    private DcMotor lift = null;

    
    //intake
    private DcMotor intakeRight = null;
    private DcMotor intakeLeft = null;
    private Servo omni = null;
    
    //foundation
    private Servo hookRight = null;
    private Servo hookLeft = null;
    
    //skystoneGrabber
    private Servo skystoneGrabber = null;
    
    //claw
    private Servo claw = null;
    private Servo pivot = null;
    
    //flicker
    private Servo skytoneGrabber = null;
    
    //capstone
    private Servo capstone = null;
    
    //tape 
    private DcMotor tape = null;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        //lift
        lift = hardwareMap.get(DcMotor.class, "lift");
        
        //intake
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        omni = hardwareMap.get(Servo.class, "omni");
        
        //servo
        //hooks
        hookRight = hardwareMap.get(Servo.class, "hookRight");
        hookLeft = hardwareMap.get(Servo.class, "hookLeft");
        boolean pressed = false;
        
        //skystone grabber
        skystoneGrabber = hardwareMap.get(Servo.class, "skystoneGrabber");
        
        //claw
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        
        //flicker
        skystoneGrabber = hardwareMap.get(Servo.class, "skystoneGrabber");
        
        //capstone
        capstone = hardwareMap.get(Servo.class, "capstone");
        
        //tape 
        tape = hardwareMap.get(DcMotor.class, "tape");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        intakeRight.setDirection(DcMotor.Direction.FORWARD);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        
    
        
        //set servos to start position
        //hookRight.setPosition(.3);
        //hookLeft.setPosition(1);
        hookRight.setPosition(.4);
        hookLeft.setPosition(1);
        //set skystone grabber to start position
        skystoneGrabber.setPosition(1);
        
        //capstone hoplder to start position
        capstone.setPosition(0.0);
        
        
        //claw start position
        boolean clawOpen = true;
        boolean pivotIn = true;
      
      
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
         
    
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            
            // Setup a variable for each drive wheel to save power level for telemetry
            
            //mecanum wheels code stuff
            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;
            
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            
            if(gamepad1.right_bumper) {
                leftFrontPower = Range.clip(drive + turn + strafe, -.3, .3);
                rightFrontPower = Range.clip(drive - turn - strafe, -.3, .3);
                rightBackPower = Range.clip(drive - turn + strafe, -.3, .3);
                leftBackPower = Range.clip(drive + turn - strafe, -.3, .3);
            } else {
                leftFrontPower = Range.clip(drive + turn + strafe, -1, 1);
                rightFrontPower = Range.clip(drive - turn - strafe, -1, 1);
                rightBackPower = Range.clip(drive - turn + strafe, -1, 1);
                leftBackPower = Range.clip(drive + turn - strafe, -1, 1);
            }
            
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);
            
            
            telemetry.addData("leftFrontPower: ", leftFrontPower);
            telemetry.addData("rightFrontPower: ", rightFrontPower);
            telemetry.addData("leftBackPower: ", leftBackPower);
            telemetry.addData("rightBackPower: ", rightBackPower);
            

            
            //lift code
            //we want the lift to go in four inch intervals
            double liftPower = Range.clip(gamepad2.left_stick_y, -0.95, 0.5 );
            telemetry.addData("liftPower: ", liftPower);
            telemetry.addData("liftPosition: ", lift.getCurrentPosition());
            if(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1)
            {
                //checks if the left stick is being pushed
                //then operate the lift on lift power
                lift.setPower(liftPower);
            }
            else if(gamepad2.right_trigger > 0.3)
            {
                //if not pressed, and right trigger is, then run the lift to a certain position
                if(lift.getCurrentPosition() > -250 && !gamepad2.right_bumper)
                {
                    lift.setPower(-.5);
                }
                else
                {
                    lift.setPower(0);
                }
            }
            else if(lift.getCurrentPosition() < -261)
            {
                lift.setPower(-.08);
            }
            else 
            {
                if(lift.getCurrentPosition() < 0)
                {
                    //if not hitting joystick or trigger, make the lift go down
                    lift.setPower(.3);
                }
                else
                {
                    lift.setPower(0);
                }
            }
            //lift.setPower(liftPower);
            /*if (lift.getCurrentPosition() > -2560 || gamepad2.left_stick_y > 0.2) {
               if (lift.getCurrentPosition() < 0 || gamepad2.left_stick_y < -0.2){
                lift.setPower(liftPower);
               } else {
                   lift.setPower(0);
               }
            } /*else if (lift.getCurrentPosition() < 0 || gamepad2.left_stick_y < -0.2){
                lift.setPower(liftPower);
            }*/
            /*else{
                lift.setPower(0);
            }*/
            
            //intake code (x and y buttons)
            double intakePower = 0.9; //gamepad2.right_trigger - gamepad2.left_trigger; //0.9;
            //intakeLeft.setPower(intakePower);
            //intakeRight.setPower(intakePower);
            
            if(gamepad2.right_trigger > 0.3 || gamepad2.right_bumper)
            {
                intakeLeft.setPower(intakePower);
                intakeRight.setPower(intakePower);
                omni.setPosition(1);
            }
            else if(gamepad2.left_trigger > 0.3)
            {
                intakeLeft.setPower(-intakePower);
                intakeRight.setPower(-intakePower);
                omni.setPosition(0);
                
                
            } 
            else 
            {
                intakeLeft.setPower(0.0);
                intakeRight.setPower(0.0);
                omni.setPosition(.5);
                
            }
            
            //if (liftPower < 0.1 && liftPower > -0.1){
                //if(right gamepage trigger<0.3)
            //}
            
            //foundation hook
            /*if(gamepad1.right_bumper)
            {
                hookRight.setPosition(.5);
                hookLeft.setPosition(1);
            }else*/ if(gamepad1.left_bumper && !pressed)
            {
                if (hookRight.getPosition() == 1)
                {
                    hookRight.setPosition(.5);
                    hookLeft.setPosition(1);
                }
                
                else 
                {
                    hookRight.setPosition(1);
                    hookLeft.setPosition(.3);
                }
                pressed = true;
                    
            }
            
            if (!gamepad1.left_bumper)
                pressed = false;
            
            //tape 
            double tapePower = gamepad1.right_trigger -gamepad1.left_trigger;
            tape.setPower(tapePower);
            if(tapePower < 0){
                tape.setPower(.5*tapePower);
            }
            
            //claw code
            //two servos cause we fancy
            double clawSpeed = 0.5;
            double pivotSpeed = 0.5;
            
            
            if(gamepad2.a) {
                claw.setPosition(.3);
            } else if (gamepad2.y) {
                claw.setPosition(1);
            }
            
            if(gamepad2.b) {
                pivot.setPosition(0);
            } else if (gamepad2.x) {
                pivot.setPosition(0.76);
            }
            
            //capstone code
            if(gamepad2.left_bumper) {
                capstone.setPosition(1);
                
            } else {
                capstone.setPosition(0.05);
            }
            
            
            
        
            telemetry.addData("hookRight: ", hookRight.getPosition());
            telemetry.addData("pressed: ", pressed);
            telemetry.update();
            
        }
        
        


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
        }
        
    }
