package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

//Features and improvements of Path class
    //1) moved follow path function from tankDrive to Path, allows easy resetting of timer for motion profile.
    //2) added states so that robot will turn first, then drive straight, then stop
    //3) added boolean and constants so path can be driven forwards or reverse (by flipping heading and direction)
    //4) added a boolean so you  can check if the robot has reached its target (used in opMode)
    //5) added a condition so that if the robot overshoots target, it should stop

//Need to Test (in somewhat order they should be checked)

    //1) transition from turning to driving straight
    //2) transition from driving straight to stopping
    //3) running a path in reverse
    //4) running two paths in sequence
    //5) IF ALL THE ABOVE WORKS...then you can tune constants for PID controllers
    //6) Once function works and constants are tuned so robot drives predictably...you should be good to make auto.
//note - we use tank drive
/* 
Our robot uses an algorithm that is able to take in a max velocity and acceleration to calculate where the robot should be 
between its starting position and its end position (a coordinate on the field) over time. This calculates ‘steps’ for the 
robot to follow, and the error between the robot’s position and this target. The error is then passed to the PID Controller 
to calculate motor output. Our motion profile has multiple states for the turn to the target, drive in a straight line to 
that target (which is necessary as the junctions on the field require more precise movement), and stop. The robot is able 
to correct its course due to its localization
*/

public class Path {
    public double max_acceleration;
    public double max_velocity;

    public double entire_dt;
    public double targetX;
    public double targetY;
    public double theta;
    public double totalDistance;
    public double currentDistance;
    public double currentHeading;

    public double headingError;

    public double f;
    public double t;

    public boolean forward;
    public boolean targetReached;
    public double headingOffset;
    public double directionMultiplier;
    public String state;
    public ElapsedTime time;

    SampleTankDrive robot;

    public Path(SampleTankDrive r, double tX, double tY, boolean tF) {
        //bring in robot
        robot = r;

        //default first action to turning
        state = "TURN_TO_TARGET";
        //timer will be used for the motion profile. Only reset after robot is done turning
        time = new ElapsedTime();
        targetReached = false;

        max_acceleration = robot.maxAcceleration;
        max_velocity = robot.maxVelocity;

        targetX = tX;
        targetY = tY;

        //if the path is set to go in reverse, alter the offset and multiplier to flip drive direction and heading
        forward = tF;
        if(forward){
            headingOffset = 0;
            directionMultiplier = 1;
        }else {
            headingOffset = Math.PI;
            directionMultiplier = -1;
        }

        //calculate initial distance and angle to target
        double xError = targetX - robot.getPoseEstimate().getX();
        double yError = targetY - robot.getPoseEstimate().getY();
        //theta stores angle from robot to the target
        theta = Math.atan2(yError, xError)+ headingOffset;
        // distance stores the initial distance from robot to the target
        totalDistance= Math.hypot(xError, yError);
        //use distance to calculate motion profile
    }


    public void followPath() {
        //update localization of robot
        robot.update();

        //update theta and distance to target
        double xError = targetX - robot.getPoseEstimate().getX();
        double yError = targetY - robot.getPoseEstimate().getY();
        //confirm theta is in radians and runs positive and negative as expected (it needs to be in same format as robot heading)
        // Note : heading offset added to account for direction robot is traveling
        theta = Math.atan2(yError, xError) + headingOffset; //radians
        currentDistance = Math.hypot(xError, yError);
        //get the current heading of the robot, wrapped and converted to radians
        // Note: *check angle units of getPoseEstimate.getHeading
        // Reminder: theta must be in same format as currentHeading

        //currentHeading = robot.angleWrap(robot.getPoseEstimate().getHeading());   //radians
        //**according to ctrl alt ftc, 'normalizing' of angle should happen AFTER error is calculated
        //**try storing currentHeading without wrapping
        currentHeading = robot.getPoseEstimate().getHeading();

        //get the error between robot heading and angle to target
        //**Now call angleWrap on the error
        headingError = robot.angleWrap(theta-currentHeading);     //radians

        if(state == "TURN_TO_TARGET"){
            //calculate an output to only turn the robot
            f = 0;
            t = robot.headingPID.calculate(headingError);
            //if heading error is less than threshold (true when robot is pointed at target)
            if(Math.abs(headingError) < Math.toRadians(4) || time.seconds() > 1.3){
                state = "DRIVE_TO_TARGET";
                time.reset();
                totalDistance = currentDistance;
            }
        } else if(state == "DRIVE_TO_TARGET"){
            //calculate the outputs to drive and maintain heading
            //note that the 'reference' for f should be approaching 0
            f = robot.drivePID.calculate(-totalDistance + calculate(time.seconds()), -currentDistance);
            //t = robot.headingPID.calculate(headingError);
            // prevents t from taking over motor power as f goes to zero
            if(Math.abs(currentDistance) > 6.0){
                t = robot.headingPID.calculate(headingError);
            } else {
                t = 0;
            }
            //if distance is less than threshold (once robot reaches target)
            if(Math.abs(currentDistance) < 0.5){
                state = "STOP";
            }
            //handle overshoot...if angle to target changes by large margin, stop the robot
            if(Math.abs(headingError) > Math.PI/2){
                state = "STOP";
            }
        }else if(state == "STOP"){
            //make the outputs zero to stop the robot
            f = 0;
            t = 0;
            targetReached = true;
        }

        //assign motor powers
        double left_power;
        double right_power;
        //handle robot's direction by multiplying by direction multiplier (-1 if path reversed)
        left_power = (f*directionMultiplier) - t;
        right_power = (f*directionMultiplier) + t;
        robot.leftFront.setPower(left_power);
        robot.leftRear.setPower(left_power);
        robot.rightFront.setPower(right_power);
        robot.rightRear.setPower(right_power);
    }

    public void followPathSteady() {
        //update localization of robot
        robot.update();

        //update distance to target
        double xError = targetX - robot.getPoseEstimate().getX();
        double yError = targetY - robot.getPoseEstimate().getY();
        //confirm theta is in radians and runs positive and negative as expected (it needs to be in same format as robot heading)
        // Note : heading offset added to account for direction robot is traveling
        //theta = Math.atan2(yError, xError) + headingOffset; //radians
        currentDistance = Math.hypot(xError, yError);
        //get the current heading of the robot, wrapped and converted to radians
        // Note: *check angle units of getPoseEstimate.getHeading
        // Reminder: theta must be in same format as currentHeading

        //currentHeading = robot.angleWrap(robot.getPoseEstimate().getHeading());   //radians
        //**according to ctrl alt ftc, 'normalizing' of angle should happen AFTER error is calculated
        //**try storing currentHeading without wrapping
        currentHeading = robot.getPoseEstimate().getHeading();

        //get the error between robot heading and angle to target
        //**Now call angleWrap on the error
        headingError = robot.angleWrap(theta-currentHeading);     //radians

        if(state == "TURN_TO_TARGET"){
            //calculate an output to only turn the robot
            f = 0;
            t = robot.headingPID.calculate(headingError);
            //if heading error is less than threshold (true when robot is pointed at target)
            if(Math.abs(headingError) < Math.toRadians(3)){
                state = "DRIVE_TO_TARGET";
                time.reset();
                totalDistance = currentDistance;
            }
        }else if(state == "DRIVE_TO_TARGET"){
            //calculate the outputs to drive and maintain heading
            //note that the 'reference' for f should be approaching 0
            f = robot.drivePID.calculate(-totalDistance + calculate(time.seconds()), -currentDistance);
            t = robot.headingPID.calculate(headingError);

            //if distance is less than threshold (once robot reaches target)
            if(Math.abs(currentDistance) < 0.5){
                state = "STOP";
            }
            double liveTheta = Math.atan2(yError, xError) + headingOffset;
            double liveHeadingError = robot.angleWrap(liveTheta-currentHeading);
            //handle overshoot...if angle to target changes by large margin, stop the robot
            if(Math.abs(liveHeadingError) > Math.PI/2){
                state = "STOP";
            }
        }else if(state == "STOP"){
            //make the outputs zero to stop the robot
            f = 0;
            t = 0;
            targetReached = true;
        }

        //assign motor powers
        double left_power;
        double right_power;
        //handle robot's direction by multiplying by direction multiplier (-1 if path reversed)
        left_power = (f*directionMultiplier) - t;
        right_power = (f*directionMultiplier) + t;
        robot.leftFront.setPower(left_power);
        robot.leftRear.setPower(left_power);
        robot.rightFront.setPower(right_power);
        robot.rightRear.setPower(right_power);
    }

    public double calculate(double current_dt) {
        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        if (totalDistance < 0) {
            totalDistance*=-1;
            //direction = -1;
        }

        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = totalDistance / 2;
        double acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = totalDistance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        if (current_dt > entire_dt) {
            return totalDistance;
        }

        // if we're accelerating
        if (current_dt < acceleration_dt)
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * (current_dt * current_dt);

            // if we're cruising
        else if (current_dt < deacceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);
            double cruise_current_dt = current_dt - acceleration_dt;

            // use the kinematic equation for constant velocity
            return (acceleration_distance + max_velocity * cruise_current_dt);
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);
            cruise_distance = max_velocity * cruise_dt;
            deacceleration_time = current_dt - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return (acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * (deacceleration_time * deacceleration_time));
        }
    }

    public double totalTime() {
        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        if (totalDistance < 0) {
            totalDistance*=-1;
            //direction = -1;
        }

        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = totalDistance / 2;
        double acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = totalDistance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        return entire_dt;
    }

}
