package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class PIDController {
//PID Control Class

    /*

     * Proportional Integral Derivative Controller

     */

    double Kp, Ki, Kd;
    double lastError = 0;
    double lastReference = 0;
    double integralSum = 0;
    public static double maxOut = 0.95;    //test and change this (if necessary)
    double errorMargin = 1;     //could make smaller (this currently in encoder ticks/degrees)
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;

    double error;



    PIDController(double kpIn, double kiIn, double kdIn){
        Kp = kpIn;
        Ki = kiIn;
        Kd = kdIn;
    }

    public double calculate(double reference, double currentPosition){
        // check if new target
        /*if(lastReference != reference){
            reset();
        }*/

        // calculate the error
        error = reference - currentPosition;

        // rate of change of the error
        double derivative = (error - lastError);

        // sum of all error over time
        integralSum = integralSum + (error);

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        out = Range.clip(out, -maxOut, maxOut);

        lastError = error;
        lastReference = reference;

        return out;
    }

    boolean targetReached(double reference, double currentPosition){
        double error = reference-currentPosition;
        if(Math.abs(error) > errorMargin){
            return false;
        }else{
            return true;
        }
    }

    void reset(){
        integralSum = 0;
        previousFilterEstimate = 0;
        currentFilterEstimate = 0;
    }
}
