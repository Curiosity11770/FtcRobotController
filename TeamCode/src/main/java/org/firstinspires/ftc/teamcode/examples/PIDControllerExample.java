package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class PIDControllerExample {
//PID Control Class

    /*

     * Proportional Integral Derivative Controller

     */

    double Kp, Ki, Kd;
    double lastError = 0;
    double lastReference = 0;
    double integralSum = 0;
    double integralCap = 0.25;
    public static double maxOut = 0.95;    //test and change this (if necessary)
    double errorMargin = 1;     //could make smaller (this currently in encoder ticks/degrees)
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;
    double a = 0.5;

    double error;



    PIDControllerExample(double kpIn, double kiIn, double kdIn){
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
        double errorChange = (error - lastError);

        //Low pass filter, reduces noise in signal
        currentFilterEstimate = (a*previousFilterEstimate)+(1-a)*errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate;

        // sum of all error over time
        integralSum = integralSum + (error);

        //anti-windup caps integralSum
        if(integralSum*Ki > integralCap){
            integralSum = integralCap/Ki;
        }else if(integralSum*Ki < -integralCap){
            integralSum = -integralCap/Ki;
        }

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        out = Range.clip(out, -maxOut, maxOut);

        lastError = error;
        lastReference = reference;

        return out;
    }

    public double calculate(double error){
        // check if new target
        /*if(lastReference != reference){
            reset();
        }*/

        // rate of change of the error
        double errorChange = (error - lastError);

        //Low pass filter, reduces noise in signal
        currentFilterEstimate = (a*previousFilterEstimate)+(1-a)*errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate;

        // sum of all error over time
        integralSum = integralSum + (error);

        //anti-windup caps integralSum
        if(integralSum*Ki > integralCap){
            integralSum = integralCap/Ki;
        }else if(integralSum*Ki < -integralCap){
            integralSum = -integralCap/Ki;
        }

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        out = Range.clip(out, -maxOut, maxOut);

        lastError = error;

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
