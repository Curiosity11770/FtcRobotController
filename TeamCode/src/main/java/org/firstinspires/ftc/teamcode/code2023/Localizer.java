package org.firstinspires.ftc.teamcode.code2023;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Localizer {

    //Declare and assign constants related to ODOMETRY pod hardware
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    // in; distance between the left and right wheels
    public static double LATERAL_DISTANCE = 4.671152406720355; //4.98;

    // in; offset of the lateral wheel from left and right wheels. '-' is behind, '+' is in front
    public static double FORWARD_OFFSET = 0;

    //These multipliers can be used to tune any discrepency between the left and right wheel
    public static double X_MULTIPLIER = 1.0;
    public static double Y_MULTIPLIER = 1.0;

    private Encoder leftEncoder, rightEncoder, centerEncoder;

    Localizer()

}
