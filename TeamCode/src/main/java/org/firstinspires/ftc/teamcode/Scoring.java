package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {

    private LinearOpMode myOpMode = null;
    public CRServo spinIntake = null;
    public Servo flipIntake = null;

    public static final double SPIN_SPEED = 0.7;
    public static final double INTAKE_CLOSED = 0.7;
    public static final double INTAKE_OPEN = 0;

    public double flipPosition;
    public Scoring(LinearOpMode opmode) {
        myOpMode = opmode;
    }


}
