package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Extension {
    private LinearOpMode myOpMode = null;

    public Servo leftLink = null;
    public Servo rightLink = null;

    public static final double LEFT_LINK_IN = 0;
    public static final double RIGHT_LINK_IN = 0;
    public static final double LEFT_LINK_OUT = 0.4;
    public static final double RIGHT_LINK_OUT = 0.4;

    public double leftLinkPosition;
    public double rightLinkPosition;

    public Extension(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        leftLink = myOpMode.hardwareMap.get(Servo.class, "leftLink");
        rightLink = myOpMode.hardwareMap.get(Servo.class, "rightLink");

        leftLink.setPosition(LEFT_LINK_IN);
        rightLink.setPosition(RIGHT_LINK_IN);

        leftLinkPosition = LEFT_LINK_IN;
        rightLinkPosition = RIGHT_LINK_IN;
    }

    public void teleOp() {
        leftLink.setPosition(leftLinkPosition);
        rightLink.setPosition(rightLinkPosition);
        if(myOpMode.gamepad2.right_bumper){
            leftLinkPosition = LEFT_LINK_IN;
            rightLinkPosition = RIGHT_LINK_IN;
        } else if (myOpMode.gamepad2.right_bumper){
            leftLinkPosition = LEFT_LINK_OUT;
            rightLinkPosition = RIGHT_LINK_OUT;
        }

    }

}
