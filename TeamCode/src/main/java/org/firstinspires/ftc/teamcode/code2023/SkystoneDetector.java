package org.firstinspires.ftc.teamcode.code2023;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;
    /*
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));

     */
    static final Rect MIDDLE_ROI = new Rect(
            new Point(60, 35),
            new Point(200, 75));

    static double PERCENT_COLOR_THRESHOLD = 0.4; //ie 40%

    public SkystoneDetector(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


        Scalar lowHSVGreen = new Scalar(40, 50, 70);
        Scalar highHSVGreen = new Scalar(80, 255, 255);


        Scalar lowHSVYellow = new Scalar(20, 50, 70);
        Scalar highHSVYellow = new Scalar(37, 255, 255);

        Scalar lowHSVBlue = new Scalar(100, 50, 70);
        Scalar highHSVBlue = new Scalar(130, 255, 255);

        Scalar lowHSVRed = new Scalar(0, 50, 70);
        Scalar highHSVRed = new Scalar(15, 255, 255);

        //hue, saturation, value
        Core.inRange(mat, lowHSVRed, highHSVRed, mat);
        /*
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

         */
        Mat middle = mat.submat(MIDDLE_ROI);
/*
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;


 */
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
/*
        left.release();
        right.release();


 */
        middle.release();

        /*
        telemetry.addData("left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");
        */

        telemetry.addData("HELLO", "GOODBYE");
        /*
        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;


         */
        boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
/*
        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("skystone location", "not found");
        } else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("skystone location", "right");
        } else {
            location = Location.LEFT;
            telemetry.addData("skystone location", "left");
            telemetry.addData("test", "hello");
        }
*/
        if(stoneMiddle){
            location = Location.MIDDLE;
            telemetry.addData("skystone location", "middle");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0); //red
        Scalar colorSkystone = new Scalar(0, 255, 0); //green

        Scalar colorMiddle = new Scalar(255, 255, 0);
/*
        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? colorSkystone : colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT ? colorSkystone : colorStone);


 */
        Imgproc.rectangle(mat, MIDDLE_ROI, colorMiddle);

        return mat;
    }

    public Location getLocation(){
        return location;
    }
}