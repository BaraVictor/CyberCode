package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedTeamProp extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND,
        MID
    }
    private Location location;
    static final Rect LEFT_ROI = new Rect(
            new Point(0,150),
            new Point(50,200));
    static final Rect RIGHT_ROI = new Rect(
            new Point(640, 200),
            new Point(580, 310));
    static final Rect MID_ROI = new Rect(
            new Point(255, 185),
            new Point(355, 285));
    static double PERCENT_COLOR_THRESHOLD = 0.20;

    public RedTeamProp(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);
        Scalar lowHSV = new Scalar(110, 25, 25);
        Scalar highHSV = new Scalar(130, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat mid = mat.submat(MID_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;

        left.release();
        right.release();
        mid.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Mid raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMid = midValue >PERCENT_COLOR_THRESHOLD;


        if (stoneRight) {
            location = Location.RIGHT;
            //telemetry.addData("TeamProp Location", "right");
        }
        else if(stoneLeft){
            location = Location.LEFT;
            //telemetry.addData("TeamProp Location", "left");
        }
        else if(stoneMid){
            location = Location.MID;
            //telemetry.addData("TeamProp Location","mid")
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MID? colorSkystone:colorStone);
        return mat;
    }

    public Location getLocation() {
        return location;
    }
}