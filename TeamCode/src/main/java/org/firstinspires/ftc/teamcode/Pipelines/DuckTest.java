package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DuckTest extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat yMat = new Mat();

    static final Rect LEFT_ROI = new Rect(
            new Point(50, 100),
            new Point(107, 160));

    static final Rect MIDDLE_ROI = new Rect(
            new Point(125, 100),
            new Point(190, 160));

    static final Rect RIGHT_ROI = new Rect(
            new Point(220, 100),
            new Point(275, 160));

    public DuckTest(Telemetry t){telemetry=t;}
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // yellow
        Scalar yLow = new Scalar(20,100,100);
        Scalar yHigh = new Scalar(30,255,255);

        Core.inRange(mat, yLow, yHigh, yMat);

        Mat yLeft = yMat.submat(LEFT_ROI);
        Mat yMiddle = yMat.submat(MIDDLE_ROI);
        Mat yRight = yMat.submat(RIGHT_ROI);

        Scalar color = new Scalar(100, 100, 100);

        Point point1a = new Point(50, 100);
        Point point1b = new Point(107, 160);
        Imgproc.rectangle(mat, point1a, point1b, color, 2);

        Point point2a = new Point(125, 100);
        Point point2b = new Point(190, 160);
        Imgproc.rectangle(mat, point2a, point2b, color, 2);

        Point point3a = new Point(220, 100);
        Point point3b = new Point(275, 160);
        Imgproc.rectangle(mat, point3a, point3b, color, 2);

        double yLeftValue = Core.sumElems(yLeft).val[0] / LEFT_ROI.area() / 255;
        double yMiddleValue = Core.sumElems(yMiddle).val[0] / MIDDLE_ROI.area() / 255;
        double yRightValue = Core.sumElems(yRight).val[0] / RIGHT_ROI.area() / 255;

        double maxValue = Math.max(yLeftValue, Math.max(yMiddleValue, yRightValue));
        if (maxValue == yLeftValue) {
            telemetry.addData("result", "left");
        } else if (maxValue == yMiddleValue) {
            telemetry.addData("result", "middle");
        } else {
            telemetry.addData("result", "right");
        }

        telemetry.addData("Yellow left percentage", Math.round(yLeftValue * 100) + "%");
        telemetry.addData("Yellow middle percentage", Math.round(yMiddleValue * 100) + "%");
        telemetry.addData("Yellow right percentage", Math.round(yRightValue * 100) + "%");
        telemetry.update();

        return mat;
    }
}