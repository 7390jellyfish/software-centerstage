package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FarVisionBlue extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat rMat = new Mat();

    public static int position = 1;
    public static int getPosition() {
        return position;
    }

    static final Rect MIDDLE_ROI = new Rect(
            new Point(400, 225),
            new Point(700, 500));

    static final Rect RIGHT_ROI = new Rect(
            new Point(1000, 275),
            new Point(1280, 575));

    public FarVisionBlue(Telemetry t) {
        telemetry = t;
    }
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // blue
        Scalar rLow = new Scalar(100, 120, 50);
        Scalar rHigh = new Scalar(125, 225, 110);

        Core.inRange(mat, rLow, rHigh, rMat);

        Mat rMiddle = rMat.submat(MIDDLE_ROI);
        Mat rRight = rMat.submat(RIGHT_ROI);

        Scalar color = new Scalar(100, 100, 100);

        // middle
        Point point2a = new Point(400, 275);
        Point point2b = new Point(700, 600);
        Imgproc.rectangle(mat, point2a, point2b, color, 5);

        // right
        Point point3a = new Point(1000, 275);
        Point point3b = new Point(1280, 600);
        Imgproc.rectangle(mat, point3a, point3b, color, 5);

        double yMiddleValue = Core.sumElems(rMiddle).val[0] / MIDDLE_ROI.area() / 255;
        double yRightValue = Core.sumElems(rRight).val[0] / RIGHT_ROI.area() / 255;

        if ((Math.round(yMiddleValue * 100) > Math.round(yRightValue * 100)) && (Math.round(yMiddleValue * 100) > 5)) {
            position = 2;
        } else if (Math.round(yRightValue * 100) > 5) {
            position = 3;
        } else {
            position = 1;
        }

        telemetry.addLine("ROBOT IS READY");
        telemetry.addData("spike mark", position);
        telemetry.addData("red middle percentage", Math.round(yMiddleValue * 100));
        telemetry.addData("red right percentage", Math.round(yRightValue * 100));
        telemetry.update();

        return mat;
    }
}