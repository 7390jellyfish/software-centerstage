package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CloseBlueVision extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat rMat = new Mat();

    public static int position = 1;
    public static int getPosition() {
        return position;
    }

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 350),
            new Point(375, 575));

    static final Rect MIDDLE_ROI = new Rect(
            new Point(625, 300),
            new Point(975, 550));

    public CloseBlueVision(Telemetry t) {
        telemetry = t;
    }
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // blue
        Scalar rLow = new Scalar(58, 68, 0);
        Scalar rHigh = new Scalar(140, 255, 178);
//        Scalar rLow = new Scalar(100, 100, 0);
//        Scalar rHigh = new Scalar(128, 255, 120);

        Core.inRange(mat, rLow, rHigh, rMat);

        Mat rLeft = rMat.submat(LEFT_ROI);
        Mat rMiddle = rMat.submat(MIDDLE_ROI);

        Scalar color = new Scalar(100, 100, 100);

        // left
        Point point2a = new Point(0, 350);
        Point point2b = new Point(375, 575);
        Imgproc.rectangle(mat, point2a, point2b, color, 5);

        // middle
        Point point3a = new Point(625, 300);
        Point point3b = new Point(975, 550);
        Imgproc.rectangle(mat, point3a, point3b, color, 5);

        double yLeftValue = Core.sumElems(rLeft).val[0] / LEFT_ROI.area() / 255;
        double yMiddleValue = Core.sumElems(rMiddle).val[0] / MIDDLE_ROI.area() / 255;

        if ((Math.round(yLeftValue * 100) > Math.round(yMiddleValue * 100)) && (Math.round(yLeftValue * 100) > 20)) {
            position = 1;
        } else if (Math.round(yMiddleValue * 100) > 20) {
            position = 2;
        } else {
            position = 3;
        }

        telemetry.addLine("ROBOT IS READY");
        telemetry.addData("spike mark", position);
        telemetry.addData("blue left percentage", Math.round(yLeftValue * 100));
        telemetry.addData("blue middle percentage", Math.round(yMiddleValue * 100));
        telemetry.update();

        return mat;
    }
}