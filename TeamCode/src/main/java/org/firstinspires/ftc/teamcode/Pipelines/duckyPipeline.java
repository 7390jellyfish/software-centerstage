package org.firstinspires.ftc.teamcode.Pipelines;
// import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;


public class duckyPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    Mat colormat = new Mat();

    public duckyPipeline(Telemetry tel)
    {
        telemetry=tel;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar bluehighHSV = new Scalar(140, 255, 255);
        Scalar bluelowHSV = new Scalar(100, 130, 0);
        Scalar redhighHSV = new Scalar(2,94,83);
        Scalar redlowHSV = new Scalar(5,97,64);

        Rect RECT_LEFT = new Rect(
                new Point(0, 180),
                new Point(110, 60));
        Rect RECT_MIDDLE = new Rect(
                new Point(140, 120),
                new Point(200, 60));
        Rect RECT_RIGHT = new Rect(
                new Point(210, 155),
                new Point(320, 50));
        Scalar color = new Scalar(64, 64, 64);
        Imgproc.rectangle(mat, RECT_LEFT, color, 2);
        Imgproc.rectangle(mat, RECT_MIDDLE, color, 2);
        Imgproc.rectangle(mat, RECT_RIGHT, color,2);

//         Core.inRange(mat, bluelowHSV, bluehighHSV, colormat);
        Core.inRange(mat, redlowHSV, redhighHSV, colormat);

        Mat left = colormat.submat(RECT_LEFT);
        Mat middle = colormat.submat(RECT_MIDDLE);
        Mat right = colormat.submat(RECT_RIGHT);

        double leftValue = Core.sumElems(left).val[0];
        double middleValue = Core.sumElems(middle).val[0];
        double rightValue = Core.sumElems(right).val[0];

        if (leftValue > middleValue || leftValue > rightValue) {
            telemetry.addData("Duck", "Left");
        }
        if (middleValue > leftValue || middleValue > rightValue) {
            telemetry.addData("Duck", "Middle");
        }
        if (rightValue > middleValue || rightValue > leftValue) {
            telemetry.addData("Duck", "Right");
        }
        telemetry.addData("test", "test");
        telemetry.addData("leftValue", leftValue + "");
        telemetry.addData("middleValue", middleValue + "");
        telemetry.addData("rightValue", rightValue + "");
        telemetry.update();

        return mat;
    }
}