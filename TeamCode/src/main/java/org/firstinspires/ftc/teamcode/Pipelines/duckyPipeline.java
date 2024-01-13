package org.firstinspires.ftc.teamcode.Pipelines;
// import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static org.opencv.core.Core.sumElems;

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
//    Telemetry telemetry;
    Mat rmat = new Mat();
    Mat bmat = new Mat();
    public int zone = 1;

    Mat colormat = new Mat();

//    public duckyPipeline(Telemetry tel)
//    {
//        telemetry=tel;
//    }
public int getPosition() {
    return zone;
}
    public EmptyPipeline.ColorSpace colorSpace = EmptyPipeline.ColorSpace.YCrCb;
    private Mat ycrcbMat       = new Mat();
    enum ColorSpace {

        YCrCb(Imgproc.COLOR_RGB2YCrCb);
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);
        Scalar bluehighHSV = new Scalar(162, 121, 255);
        Scalar bluelowHSV = new Scalar(0, 90.7, 127.5);
        Scalar redhighHSV = new Scalar(255,255,255);
        Scalar redlowHSV = new Scalar(0,191,0);
        Core.inRange(ycrcbMat, redlowHSV, redhighHSV, rmat);
        Core.inRange(ycrcbMat, bluelowHSV, bluehighHSV, bmat);

////
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
        Imgproc.rectangle(rmat, RECT_LEFT, color, 2);
        Imgproc.rectangle(rmat, RECT_MIDDLE, color, 2);
        Imgproc.rectangle(rmat, RECT_RIGHT, color,2);
        Imgproc.rectangle(bmat, RECT_LEFT, color, 2);
        Imgproc.rectangle(bmat, RECT_MIDDLE, color, 2);
        Imgproc.rectangle(bmat, RECT_RIGHT, color,2);
//

//
        Mat rleft = rmat.submat(RECT_LEFT);
        Mat rmiddle = rmat.submat(RECT_MIDDLE);
        Mat rright = rmat.submat(RECT_RIGHT);
        Mat bleft = bmat.submat(RECT_LEFT);
        Mat bmiddle = bmat.submat(RECT_MIDDLE);
        Mat bright = bmat.submat(RECT_RIGHT);
        double rleftValue = Core.sumElems(rleft).val[0] / RECT_LEFT.area() / 255;
        double rmiddleValue = Core.sumElems(rmiddle).val[0] / RECT_MIDDLE.area() / 255;
        double rrightValue = Core.sumElems(rright).val[0] / RECT_RIGHT.area() / 255;

//        System.out.println(rleft);
//
//        telemetry.addData("leftValue", rleftValue*100+ "");
//        telemetry.addData("middleValue", rmiddleValue*100 + "");
//        telemetry.addData("rightValue", rrightValue*100+"");
//        telemetry.update();
        if(rleftValue>rmiddleValue&&rleftValue>rrightValue){
            zone = 0;
        } else if (rrightValue>rmiddleValue&&rrightValue>rleftValue) {
            zone = 2;
        }
        else{
            zone = 1;
        }
//
        return rmat;
    }
}