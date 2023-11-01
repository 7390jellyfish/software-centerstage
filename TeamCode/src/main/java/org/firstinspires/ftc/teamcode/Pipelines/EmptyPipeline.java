package org.firstinspires.ftc.teamcode.Pipelines;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;

import java.io.StringWriter;

public class EmptyPipeline extends OpenCvPipeline {
    Mat gmat = new Mat();
    Mat bmat = new Mat();
    Mat rmat = new Mat();
    Telemetry telemetry;
    public EmptyPipeline(Telemetry t) { telemetry = t; }


    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Left
        Point point1 = new Point(60, 160);
        Point point2 = new Point(100, 110);
        Scalar color = new Scalar(64, 64, 64);
        Imgproc.rectangle(mat, point1, point2, color, 2);
//        middle
        Point point3 = new Point(130, 110);
        Point point4 = new Point(180, 160);
        Imgproc.rectangle(mat, point3, point4, color, 2);
//        right
        Point point5 = new Point(225, 110);
        Point point6 = new Point(270, 150);
        Imgproc.rectangle(mat, point5, point6, color, 2);
        Scalar highHSV = new Scalar(30,255,255);
        Scalar lowHSV = new Scalar(20,200,100);
        try {

            Core.inRange(mat, lowHSV, highHSV, rmat);
            Mat left = rmat.submat(new Rect(point1,point2));
            Mat middle = rmat.submat(new Rect(point3,point4));
            Mat right = rmat.submat(new Rect(point5,point6));


            double lefte = Core.sumElems(left).val[0] / new Rect(point1,point2).area() / 255;
            double middlee = Core.sumElems(middle).val[0] / new Rect(point3,point4).area() / 255;
            double righte = Core.sumElems(right).val[0] / new Rect(point5,point6).area() / 255;


//            left.release();
//            middle.release();
//            right.release();

            telemetry.addData("Left", Math.round(lefte * 100) + "%");
            telemetry.addData("Middle", Math.round(middlee * 100) + "%");
            telemetry.addData("Right", Math.round(righte * 100) + "%");
            if(lefte>righte&&lefte>middlee){
                telemetry.addData("Winner", "left");
            } else if (righte>lefte&&righte>middlee) {
                telemetry.addData("Winner", "right");
            }else{
                telemetry.addData("Winner", "middle");
            }
            telemetry.update();
            return mat  ;
        } catch (Exception e) {

            telemetry.addData("error", Arrays.toString(e.getStackTrace()));
            telemetry.update();
            e.printStackTrace();
            return mat;
        }


    }


}