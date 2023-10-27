//package org.firstinspires.ftc.teamcode.Pipelines;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//public class EmptyPipeline extends OpenCvPipeline{
//    Mat gmat = new Mat();
//
//
//    @Override
//    public Mat processFrame(Mat input) {
//        Mat mat = new Mat();
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Rect rec = new Rect(
//                new Point(550, 300),
//                new Point(750, 500));
//        gmat = mat.submat(rec);
//        Point point1 = new Point(550, 300);
//        Point point2 = new Point(750, 500);
//        Scalar color = new Scalar(64, 64, 64);
//        Imgproc.rectangle(mat, point1, point2, color, 10);
//        Scalar highHSV = new Scalar(80, 255, 255);
//        Scalar lowHSV = new Scalar(40, 60, 70);
//        Core.inRange(gmat, lowHSV, highHSV, gmat);
//        double gleftValue = Core.sumElems(gmat).val[0] / rec.area() / 255;
////        telemetry.addData("Green Left percentage", Math.round(gleftValue * 100) + "%");
//        return mat  ;
//    }
//
//
//}