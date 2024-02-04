package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Pipelines.CloseVisionBlue;

@Autonomous
public class FarRedPathing extends LinearOpMode {
    DcMotor leftLift = null;
    DcMotor rightLift = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo wrist = null;
    Servo claw = null;
    @Override
    public void runOpMode() throws InterruptedException {



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        CloseVisionBlue closeVisionBlue = new CloseVisionBlue(telemetry);
        camera.setPipeline(closeVisionBlue);
        int spikeMarkPosition = 1;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40.5, -64, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        // offset
        TrajectorySequence offset = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .build();

        // left
        TrajectorySequence spikeMarkLeft = drive.trajectorySequenceBuilder(offset.end())
                .strafeLeft(22)
                .back(6)
                .turn(Math.toRadians(90))
                .forward(10)
                .back(10)
                .build();
        TrajectorySequence backdropLeft = drive.trajectorySequenceBuilder(spikeMarkLeft.end())
                .back(10)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-35,-57))
                .back(50)
                .lineToLinearHeading(new Pose2d(46, -27.3, Math.toRadians(180)))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backdropLeft.end())
                .forward(20)
                .strafeRight(17)
                .back(23)
                .build();

        // middle
        TrajectorySequence spikeMarkMiddle = drive.trajectorySequenceBuilder(offset.end())
                .strafeLeft(34)
                .strafeRight(11)
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence backdropMiddle = drive.trajectorySequenceBuilder(spikeMarkMiddle.end())
                .lineToConstantHeading(new Vector2d(-35,-57))
                .turn(Math.toRadians(90))
                .back(50)
                .lineToLinearHeading(new Pose2d(46, -27.3, Math.toRadians(180)))
                .build();
        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backdropMiddle.end())
                .forward(10)
                .strafeLeft(28)
                .back(10)
                .build();

        // right
        TrajectorySequence spikeMarkRight = drive.trajectorySequenceBuilder(offset.end())
                .strafeLeft(30)
                .forward(20)
                .back(14.5)
                .strafeLeft(5)
                .build();
        TrajectorySequence backdropRight = drive.trajectorySequenceBuilder(spikeMarkRight.end())
                .lineToConstantHeading(new Vector2d(-35,-57))
                .turn(Math.toRadians(90))
                .back(50)
                .lineToLinearHeading(new Pose2d(46, -27.3, Math.toRadians(180)))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backdropRight.end())
                .forward(10)
                .strafeLeft(22)
                .back(10)
                .build();

        waitForStart();

        if (!isStopRequested()) {
//            spikeMarkPosition = CloseVisionBlue.getPosition();
            spikeMarkPosition = 1;
            drive.followTrajectorySequence(offset);
            if (spikeMarkPosition == 1) {
                drive.followTrajectorySequence(spikeMarkLeft);
                sleep(4000);
                drive.followTrajectorySequence(backdropLeft);
                sleep(1000);
                drive.followTrajectorySequence(parkLeft);
            } else if (spikeMarkPosition == 2) {
                drive.followTrajectorySequence(spikeMarkMiddle);                sleep(4000);
                sleep(4000);
                drive.followTrajectorySequence(backdropMiddle);
                sleep(1000);
                drive.followTrajectorySequence(parkMiddle);
            } else {
                drive.followTrajectorySequence(spikeMarkRight);
                sleep(4000);
                drive.followTrajectorySequence(backdropRight);
                sleep(1000);
                drive.followTrajectorySequence(parkRight);
            }


            camera.closeCameraDevice();
        }
    }
}