package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.Pipelines.VisionRed;

@Autonomous
public class CloseRed extends LinearOpMode {
    DcMotor leftLift = null;
    DcMotor rightLift = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo claw = null;
    @Override
    public void runOpMode() throws InterruptedException {
        leftLift = hardwareMap.dcMotor.get("ll");
        rightLift = hardwareMap.dcMotor.get("rl");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        claw = hardwareMap.servo.get("claw");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        VisionRed visionRed = new VisionRed(telemetry);
        camera.setPipeline(visionRed);
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

        Pose2d startPose = new Pose2d(0, 0, 180);

        drive.setPoseEstimate(startPose);

        TrajectorySequence spikeMarkLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(30)
                .turn(Math.toRadians(180))
                .forward(20)
                .back(13.5)
                .strafeRight(5)
                .build();
        TrajectorySequence spikeMarkMiddle = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(34)
                .strafeRight(11)
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence spikeMarkRight = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(22)
                .forward(6)
                .turn(Math.toRadians(90))
                .forward(10)
                .back(10)
                .build();

        TrajectorySequence backdropLeft1 = drive.trajectorySequenceBuilder(spikeMarkLeft.end())
                .strafeLeft(6)
                .back(43)
                .build();
        TrajectorySequence backdropLeft2 = drive.trajectorySequenceBuilder(backdropLeft1.end())
                .forward(10)
                .strafeLeft(24)
                .back(10)
                .build();
        TrajectorySequence backdropMiddle1 = drive.trajectorySequenceBuilder(spikeMarkMiddle.end())
                .forward(10)
                .build();
        TrajectorySequence backdropMiddle2 = drive.trajectorySequenceBuilder(backdropMiddle1.end())
                .forward(10)
                .build();
        TrajectorySequence backdropRight1 = drive.trajectorySequenceBuilder(spikeMarkRight.end())
                .forward(10)
                .build();
        TrajectorySequence backdropRight2 = drive.trajectorySequenceBuilder(backdropRight1.end())
                .forward(10)
                .build();

//        TrajectorySequence cycle = drive.trajectorySequenceBuilder(backdrop.end())
//                .forward(10)
//                .addDisplacementMarker(() -> {
//                    intake.setPower(1);
//                    transit.setPower(1);
//                    sleep(1000);
//                })
//                .forward(10)
//                .addDisplacementMarker(() -> {
//                    upDeposit();
//                    downDeposit();
//                    upDeposit();
//                })
//                .build();

        waitForStart();

        if (!isStopRequested()) {
            spikeMarkPosition = VisionRed.getPosition();
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);
            if (spikeMarkPosition == 1) {
                drive.followTrajectorySequence(spikeMarkLeft);
            } else if (spikeMarkPosition == 2) {
                drive.followTrajectorySequence(spikeMarkMiddle);
            } else {
                drive.followTrajectorySequence(spikeMarkRight);
            }
            intake.setPower(-0.5);
            transit.setPower(-1);
            sleep(4000);
            intake.setPower(0);
            transit.setPower(0);
            if (spikeMarkPosition == 1) {
//                drive.followTrajectorySequence(backdropLeft1);
//                upDeposit();
//                sleep(1000);
//                downDeposit();
//                drive.followTrajectorySequence(backdropLeft2);
            } else if (spikeMarkPosition == 2) {
//                drive.followTrajectorySequence(backdropMiddle1);
//                upDeposit();
//                sleep(1000);
//                downDeposit();
//                drive.followTrajectorySequence(backdropMiddle2);
            } else {
//                drive.followTrajectorySequence(backdropRight1);
//                upDeposit();
//                sleep(1000);
//                downDeposit();
//                drive.followTrajectorySequence(backdropRight2);
            }

//            drive.followTrajectorySequence(cycle);
//            upDeposit();
//            downDeposit();
        }
    }
    void upDeposit() {
        claw.setPosition(0);
        sleep(1000);
        rightLift.setTargetPosition(2000);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(1);
        rightLift.setPower(1);
        while (opModeIsActive() && rightLift.isBusy()) { }
        leftLift.setPower(0);
        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setPosition(0.6);
    }
    void downDeposit() {
        rightLift.setTargetPosition(0);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(-1);
        rightLift.setPower(-1);
        while (opModeIsActive() && rightLift.isBusy()) { }
        leftLift.setPower(0);
        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}