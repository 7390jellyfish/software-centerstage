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
public class FarRed extends LinearOpMode {
    DcMotor leftLift = null;
    DcMotor rightLift = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo wrist = null;
    Servo claw = null;
    @Override
    public void runOpMode() throws InterruptedException {
        leftLift = hardwareMap.dcMotor.get("ll");
        rightLift = hardwareMap.dcMotor.get("rl");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        Pose2d startPose = new Pose2d(16.5, 64, Math.toRadians(0));

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
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(53.5, 33))
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backdropLeft.end())
                .forward(10)
                .strafeRight(26)
                .back(10)
                .build();

        // middle
        TrajectorySequence spikeMarkMiddle = drive.trajectorySequenceBuilder(offset.end())
                .strafeLeft(34)
                .strafeRight(11)
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence backdropMiddle = drive.trajectorySequenceBuilder(spikeMarkMiddle.end())
                .back(10)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(53.5, 33))
                .build();
        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backdropMiddle.end())
                .forward(10)
                .strafeRight(26)
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
                .lineToConstantHeading(new Vector2d(53.5, 33))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backdropRight.end())
                .forward(10)
                .strafeRight(26)
                .back(10)
                .build();

        waitForStart();

        if (!isStopRequested()) {
            spikeMarkPosition = CloseVisionBlue.getPosition();
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);
            drive.followTrajectorySequence(offset);
            if (spikeMarkPosition == 1) {
                drive.followTrajectorySequence(spikeMarkLeft);
            } else if (spikeMarkPosition == 2) {
                drive.followTrajectorySequence(spikeMarkMiddle);
            } else {
                drive.followTrajectorySequence(spikeMarkRight);
            }
            intake.setPower(-0.4);
            transit.setPower(-1);
            sleep(4000);
            intake.setPower(0);
            transit.setPower(0);
            if (spikeMarkPosition == 1) {
                drive.followTrajectorySequence(backdropLeft);
                upDeposit();
                sleep(1000);
                downDeposit();
                drive.followTrajectorySequence(parkLeft);
            }
            else if (spikeMarkPosition == 2) {
                drive.followTrajectorySequence(backdropMiddle);
                upDeposit();
                sleep(1000);
                downDeposit();
                drive.followTrajectorySequence(parkMiddle);
            } else {
                drive.followTrajectorySequence(backdropRight);
                upDeposit();
                sleep(1000);
                downDeposit();
                drive.followTrajectorySequence(parkRight);
            }
            camera.closeCameraDevice();
        }
    }
    void upDeposit() {
        wrist.setPosition(0.48);
        claw.setPosition(1);
        sleep(1000);
        leftLift.setTargetPosition(1500);
        rightLift.setTargetPosition(1500);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(1);
        rightLift.setPower(1);
        while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
        leftLift.setPower(0);
        rightLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setPosition(0.65);
        claw.setPosition(0.1);
    }
    void downDeposit() {
        wrist.setPosition(0.48);
        claw.setPosition(0.1);
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(-1);
        rightLift.setPower(-1);
        while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
        leftLift.setPower(0);
        rightLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}