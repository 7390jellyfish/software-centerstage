package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Pipelines.CloseRedVision;

@Autonomous
public class CloseRed extends LinearOpMode {
    DcMotor leftLift = null;
    DcMotor rightLift = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo wrist = null;
    Servo claw = null;
    Servo pacifier = null;
    @Override
    public void runOpMode() throws InterruptedException {
        leftLift = hardwareMap.dcMotor.get("ll");
        rightLift = hardwareMap.dcMotor.get("rl");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");
        pacifier = hardwareMap.servo.get("pacifier");

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        pacifier.setDirection(Servo.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        CloseRedVision closeRedVision = new CloseRedVision(telemetry);
        camera.setPipeline(closeRedVision);
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

        Pose2d startPose = new Pose2d(12.7, -61, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        // offset
        TrajectorySequence offset = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .build();

        // left
        TrajectorySequence spikeMarkLeft = drive.trajectorySequenceBuilder(offset.end())
                .lineToConstantHeading(new Vector2d(15, -34.5))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(7.25, -34.5))
                .build();
        TrajectorySequence backdropLeft = drive.trajectorySequenceBuilder(spikeMarkLeft.end())
                .lineToConstantHeading(new Vector2d(15, -34.5))
                .turn(Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    leftLift.setTargetPosition(1100);
                    rightLift.setTargetPosition(1100);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(47, -25))
                .lineToConstantHeading(new Vector2d(50, -25),
                        SampleMecanumDrive.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backdropLeft.end())
                .forward(5)
                .strafeLeft(28)
                .build();

        // middle
        TrajectorySequence spikeMarkMiddle = drive.trajectorySequenceBuilder(offset.end())
                .lineToConstantHeading(new Vector2d(9.5, -34))
                .build();
        TrajectorySequence backdropMiddle = drive.trajectorySequenceBuilder(spikeMarkMiddle.end())
                .lineToConstantHeading(new Vector2d(15, -40))
                .turn(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    leftLift.setTargetPosition(1100);
                    rightLift.setTargetPosition(1100);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(47, -32))
                .lineToConstantHeading(new Vector2d(50, -32),
                        SampleMecanumDrive.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backdropMiddle.end())
                .forward(5)
                .strafeLeft(24)
                .build();

        // right
        TrajectorySequence spikeMarkRight = drive.trajectorySequenceBuilder(offset.end())
                .lineToConstantHeading(new Vector2d(16, -42))
                .build();
        TrajectorySequence backdropRight = drive.trajectorySequenceBuilder(spikeMarkRight.end())
                .lineToConstantHeading(new Vector2d(16, -50))
                .turn(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    leftLift.setTargetPosition(1100);
                    rightLift.setTargetPosition(1100);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(47, -37))
                .lineToConstantHeading(new Vector2d(50, -37),
                        SampleMecanumDrive.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backdropRight.end())
                .forward(5)
                .strafeLeft(22)
                .build();

        waitForStart();

        if (!isStopRequested()) {
            pacifier.setPosition(0.33);
            wrist.setPosition(0.43);
            claw.setPosition(1);
            spikeMarkPosition = CloseRedVision.getPosition();
            spikeMarkPosition = 3;
            if (spikeMarkPosition == 1) {
                drive.followTrajectorySequence(offset);

                drive.followTrajectorySequence(spikeMarkLeft);
                sleep(500);
                while (pacifier.getPosition() < 1) {
                    pacifier.setPosition(pacifier.getPosition() + 0.01);
                    sleep(9);
                }
                sleep(300);
                pacifier.setPosition(0.33);
                sleep(100);

                drive.followTrajectorySequence(backdropLeft);
                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(200);
                wrist.setPosition(0.57);
                sleep(1000);
                claw.setPosition(0.475);
                sleep(200);

                wrist.setPosition(0.43);
                claw.setPosition(0.51);
                drive.followTrajectorySequence(parkLeft);
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
            } else if (spikeMarkPosition == 2) {
                drive.followTrajectorySequence(offset);

                drive.followTrajectorySequence(spikeMarkMiddle);
                sleep(500);
                while (pacifier.getPosition() < 1) {
                    pacifier.setPosition(pacifier.getPosition() + 0.01);
                    sleep(9);
                }
                sleep(300);
                pacifier.setPosition(0.33);
                sleep(100);

                drive.followTrajectorySequence(backdropMiddle);
                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(200);
                wrist.setPosition(0.57);
                sleep(1000);
                claw.setPosition(0.475);
                sleep(200);

                wrist.setPosition(0.43);
                claw.setPosition(0.51);
                drive.followTrajectorySequence(parkMiddle);
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
            } else {
                drive.followTrajectorySequence(offset);

                drive.followTrajectorySequence(spikeMarkRight);
                sleep(500);
                while (pacifier.getPosition() < 1) {
                    pacifier.setPosition(pacifier.getPosition() + 0.01);
                    sleep(9);
                }
                sleep(300);
                pacifier.setPosition(0.33);
                sleep(100);

                drive.followTrajectorySequence(backdropRight);
                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(200);
                wrist.setPosition(0.57);
                sleep(1000);
                claw.setPosition(0.475);
                sleep(200);

                wrist.setPosition(0.43);
                claw.setPosition(0.51);
                drive.followTrajectorySequence(parkRight);
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
            camera.closeCameraDevice();
        }
    }
}