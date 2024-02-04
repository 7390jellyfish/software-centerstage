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
import org.firstinspires.ftc.teamcode.Pipelines.FarVisionRed;

@Autonomous
public class FarRed extends LinearOpMode {
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

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
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
        FarVisionRed farVisionRed = new FarVisionRed(telemetry);
        camera.setPipeline(farVisionRed);
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

        Pose2d startPose = new Pose2d(-37.5, -61, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        // offset
        TrajectorySequence offset = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .build();

        // left
        TrajectorySequence spikeMarkLeft = drive.trajectorySequenceBuilder(offset.end())
                .lineToConstantHeading(new Vector2d(-52, -46))
                .build();
        TrajectorySequence backdropLeft = drive.trajectorySequenceBuilder(spikeMarkLeft.end())
                .lineToConstantHeading(new Vector2d(-44, -28))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-35,-57))
                .lineToConstantHeading(new Vector2d(15,-55))
                .addDisplacementMarker(() -> {
                    leftLift.setTargetPosition(1100);
                    rightLift.setTargetPosition(1100);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(47, -33))
                .lineToConstantHeading(new Vector2d(50, -33),
                        SampleMecanumDrive.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backdropLeft.end())
                .forward(10)
                .strafeLeft(20)
                .build();

        // middle
        TrajectorySequence spikeMarkMiddle = drive.trajectorySequenceBuilder(offset.end())
                .lineToConstantHeading(new Vector2d(-45.5, -34))
                .build();
        TrajectorySequence backdropMiddle = drive.trajectorySequenceBuilder(spikeMarkMiddle.end())
                .lineToConstantHeading(new Vector2d(-44, -28))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-35,-57))
                .lineToConstantHeading(new Vector2d(15,-55))
                .addDisplacementMarker(() -> {
                    leftLift.setTargetPosition(1100);
                    rightLift.setTargetPosition(1100);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(47, -33))
                .lineToConstantHeading(new Vector2d(50, -33),
                        SampleMecanumDrive.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backdropMiddle.end())
                .forward(10)
                .strafeLeft(20)
                .build();

        // right
        TrajectorySequence spikeMarkRight = drive.trajectorySequenceBuilder(offset.end())
                .lineToConstantHeading(new Vector2d(-46, -32))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-37.75, -28))
                .build();
        TrajectorySequence backdropRight = drive.trajectorySequenceBuilder(spikeMarkRight.end())
                .lineToConstantHeading(new Vector2d(-44, -28))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-35,-57))
                .lineToConstantHeading(new Vector2d(15,-55))
                .addDisplacementMarker(() -> {
                    leftLift.setTargetPosition(1100);
                    rightLift.setTargetPosition(1100);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                    rightLift.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(47, -33))
                .lineToConstantHeading(new Vector2d(50, -33),
                        SampleMecanumDrive.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backdropRight.end())
                .forward(10)
                .strafeLeft(20)
                .build();

        waitForStart();

        if (!isStopRequested()) {
            pacifier.setPosition(0);
            wrist.setPosition(0.43);
            claw.setPosition(1);
            spikeMarkPosition = FarVisionRed.getPosition();
            if (spikeMarkPosition == 1) {
                drive.followTrajectorySequence(offset);

                drive.followTrajectorySequence(spikeMarkLeft);
                sleep(500);
                while (pacifier.getPosition() <= 0.75) {
                    pacifier.setPosition(pacifier.getPosition() + 0.01);
                    sleep(9);
                }
                sleep(300);
                pacifier.setPosition(0);
                sleep(100);

                drive.followTrajectorySequence(backdropLeft);
                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(200);
                wrist.setPosition(0.57);
                sleep(200);
                claw.setPosition(0);
                sleep(100);

                drive.followTrajectorySequence(parkLeft);
            } else if (spikeMarkPosition == 2) {
                drive.followTrajectorySequence(offset);

                drive.followTrajectorySequence(spikeMarkMiddle);
                sleep(500);
                while (pacifier.getPosition() <= 0.75) {
                    pacifier.setPosition(pacifier.getPosition() + 0.01);
                    sleep(9);
                }
                sleep(300);
                pacifier.setPosition(0);
                sleep(100);

                drive.followTrajectorySequence(backdropMiddle);
                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(200);
                wrist.setPosition(0.57);
                sleep(200);
                claw.setPosition(0);
                sleep(100);

                drive.followTrajectorySequence(parkMiddle);
            } else {
                drive.followTrajectorySequence(offset);

                drive.followTrajectorySequence(spikeMarkRight);
                sleep(500);
                while (pacifier.getPosition() <= 0.75) {
                    pacifier.setPosition(pacifier.getPosition() + 0.01);
                    sleep(9);
                }
                sleep(300);
                pacifier.setPosition(0);
                sleep(100);

                drive.followTrajectorySequence(backdropRight);
                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(200);
                wrist.setPosition(0.57);
                sleep(200);
                claw.setPosition(0);
                sleep(100);

                drive.followTrajectorySequence(parkRight);
            }
            camera.closeCameraDevice();
        }
    }
}