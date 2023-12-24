package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 90);

        drive.setPoseEstimate(startPose);

        TrajectorySequence spikeMark = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .addDisplacementMarker(() -> {
                    intake.setPower(-1);
                    sleep(1000);
                })
                .build();
        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(spikeMark.end())
                .forward(10)
                .addDisplacementMarker(() -> {
                    upDeposit();
                })
                .build();
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(backdrop.end())
                .forward(10)
                .addDisplacementMarker(() -> {
                    intake.setPower(1);
                    transit.setPower(1);
                    sleep(1000);
                })
                .forward(10)
                .addDisplacementMarker(() -> {
                    upDeposit();
                    downDeposit();
                    upDeposit();
                })
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(spikeMark);
            drive.followTrajectorySequence(backdrop);
            drive.followTrajectorySequence(cycle);
            upDeposit();
            downDeposit();
        }
    }
    void upDeposit() {
        claw.setPosition(0);
        sleep(1000);
        leftLift.setTargetPosition(1500);
        rightLift.setTargetPosition(1500);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(1);
        rightLift.setPower(1);
        while (opModeIsActive() && (leftLift.isBusy() && rightLift.isBusy())) { }
        leftLift.setPower(0);
        rightLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setPosition(0.9);
    }
    void downDeposit() {
        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(-1);
        rightLift.setPower(-1);
        while (opModeIsActive() && (leftLift.isBusy() && rightLift.isBusy())) { }
        leftLift.setPower(0);
        rightLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}