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
public class FINALRedAutonJelly extends LinearOpMode {
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    DcMotor liftLeft = null;
    DcMotor liftRight = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo claw = null;
    Servo drone = null;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        liftLeft = hardwareMap.dcMotor.get("ll");
        liftRight = hardwareMap.dcMotor.get("rl");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        claw = hardwareMap.servo.get("claw");
        drone = hardwareMap.servo.get("drone");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(40)
                .back(37)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(34)
                .back(37)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(28)
                .back(37)
                .build();

        waitForStart();

        if (!isStopRequested()) {
//            if (spikeMark() == 1) {
//                drive.followTrajectorySequence(left);
//            } else if (spikeMark() == 2) {
//                drive.followTrajectorySequence(middle);
//            } else if (spikeMark() == 3) {
//                drive.followTrajectorySequence(right);
//            }
//            claw.setPosition(0);
//            liftLeft.setTargetPosition(10);
//            liftRight.setTargetPosition(10);
//            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftLeft.setPower(1);
//            liftLeft.setPower(1);
//            while(opModeIsActive() && liftLeft.isBusy() && liftRight.isBusy()) { }
//            liftLeft.setPower(0);
//            liftRight.setPower(0);
//            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            claw.setPosition(0.8);

            drive.followTrajectorySequence(left);
        }
    }
}