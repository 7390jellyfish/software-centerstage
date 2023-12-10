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
public class FINALBlueAutonJelly extends LinearOpMode {
    DcMotor liftLeft = null;
    DcMotor liftRight = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        liftLeft = hardwareMap.dcMotor.get("ll");
        liftRight = hardwareMap.dcMotor.get("rl");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        claw = hardwareMap.servo.get("claw");

        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(20)
                .back(40)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(backdrop.end())
                .forward(10)
                .turn(Math.toRadians(180))
                .strafeLeft(20)
                .forward(10)
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(backdrop);
            claw.setPosition(0);
            sleep(1500);
            liftRight.setTargetPosition(1500);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setPower(1);
            liftLeft.setPower(1);
            while (opModeIsActive() && liftRight.isBusy()) { }
            liftRight.setPower(0);
            liftLeft.setPower(0);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(500);
            claw.setPosition(0.8);
            sleep(1500);

            drive.followTrajectorySequence(park);
            intake.setPower(-1);
            transit.setPower(-1);
            sleep(5000);
            intake.setPower(0);
            transit.setPower(0);

//            claw.setPosition(0.8);
//            sleep(1500);
//            liftRight.setTargetPosition(0);
//            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftRight.setPower(1);
//            liftLeft.setPower(1);
//            while (opModeIsActive() && liftRight.isBusy()) { }
//            liftRight.setPower(0);
//            liftLeft.setPower(0);
//            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            sleep(500);
//            claw.setPosition(0);
//            sleep(1500);
//
//            claw.setPosition(0);
//            sleep(1500);
//            liftRight.setTargetPosition(1500);
//            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftRight.setPower(1);
//            liftLeft.setPower(1);
//            while (opModeIsActive() && liftRight.isBusy()) { }
//            liftRight.setPower(0);
//            liftLeft.setPower(0);
//            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            sleep(500);
//            claw.setPosition(0.8);
//            sleep(1500);
        }
    }
}