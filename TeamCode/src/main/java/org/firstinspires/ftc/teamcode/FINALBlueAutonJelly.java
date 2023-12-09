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
    Servo drone = null;

    @Override
    public void runOpMode() throws InterruptedException {
        liftLeft = hardwareMap.dcMotor.get("ll");
        liftRight = hardwareMap.dcMotor.get("rl");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        claw = hardwareMap.servo.get("claw");
        drone = hardwareMap.servo.get("drone");


        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(14.7)
                .back(42)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(5)
                .back(5)
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(backdrop);
            claw.setPosition(0);
            sleep(1500);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setTargetPosition(1500);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftLeft.setPower(1 - 0.03);
            liftRight.setPower(1);
            while(opModeIsActive() && liftRight.isBusy()) { }
            liftLeft.setPower(0);
            liftRight.setPower(0);
            sleep(500);
            claw.setPosition(0.8);
            drive.followTrajectorySequence(park);

//            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            liftRight.setTargetPosition(0);
//            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftLeft.setPower(1 - 0.03);
//            liftRight.setPower(1);
//            while(opModeIsActive() && liftRight.isBusy()) { }
//            liftLeft.setPower(0);
//            liftRight.setPower(0);
//            transit.setPower(0.7);
//            sleep(5000);
//            transit.setPower(0);
//
//            claw.setPosition(0);
//            sleep(1500);
//            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            liftRight.setTargetPosition(1500);
//            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftLeft.setPower(1 - 0.03);
//            liftRight.setPower(1);
//            while(opModeIsActive() && liftRight.isBusy()) { }
//            liftLeft.setPower(0);
//            liftRight.setPower(0);
//            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            sleep(500);
//            claw.setPosition(0.8);
        }
    }
}