package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class JellyAutonCloseBlue extends LinearOpMode {

    int prop = 1;
    Trajectory spikeMarkRight;
    Trajectory spikeMarkMiddle;
    Trajectory spikeMarkLeft;
    Trajectory backdropRight;
    Trajectory backdropMiddleA;
    Trajectory backdropMiddleB;
    Trajectory backdropLeftA;
    Trajectory backdropLeftB;
    Trajectory load1a;
    Trajectory load1b;
    Trajectory load2a;
    Trajectory load2b;
    Trajectory load3a;
    Trajectory load3b;

    Trajectory deposit1a;
    Trajectory deposit1b;
    Trajectory deposit2a;
    Trajectory deposit2b;
    Trajectory deposit3a;
    Trajectory deposit3b;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    DcMotor lift = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo claw = null;

    public double inchToTick (double inches) {
        return inches/0.0233749453;
    }

    public void moveLift(double inches) {
        int liftTicks = (int)(inchToTick(inches));
        lift.setTargetPosition(liftTicks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        while(lift.isBusy()) {}
        lift.setPower(0);
    }

    public void depositPixel() {
        moveLift(4);
        claw.setPosition(180);
        sleep(1000);
        claw.setPosition(0);
        moveLift(0);
    }

    public void intakePixel() {
        intake.setPower(1);
        transit.setPower(1);
        sleep(1000);
        intake.setPower(0);
        transit.setPower(0);
    }

    public void outtakePixel() {
        intake.setPower(-1);
        transit.setPower(-1);
        sleep(1000);
        intake.setPower(0);
        transit.setPower(0);
    }
    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        claw = hardwareMap.servo.get("claw");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        spikeMarkRight = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, 34, Math.toRadians(180)))
                .build();
        spikeMarkMiddle = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, 34, Math.toRadians(270)))
                .build();
        spikeMarkLeft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, 34, Math.toRadians(0)))
                .build();
        backdropRight = drive.trajectoryBuilder(spikeMarkRight.end())
                .lineToSplineHeading(new Pose2d(49, 38, Math.toRadians(180)))
                .build();
        backdropMiddleA = drive.trajectoryBuilder(spikeMarkMiddle.end())
                .lineToSplineHeading(new Pose2d(49, 34, Math.toRadians(180)))
                .build();
        backdropMiddleB = drive.trajectoryBuilder(backdropMiddleA.end())
                .lineToSplineHeading(new Pose2d(49, 10, Math.toRadians(180)))
                .build();
        backdropLeftA = drive.trajectoryBuilder(spikeMarkLeft.end())
                .lineToSplineHeading(new Pose2d(12, 61, Math.toRadians(180)))
                .build();
        backdropLeftB = drive.trajectoryBuilder(backdropLeftA.end())
                .lineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)))
                .build();
        //cycle functions
        load1a = drive.trajectoryBuilder(backdropRight.end())
                .lineToSplineHeading(new Pose2d(14, 10, Math.toRadians(180)))
                .build();
        load1b = drive.trajectoryBuilder(load1a.end())
                .lineToSplineHeading(new Pose2d(-60, 10, Math.toRadians(180)))
                .build();
        load2a = drive.trajectoryBuilder(backdropMiddleB.end())
                .lineToSplineHeading(new Pose2d(-60, 10, Math.toRadians(180)))
                .build();
        load3a = drive.trajectoryBuilder(backdropLeftB.end())
                .lineToSplineHeading(new Pose2d(49, 10, Math.toRadians(180)))
                .build();
        load3b = drive.trajectoryBuilder(backdropLeftB.end())
                .lineToSplineHeading(new Pose2d(-60, 10, Math.toRadians(180)))
                .build();
        deposit1a = drive.trajectoryBuilder(load1b.end())
                .lineToSplineHeading(new Pose2d(14, 10, Math.toRadians(180)))
                .build();
        deposit1b = drive.trajectoryBuilder(deposit1a.end())
                .lineToSplineHeading(new Pose2d(49, 38, Math.toRadians(180)))
                .build();
        deposit2a = drive.trajectoryBuilder(load2b.end())
                .lineToSplineHeading(new Pose2d(49, 10, Math.toRadians(180)))
                .build();
        deposit2b = drive.trajectoryBuilder(deposit2a.end())
                .lineToSplineHeading(new Pose2d(49, 34, Math.toRadians(180)))
                .build();
        deposit3a = drive.trajectoryBuilder(load3b.end())
                .lineToSplineHeading(new Pose2d(49, 10, Math.toRadians(180)))
                .build();
        deposit3b = drive.trajectoryBuilder(deposit3a.end())
                .lineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        // 1 = right, 2 = middle, 3 = left
        if (prop == 1) {
            drive.followTrajectory(spikeMarkRight);
            outtakePixel();
            drive.followTrajectory(backdropRight);
        } else if (prop == 2) {
            drive.followTrajectory(spikeMarkMiddle);
            outtakePixel();
            drive.followTrajectory(backdropMiddleA);
            drive.followTrajectory(backdropMiddleB);
        } else if (prop == 3) {
            drive.followTrajectory(spikeMarkLeft);
            outtakePixel();
            drive.followTrajectory(backdropLeftA);
            drive.followTrajectory(backdropLeftB);
        }
        depositPixel();
        for (int i = 0; i < 2; i++) { // # cycles
            if (prop == 1) {
                drive.followTrajectory(load1a);
                drive.followTrajectory(load1b);
                intakePixel();
                drive.followTrajectory(deposit1a);
                drive.followTrajectory(deposit1b);
            } else if (prop == 2) {
                drive.followTrajectory(load2a);
                drive.followTrajectory(load2b);
                intakePixel();
                drive.followTrajectory(deposit2a);
                drive.followTrajectory(deposit2b);
            } else if (prop == 3) {
                drive.followTrajectory(load3a);
                drive.followTrajectory(load3b);
                intakePixel();
                drive.followTrajectory(deposit3a);
                drive.followTrajectory(deposit3b);
            }
            depositPixel();
        }
    }
}