package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
@Disabled
public class RRCloseRedAutonJelly extends LinearOpMode {

    int prop = 1;
    Trajectory spikeMarkRight;
    Trajectory spikeMarkMiddle;
    Trajectory spikeMarkLeft;
    Trajectory backdropRight;
    Trajectory backdropMiddle;
    Trajectory backdropLeftA;
    Trajectory backdropLeftB;
    Trajectory load1A;
    Trajectory load1B;
    Trajectory load2A;
    Trajectory load2B;
    Trajectory load3A;
    Trajectory load3B;
    Trajectory deposit1A;
    Trajectory deposit1B;
    Trajectory deposit2A;
    Trajectory deposit2B;
    Trajectory deposit3A;
    Trajectory deposit3B;
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
    DcMotor liftLeft = null;
    DcMotor liftRight = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo claw = null;
    Servo drone = null;

    public double inchToTick (double inches) {
        return inches/0.0233749453;
    }

    public void outtakePixel() {
        intake.setPower(-1);
        transit.setPower(-1);
        sleep(1000);
        intake.setPower(0);
        transit.setPower(0);
    }

    public void moveLift(double inches) {
        int liftTicks = (int)(inchToTick(inches));
        liftLeft.setTargetPosition(liftTicks);
        liftRight.setTargetPosition(liftTicks);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(1);
        liftRight.setPower(1);
        while(liftLeft.isBusy() || liftRight.isBusy()) {}
        liftLeft.setPower(0);
        liftRight.setPower(0);
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
    public void runOpMode() {
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
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        drone.setDirection(Servo.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(14, 61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        spikeMarkRight = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(11, 34, Math.toRadians(180)))
                .build();
        spikeMarkMiddle = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, 34, Math.toRadians(270)))
                .build();
        spikeMarkLeft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(13, 34, Math.toRadians(0)))
                .build();
        backdropRight = drive.trajectoryBuilder(spikeMarkRight.end())
                .lineToSplineHeading(new Pose2d(49, 41, Math.toRadians(180)))
                .build();
        backdropMiddle = drive.trajectoryBuilder(spikeMarkMiddle.end())
                .lineToSplineHeading(new Pose2d(49, 34, Math.toRadians(180)))
                .build();
        backdropLeftA = drive.trajectoryBuilder(spikeMarkLeft.end())
                .lineToSplineHeading(new Pose2d(14, 61, Math.toRadians(0)))
                .build();
        backdropLeftB = drive.trajectoryBuilder(backdropLeftA.end())
                .lineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)))
                .build();
        //cycle
        load1A = drive.trajectoryBuilder(backdropRight.end())
                .lineToSplineHeading(new Pose2d(49, 11, Math.toRadians(180)))
                .build();
        load1B = drive.trajectoryBuilder(load1A.end())
                .lineToSplineHeading(new Pose2d(-58, 11, Math.toRadians(180)))
                .build();
        load2A = drive.trajectoryBuilder(backdropMiddle.end())
                .lineToSplineHeading(new Pose2d(49, 11, Math.toRadians(180)))
                .build();
        load2B = drive.trajectoryBuilder(load2A.end())
                .lineToSplineHeading(new Pose2d(-58, 11, Math.toRadians(180)))
                .build();
        load3A = drive.trajectoryBuilder(backdropLeftB.end())
                .lineToSplineHeading(new Pose2d(49, 11, Math.toRadians(180)))
                .build();
        load3B = drive.trajectoryBuilder(load3A.end())
                .lineToSplineHeading(new Pose2d(-58, 11, Math.toRadians(180)))
                .build();
        deposit1A = drive.trajectoryBuilder(load1B.end())
                .lineToSplineHeading(new Pose2d(49, 11, Math.toRadians(180)))
                .build();
        deposit1B = drive.trajectoryBuilder(deposit1A.end())
                .lineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)))
                .build();
        deposit2A = drive.trajectoryBuilder(load2B.end())
                .lineToSplineHeading(new Pose2d(49, 11, Math.toRadians(180)))
                .build();
        deposit2B = drive.trajectoryBuilder(deposit2A.end())
                .lineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)))
                .build();
        deposit3A = drive.trajectoryBuilder(load3B.end())
                .lineToSplineHeading(new Pose2d(49, 11, Math.toRadians(180)))
                .build();
        deposit3B = drive.trajectoryBuilder(deposit3A.end())
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
            drive.followTrajectory(backdropMiddle);
        } else if (prop == 3) {
            drive.followTrajectory(spikeMarkLeft);
            outtakePixel();
            drive.followTrajectory(backdropLeftA);
            drive.followTrajectory(backdropLeftB);
        }
        depositPixel();
        for (int i = 0; i < 2; i++) { // # of cycles
            if (prop == 1) {
                drive.followTrajectory(load1A);
                drive.followTrajectory(load1B);
                intakePixel();
                drive.followTrajectory(deposit1A);
                drive.followTrajectory(deposit1B);
            } else if (prop == 2) {
                drive.followTrajectory(load2A);
                drive.followTrajectory(load2B);
                intakePixel();
                drive.followTrajectory(deposit2A);
                drive.followTrajectory(deposit2B);
            } else if (prop == 3) {
                drive.followTrajectory(load3A);
                drive.followTrajectory(load3B);
                intakePixel();
                drive.followTrajectory(deposit3A);
                drive.followTrajectory(deposit3B);
            }
            depositPixel();
        }
    }
}