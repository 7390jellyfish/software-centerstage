package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class JellyAuton extends LinearOpMode {

    int prop;
    Trajectory spikeMarkMiddle;
    Trajectory spikeMarkLeft;
    Trajectory spikeMarkRight;
    Trajectory backdropLeft;
    Trajectory backdropMiddle;
    Trajectory backdropRight;
    Trajectory load1;
    Trajectory load2;
    Trajectory load3;
    Trajectory deposit1;
    Trajectory deposit2;
    Trajectory deposit3;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
    DcMotor lift = hardwareMap.dcMotor.get("lift");
    DcMotor intake = hardwareMap.dcMotor.get("intake");
    Servo claw = hardwareMap.servo.get("claw");

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

    public void outtakePixel() {
        intake.setPower(-1);
        sleep(1000);
    }
    public void runOpMode() {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        spikeMarkLeft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, 34, Math.toRadians(180)))
                .build();
        spikeMarkMiddle = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, 34, Math.toRadians(270)))
                .build();
        spikeMarkRight = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-34, 34, Math.toRadians(0)))
                .build();
        backdropLeft = drive.trajectoryBuilder(spikeMarkLeft.end())
                .lineToSplineHeading(new Pose2d(49, 34, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(49, 41, Math.toRadians(180)))
                .build();
        backdropMiddle = drive.trajectoryBuilder(spikeMarkMiddle.end())
                .lineToSplineHeading(new Pose2d(2, 35, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)))
                .build();
        backdropRight = drive.trajectoryBuilder(spikeMarkRight.end())
                .lineToSplineHeading(new Pose2d(10, 34, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)))
                .build();
        //cycle functions
        load1 = drive.trajectoryBuilder(backdropLeft.end())
                .lineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-60, 35, Math.toRadians(180)))
                .build();
        load2 = drive.trajectoryBuilder(backdropMiddle.end())
                .lineToSplineHeading(new Pose2d(-60, 35, Math.toRadians(180)))
                .build();
        load3 = drive.trajectoryBuilder(backdropRight.end())
                .lineToSplineHeading(new Pose2d(49, 34, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-60, 35, Math.toRadians(180)))
                .build();
        deposit1 = drive.trajectoryBuilder(load1.end())
                .lineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)))
                .build();
        deposit2 = drive.trajectoryBuilder(load2.end())
                .lineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)))
                .build();
        deposit3 = drive.trajectoryBuilder(load3.end())
                .lineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        if (prop == 1) {
            drive.followTrajectory(spikeMarkLeft);
            outtakePixel();
            drive.followTrajectory(backdropLeft);
        } else if (prop == 2) {
            drive.followTrajectory(spikeMarkMiddle);
            outtakePixel();
            drive.followTrajectory(backdropMiddle);
        } else if (prop == 3) {
            drive.followTrajectory(spikeMarkRight);
            outtakePixel();
            drive.followTrajectory(backdropRight);
        }
        depositPixel();
        for (int i = 0; i < 2; i++) { // # cycles
            if (prop == 1) {
                drive.followTrajectory(load1);
            } else if (prop == 2) {
                drive.followTrajectory(load2);
            } else if (prop == 3) {
                drive.followTrajectory(load3);
            }
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);
            if (prop == 1) {
                drive.followTrajectory(deposit1);
            } else if (prop == 2) {
                drive.followTrajectory(deposit2);
            } else if (prop == 3) {
                drive.followTrajectory(deposit3);
            }
            depositPixel();
        }
    }
}
