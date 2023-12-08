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
public class RRFarRedAutonJelly extends LinearOpMode {

    int prop = 1;
    Trajectory spikeMarkRight;
    Trajectory spikeMarkMiddle;
    Trajectory spikeMarkLeft;
    Trajectory backdropRightA;
    Trajectory backdropRightB;
    Trajectory backdropRightC;
    Trajectory backdropMiddleA;
    Trajectory backdropMiddleB;
    Trajectory backdropMiddleC;
    Trajectory backdropMiddleD;
    Trajectory backdropLeftA;
    Trajectory backdropLeftB;
    Trajectory backdropLeftC;
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
<<<<<<< HEAD
=======
//        ramp = hardwareMap.servo.get("ramp");
>>>>>>> 7a44f9691e7f532dc7e84cb8de60ea04cfe90a26
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
<<<<<<< HEAD
=======
//        ramp.setDirection(Servo.Direction.FORWARD);
>>>>>>> 7a44f9691e7f532dc7e84cb8de60ea04cfe90a26
        drone.setDirection(Servo.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

<<<<<<< HEAD
=======
//        ramp.setPosition(-1.1);

>>>>>>> 7a44f9691e7f532dc7e84cb8de60ea04cfe90a26
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        spikeMarkLeft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, -34, Math.toRadians(180)))
                .build();
        spikeMarkMiddle = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-35, -34, Math.toRadians(270)))
                .build();
        spikeMarkRight = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-34, -34, Math.toRadians(0)))
                .build();
        backdropLeftA = drive.trajectoryBuilder(spikeMarkLeft.end())
                .lineToSplineHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                .build();
        backdropLeftB = drive.trajectoryBuilder(backdropLeftA.end())
                .lineToSplineHeading(new Pose2d(49, -11, Math.toRadians(180)))
                .build();
        backdropLeftC = drive.trajectoryBuilder(backdropLeftB.end())
                .lineToSplineHeading(new Pose2d(49, -28, Math.toRadians(180)))
                .build();
        backdropMiddleA = drive.trajectoryBuilder(spikeMarkMiddle.end())
                .lineToSplineHeading(new Pose2d(-35, -58, Math.toRadians(270)))
                .build();
        backdropMiddleB = drive.trajectoryBuilder(backdropMiddleA.end())
                .lineToSplineHeading(new Pose2d(0, -58, Math.toRadians(270)))
                .build();
        backdropMiddleC = drive.trajectoryBuilder(backdropMiddleB.end())
                .lineToSplineHeading(new Pose2d(49, -58, Math.toRadians(180)))
                .build();
        backdropMiddleD = drive.trajectoryBuilder(backdropMiddleC.end())
                .lineToSplineHeading(new Pose2d(49, -36, Math.toRadians(180)))
                .build();
        backdropRightA = drive.trajectoryBuilder(spikeMarkRight.end())
                .lineToSplineHeading(new Pose2d(-34, -10, Math.toRadians(0)))
                .build();
        backdropRightB = drive.trajectoryBuilder(backdropRightA.end())
                .lineToSplineHeading(new Pose2d(49, -10, Math.toRadians(180)))
                .build();
        backdropRightC = drive.trajectoryBuilder(backdropRightB.end())
                .lineToSplineHeading(new Pose2d(49, -41, Math.toRadians(180)))
                .build();
        // cycle
        load1A = drive.trajectoryBuilder(backdropLeftC.end())
                .lineToSplineHeading(new Pose2d(49, -11, Math.toRadians(180)))
                .build();
        load1B = drive.trajectoryBuilder(load1A.end())
                .lineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(180)))
                .build();
        load2A = drive.trajectoryBuilder(backdropMiddleD.end())
                .lineToSplineHeading(new Pose2d(49, -11, Math.toRadians(180)))
                .build();
        load2B = drive.trajectoryBuilder(load2A.end())
                .lineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(180)))
                .build();
        load3A = drive.trajectoryBuilder(backdropRightC.end())
                .lineToSplineHeading(new Pose2d(49, -11, Math.toRadians(180)))
                .build();
        load3B = drive.trajectoryBuilder(load3A.end())
                .lineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(180)))
                .build();
        deposit1A = drive.trajectoryBuilder(load1B.end())
                .lineToSplineHeading(new Pose2d(49, -11, Math.toRadians(180)))
                .build();
        deposit1B = drive.trajectoryBuilder(deposit1A.end())
                .lineToSplineHeading(new Pose2d(49, -29, Math.toRadians(180)))
                .build();
        deposit2A = drive.trajectoryBuilder(load2B.end())
                .lineToSplineHeading(new Pose2d(49, -11, Math.toRadians(180)))
                .build();
        deposit2B = drive.trajectoryBuilder(deposit2A.end())
                .lineToSplineHeading(new Pose2d(49, -29, Math.toRadians(180)))
                .build();
        deposit3A = drive.trajectoryBuilder(load3B.end())
                .lineToSplineHeading(new Pose2d(49, -11, Math.toRadians(180)))
                .build();
        deposit3B = drive.trajectoryBuilder(deposit3A.end())
                .lineToSplineHeading(new Pose2d(49, -29, Math.toRadians(180)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        // 1 = right, 2 = middle, 3 = left
        if (prop == 1) {
            drive.followTrajectory(spikeMarkRight);
            outtakePixel();
            drive.followTrajectory(backdropRightA);
            drive.followTrajectory(backdropRightB);
            drive.followTrajectory(backdropRightC);
        } else if (prop == 2) {
            drive.followTrajectory(spikeMarkMiddle);
            outtakePixel();
            drive.followTrajectory(backdropMiddleA);
            drive.followTrajectory(backdropMiddleB);
            drive.followTrajectory(backdropMiddleC);
            drive.followTrajectory(backdropMiddleD);
        } else if (prop == 3) {
            drive.followTrajectory(spikeMarkLeft);
            outtakePixel();
            drive.followTrajectory(backdropLeftA);
            drive.followTrajectory(backdropLeftB);
            drive.followTrajectory(backdropLeftC);
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