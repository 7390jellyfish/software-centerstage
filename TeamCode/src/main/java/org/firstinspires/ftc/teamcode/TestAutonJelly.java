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
public class TestAutonJelly extends LinearOpMode {

    int prop = 1;
    Trajectory test1;
    Trajectory test2;
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
//    Servo ramp = null;
//    Servo drone = null;

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
        liftRight = hardwareMap.dcMotor.get("lr");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        claw = hardwareMap.servo.get("claw");
//        ramp = hardwareMap.servo.get("ramp");
//        drone = hardwareMap.servo.get("drone");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
//        ramp.setDirection(Servo.Direction.FORWARD);
//        drone.setDirection(Servo.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        ramp.setPosition(-1.1);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        test1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, 34, Math.toRadians(180)))
                .build();
        test2 = drive.trajectoryBuilder(test1.end())
                .lineToSplineHeading(new Pose2d(-35, 34, Math.toRadians(270)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(test1);
        drive.followTrajectory(test2);
        drive.turn(Math.toRadians(180));
    }
}