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
public class JellyAuton extends LinearOpMode {

    Trajectory spikeMark;
    Trajectory backDrop;
    Trajectory load;
    Trajectory deposit;
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
        lift.setPower(0.25);
        while(lift.isBusy()) {}
        lift.setPower(0);
    }

    public void depositPixel() {
        moveLift(4);
        claw.setPosition(180);
        sleep(300);
        claw.setPosition(0);
        moveLift(0);
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

        spikeMark = drive.trajectoryBuilder(new Pose2d())
                .forward(1)
                .build();

        backDrop = drive.trajectoryBuilder(new Pose2d())
                .back(1)
                .strafeLeft(1)
                .forward(1)
                .build();
        //cycle functions
        load = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(1)
                .forward(1)
                .strafeRight(1)
                .build();
        deposit = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(1)
                .back(1)
                .strafeLeft(1)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(  spikeMark);
        drive.followTrajectory(backDrop);
        drive.turn(Math.toRadians(90));
        depositPixel();
        for (int i = 0; i < 2; i++) { // # cycles
            drive.followTrajectory(load);
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);
            drive.followTrajectory(deposit)
            depositPixel();
        }
    }
}
