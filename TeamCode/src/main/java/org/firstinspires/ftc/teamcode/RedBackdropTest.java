package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class RedBackdropTest extends LinearOpMode {

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    DcMotor lift = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo claw = null;
    Servo ramp = null;
    Servo drone = null;

    public void moveLift(double ticks) {
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int liftTicks = (int)(ticks);
        lift.setTargetPosition(liftTicks);
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
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        claw = hardwareMap.servo.get("claw");
        ramp = hardwareMap.servo.get("ramp");
        drone = hardwareMap.servo.get("drone");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ramp.setPosition(-1.1);

        waitForStart();

        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backRightMotor.setPower(1);
        backLeftMotor.setPower(1);

        sleep(2000);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        depositPixel();
    }
}
