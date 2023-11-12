package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor transit = hardwareMap.dcMotor.get("transit");
        Servo claw = hardwareMap.servo.get("claw");
        Servo ramp = hardwareMap.servo.get("ramp");
        Servo drone = hardwareMap.servo.get("drone");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
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

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            double liftPower = gamepad2.right_trigger - gamepad2.left_trigger;

            if (gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0) {
                lift.setPower(liftPower);
            } else {
                lift.setPower(0);
            }

            telemetry.addData("lift power", liftPower);
            telemetry.addData("lift position", lift.getCurrentPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("ramp", ramp.getPosition());
            telemetry.addData("front right", frontRightMotor.getCurrentPosition());
            telemetry.addData("front left", frontLeftMotor.getCurrentPosition());
            telemetry.addData("back right", backRightMotor.getCurrentPosition());
            telemetry.addData("back left", backLeftMotor.getCurrentPosition());
            telemetry.update();

            if (gamepad2.right_bumper){
                transit.setPower(1);
                intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                transit.setPower(-1);
                intake.setPower(-1);
            } else {
                transit.setPower(0);
                intake.setPower(0);
            }
            if (gamepad2.x){
                claw.setPosition(0.45);
                System.out.println("Triggered.");
            } else if(gamepad2.y){
                System.out.println("triggered y");
                claw.setPosition(0.6);
            }
            if (gamepad2.dpad_right) {
                drone.setDirection(Servo.Direction.REVERSE);
                drone.setPosition(5);
            }
            if (gamepad2.dpad_left) {
                drone.setDirection(Servo.Direction.FORWARD);
                drone.setPosition(3);
            }
        }
    }
}