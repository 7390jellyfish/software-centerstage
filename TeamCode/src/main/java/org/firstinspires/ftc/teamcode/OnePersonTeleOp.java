package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OnePersonTeleOp extends LinearOpMode {
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontLeftMotor = null;
    DcMotor leftLift = null;
    DcMotor rightLift = null;
    DcMotor intake = null;
    DcMotor transit = null;
    Servo wrist = null;
    Servo claw = null;
    Servo drone = null;
    Servo pacifier = null;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRightMotor = hardwareMap.dcMotor.get("fl");
        backRightMotor = hardwareMap.dcMotor.get("bl");
        backLeftMotor = hardwareMap.dcMotor.get("fr");
        frontLeftMotor = hardwareMap.dcMotor.get("br");
        leftLift = hardwareMap.dcMotor.get("ll");
        rightLift = hardwareMap.dcMotor.get("rl");
        intake = hardwareMap.dcMotor.get("intake");
        transit = hardwareMap.dcMotor.get("transit");
        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");
        drone = hardwareMap.servo.get("drone");
        pacifier = hardwareMap.servo.get("pacifier");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        pacifier.setDirection(Servo.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // dt
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // lift
//            if (gamepad1.dpad_left && !gamepad1.dpad_right) {
//                leftLift.setTargetPosition(1500);
//                rightLift.setTargetPosition(1500);
//                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftLift.setPower(1);
//                rightLift.setPower(1);
//                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
//                leftLift.setPower(0);
//                rightLift.setPower(0);
//                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            if (gamepad1.dpad_right && !gamepad1.dpad_left) {
//                leftLift.setTargetPosition(1250);
//                rightLift.setTargetPosition(1250);
//                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftLift.setPower(1);
//                rightLift.setPower(1);
//                while (opModeIsActive() && (leftLift.isBusy() || rightLift.isBusy())) { }
//                leftLift.setPower(0);
//                rightLift.setPower(0);
//                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
            double liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (((liftPower > 0) && ((leftLift.getCurrentPosition() < 2000) && (rightLift.getCurrentPosition() < 2000))) || (liftPower < 0)) {
                leftLift.setPower(liftPower);
                rightLift.setPower(liftPower);
//                if (liftPower > 0) {
//                    if ((leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) > 25) {
//                        leftLift.setPower(0);
//                        rightLift.setPower(liftPower);
//                    }
//                    if ((rightLift.getCurrentPosition() - leftLift.getCurrentPosition()) > 25) {
//                        leftLift.setPower(liftPower);
//                        rightLift.setPower(0);
//                    }
//                }
//                if (liftPower < 0) {
//                    if ((leftLift.getCurrentPosition() - rightLift.getCurrentPosition()) > 25) {
//                        leftLift.setPower(liftPower);
//                        rightLift.setPower(0);
//                    }
//                    if ((rightLift.getCurrentPosition() - leftLift.getCurrentPosition()) > 25) {
//                        leftLift.setPower(0);
//                        rightLift.setPower(liftPower);
//                    }
//                }
//                if (leftLift.getCurrentPosition() == rightLift.getCurrentPosition()) {
//                    leftLift.setPower(liftPower);
//                    rightLift.setPower(liftPower);
//                }
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }

            // intake
            double intakePower = (gamepad1.right_bumper ? 1.0 : 0.0) - (gamepad1.left_bumper ? 1.0 : 0.0);
            double transitPower =  (gamepad1.right_bumper ? 1.0 : 0.0) - (gamepad1.left_bumper ? 1.0 : 0.0);
            intake.setPower(intakePower);
            transit.setPower(transitPower * 0.5);

            // wrist
//            if (leftLift.getCurrentPosition() < 500) {
//                wrist.setPosition(0.43);
//            } else if (leftLift.getCurrentPosition() > 500) {
//                wrist.setPosition(0.57);
//            }
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                wrist.setPosition(0.57);
//                wrist.setPosition(wrist.getPosition() + 0.01);
//                while (gamepad1.dpad_up) { }
            }
            if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                wrist.setPosition(0.43);
//                wrist.setPosition(wrist.getPosition() - 0.01);
//                while (gamepad1.dpad_down) { }
            }

            // claw
//            if ((liftPower < 0) && (leftLift.getCurrentPosition() < 750) && (leftLift.getCurrentPosition() > 500)) {
//                claw.setPosition(0.25);
//            }
            if (gamepad1.a && !gamepad1.b) {
                claw.setPosition(0.25);
//                claw.setPosition(claw.getPosition() - 0.01);
//                while (gamepad1.a) { }
            }
            if (gamepad1.b && !gamepad1.a) {
                claw.setPosition(1);
//                claw.setPosition(claw.getPosition() + 0.01);
//                while (gamepad1.b) { }
            }

            // drone
            if (gamepad1.y && !gamepad1.x) {
                drone.setDirection(Servo.Direction.REVERSE);
                drone.setPosition(1);
            }
            if (gamepad1.x && !gamepad1.y) {
                drone.setDirection(Servo.Direction.FORWARD);
                drone.setPosition(1);
            }

            // pacifier
            if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                pacifier.setPosition(0);
//                pacifier.setPosition(pacifier.getPosition() + 0.01);
//                while (gamepad1.dpad_left) { }
            }
            if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                while (pacifier.getPosition() <= 0.75) {
                    pacifier.setPosition(pacifier.getPosition() - 0.01);
                    sleep(9);
                }
//                pacifier.setPosition(pacifier.getPosition() - 0.01);
//                while (gamepad1.dpad_right) { }
            }

            telemetry.addData("vertical joystick", y);
            telemetry.addData("horizontal joystick", x);
            telemetry.addData("rotational joystick", rx);
            telemetry.addData("front right", frontRightMotor.getCurrentPosition());
            telemetry.addData("back right", backRightMotor.getCurrentPosition());
            telemetry.addData("back left", backLeftMotor.getCurrentPosition());
            telemetry.addData("front left", frontLeftMotor.getCurrentPosition());
            telemetry.addData("left lift", leftLift.getCurrentPosition());
            telemetry.addData("right lift", rightLift.getCurrentPosition());
            telemetry.addData("intake", intake.getCurrentPosition());
            telemetry.addData("transit", transit.getCurrentPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("drone", drone.getPosition());
            telemetry.addData("pacifier", pacifier.getPosition());
            telemetry.update();
        }
    }
}