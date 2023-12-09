package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpJelly extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
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
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transit.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // dt
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // lift
            double liftPower = gamepad2.right_trigger - gamepad2.left_trigger;
            if ((gamepad2.right_trigger != 0) || (gamepad2.left_trigger != 0)) {
                if (gamepad2.right_trigger == 0) {
                    claw.setPosition(0.9);
                }
                liftLeft.setPower(liftPower - 0.03);
                liftRight.setPower(liftPower);
            } else {
                liftLeft.setPower(0);
                liftRight.setPower(0);
            }
//            if (gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down && !gamepad2.dpad_left) {
//                liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftLeft.setTargetPosition(100);
//                liftRight.setTargetPosition(100);
//                resetLiftEncoders();
//            }
//            if (gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_left) {
//                liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftLeft.setTargetPosition(1000);
//                liftRight.setTargetPosition(1000);
//                resetLiftEncoders();
//            }
//            if (gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_left) {
//                liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftLeft.setTargetPosition(10);
//                liftRight.setTargetPosition(10);
//                resetLiftEncoders();
//            }
//            if (gamepad2.dpad_left && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down) {
//                liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftLeft.setTargetPosition(10);
//                liftRight.setTargetPosition(10);
//                resetLiftEncoders();
//            }

            // intake
            double intakePower = (gamepad2.right_bumper ? 1.0 : 0.0) - (gamepad2.left_bumper ? 1.0 : 0.0);
            double transitPower =  (gamepad2.right_bumper ? 1.0 : 0.0) - (gamepad2.left_bumper ? 1.0 : 0.0);
            intake.setPower(intakePower);
            transit.setPower(transitPower * 0.8);

            // claw
            if (gamepad2.b && !gamepad2.a) {
                claw.setPosition(0);
            }
            if (gamepad2.a && !gamepad2.b) {
                claw.setPosition(0.9);
            }

            // drone
            if (gamepad2.y && !gamepad2.x) {
                drone.setDirection(Servo.Direction.REVERSE);
                drone.setPosition(1);
            }
            if (gamepad2.x && !gamepad2.y) {
                drone.setDirection(Servo.Direction.FORWARD);
                drone.setPosition(1);
            }

            telemetry.addData("front left", frontLeftMotor.getCurrentPosition());
            telemetry.addData("back left", backLeftMotor.getCurrentPosition());
            telemetry.addData("back right", backRightMotor.getCurrentPosition());
            telemetry.addData("front right", frontRightMotor.getCurrentPosition());
            telemetry.addData("lift left", liftLeft.getCurrentPosition());
            telemetry.addData("lift right", liftRight.getCurrentPosition());
            telemetry.addData("intake", intake.getCurrentPosition());
            telemetry.addData("transit", transit.getCurrentPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("drone", drone.getPosition());
            telemetry.update();
        }
    }
//    public void resetLiftEncoders() {
//        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftLeft.setPower(1);
//        liftLeft.setPower(1);
//        while(opModeIsActive() && liftLeft.isBusy() && liftRight.isBusy()) { }
//        liftLeft.setPower(0);
//        liftRight.setPower(0);
//        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
}