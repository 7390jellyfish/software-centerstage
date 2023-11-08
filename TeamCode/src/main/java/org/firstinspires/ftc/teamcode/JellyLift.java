package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="JellyLift", group="Linear Opmode")
public class JellyLift  extends LinearOpMode {
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;


    private double liftMax;

    private double liftPower = 0.5;
    private double countsPerInch;
    public double inchToTick (double inches) {
        return inches/0.02921868162;
    }

    public void lift (double inches) {
        int ticks = (int)(inchToTick(inches));

        leftLift.setTargetPosition(ticks);
        leftLift.setPower(liftPower);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setTargetPosition(ticks);
        rightLift.setPower(liftPower);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (rightLift.isBusy() && leftLift.isBusy())
        {
            telemetry.addData("Left is lifting", leftLift.isBusy());
            telemetry.addData("Right is lifting", rightLift.isBusy());
            telemetry.update();
        }
    }

    public void runOpMode() {

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_left) {
                lift(12);
            } else if (gamepad1.dpad_up) {
                lift(8);
            } else if (gamepad1.dpad_right) {
                lift(4);
            } else if (gamepad1.dpad_down) {
                lift(0);
            }
            while (gamepad1.left_bumper && leftLift.getCurrentPosition() <= liftMax && rightLift.getCurrentPosition() <= liftMax) {
                leftLift.setPower(liftPower);
                rightLift.setPower(liftPower);
            }
            while (gamepad1.right_bumper && leftLift.getCurrentPosition() >= liftMax && rightLift.getCurrentPosition() >= liftMax) {
                leftLift.setPower(-liftPower);
                rightLift.setPower(-liftPower);
            }
        }
    }
}