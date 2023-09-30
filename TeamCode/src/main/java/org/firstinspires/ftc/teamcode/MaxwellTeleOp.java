package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MaxwellTeleOp")
public class MaxwellTeleOp extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "fl");
        leftBack = hardwareMap.get(DcMotor.class, "bl");
        rightFront = hardwareMap.get(DcMotor.class, "fr");
        rightBack = hardwareMap.get(DcMotor.class, "br");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
// adb connect 192.168.43.1:5555
        waitForStart();

        while (opModeIsActive()) {
            double vertical   = -gamepad1.left_stick_y;
            double turn     =  gamepad1.right_stick_x;

            double leftFrontPower  = vertical + turn;
            double rightFrontPower = vertical - turn;
            double leftBackPower   = vertical + -turn;
            double rightBackPower  = vertical - -turn;

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }
    }
}