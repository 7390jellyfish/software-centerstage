package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOP extends LinearOpMode {

    // lift vars
    private DcMotor leftLift = null;

    private double liftMax = 400;

    private double liftPower = 1;
    private double countsPerInch;
    public double inchToTick (double inches) {
        return inches/0.02921868162;
    }

    // lift func
    public void lift (double ticks) {

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setTargetPosition((int) ticks);
        leftLift.setPower(liftPower);


//        while (leftLift.isBusy())
//        {
//            telemetry.addData("Left is lifting", leftLift.isBusy());
//            telemetry.update();
//        }
    }

    // Declare our motors
    // Make sure your ID's match your configuration
//    Servo claw = hardwareMap.get(Servo.class, "Release Servo");

    boolean clawDone = true;
    boolean clawClosed = false;
    boolean wristExtended;
    double CLAW_HOLD = 0.75;
    double CLAW_OPEN = 0.9;
    double backstageAngle;
    double rampPos = 0.45;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor transfer = hardwareMap.dcMotor.get("transit");
        Servo claw = hardwareMap.servo.get("claw");
        Servo ramp = hardwareMap.servo.get("ramp");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        // lift motors
        leftLift = hardwareMap.get(DcMotor.class, "lift");


        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        ramp.setPosition(rampPos);

        waitForStart();
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        System.out.println("Set reset motor");

        if (isStopRequested()) return;
        double claws = 0;

        while (opModeIsActive()) {
            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad2.right_stick_x;

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


            double liftpower = gamepad2.right_trigger - gamepad2.left_trigger;
            leftLift.setPower(liftpower);
            telemetry.addData("Liftpower", liftpower);
            telemetry.addData("Liftpos", leftLift.getCurrentPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("Ramo", ramp.getPosition());
            telemetry.addData("front right", frontRightMotor.getCurrentPosition());
            telemetry.addData("front left", frontLeftMotor.getCurrentPosition());
            telemetry.addData("back right", backRightMotor.getCurrentPosition());
            telemetry.addData("back left", backLeftMotor.getCurrentPosition());
            telemetry.update();

            if(Math.abs(leftLift.getCurrentPosition()-leftLift.getTargetPosition())<15){
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                System.out.println("MET REQS!");
            }

//            if (gamepad2.dpad_left) {
//                lift(700);
//            } else if (gamepad2.dpad_up) {
//                lift(1420);
//            } else if (gamepad2.dpad_right) {
//                lift(350);
//            } else if (gamepad2.dpad_down) {
//                lift(20);
//            }

//            while (gamepad2.left_trigger && leftLift.getCurrentPosition() <= liftMax) {
//                leftLift.setPower(lifttower);
//            }
//            while (gamepad2.right_trigger && leftLift.getCurrentPosition() >= liftMax) {
//                leftLift.setPower(lifttower);
//            }

            if (gamepad2.right_bumper){
                transfer.setPower(1);
                intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                transfer.setPower(1);
                intake.setPower(-1);
            } else {
                transfer.setPower(0);
                intake.setPower(0);
            }
            if (gamepad2.x){
                claw.setPosition(0.45);
                System.out.println("Triggered.");
            } else if(gamepad2.y){
                System.out.println("triggered y");
                claw.setPosition(0.6);
            }
//            if (gamepad2.dpad_up) {
//                if (rampPos <= 0.6) {
//                    rampPos += 0.1;
//                    ramp.setPosition(rampPos);
////                  System.out.println("triggered y");
//                }
//            } else if (gamepad2.dpad_down) {
//                if (rampPos >= 0.45) {
//                    rampPos -= 0.1;
//                    ramp.setPosition(rampPos);
////                  System.out.println("triggered y");
//                }
//            }
            if (gamepad2.dpad_up) {
                ramp.setPosition(ramp.getPosition()-0.1);
            } else if (gamepad2.dpad_down) {
                ramp.setPosition(ramp.getPosition()+0.1);
            }

//            // Closing the claw
//            if (gamepad2.a && clawDone == true && clawClosed == false) {
//                clawDone = false;
//                claw.setPosition(CLAW_HOLD);
//                clawDone = true;
//            }
//            // Opening the claw
//            else if (gamepad2.a && clawDone == true && clawClosed == true) {
//                clawDone = false;
//                claw.setPosition(CLAW_OPEN);
//                clawDone = true;
//            }
        }
    }
}