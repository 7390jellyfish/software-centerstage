package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class CloseRedAutonJelly extends LinearOpMode {
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private DcMotor intake = null;
    private DcMotor transit = null;
    private Servo claw = null;
    private Servo drone = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     driveCountsPerMotorRev    = 8192 ;
    static final double     driveGearReduction    = 1.0 ;
    static final double     wheelDiameterInches   = 1.375 ;
    static final double     driveCountsPerInch         = (driveCountsPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.1415);
    static final double     driveSpeed             = 0.3;
    static final double     turnSpeed              = 0.5;
    static final double     liftCountsPerMotorRev    = 28 ;
    static final double     liftGearReduction    = 0.05 ;
    static final double     spoolDiameterInches   = 3 ;
    static final double     liftCountsPerInch         = (liftCountsPerMotorRev * liftGearReduction) / (spoolDiameterInches * 3.1415);
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

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d", backRightMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(driveSpeed,  -24,  -24, 5.0);
        outtakePixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(leftInches * driveCountsPerInch);
            newLeftTarget = backLeftMotor.getCurrentPosition() + (int)(leftInches * driveCountsPerInch);
            newRightTarget = frontRightMotor.getCurrentPosition() + (int)(rightInches * driveCountsPerInch);
            newRightTarget = backRightMotor.getCurrentPosition() + (int)(rightInches * driveCountsPerInch);
            frontLeftMotor.setTargetPosition(newLeftTarget);
            backLeftMotor.setTargetPosition(newLeftTarget);
            frontRightMotor.setTargetPosition(newRightTarget);
            backRightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", frontLeftMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void outtakePixel() {
        intake.setPower(-1);
        transit.setPower(-1);
        sleep(20000);
        intake.setPower(0);
        transit.setPower(0);
    }

    public void moveLift() {
        liftLeft.setPower(1);
        liftRight.setPower(1);
        sleep(1000);
        liftLeft.setPower(0);
        liftRight.setPower(0);
    }

    public void depositPixel() {
        moveLift();
        claw.setPosition(180);
        sleep(1000);
        claw.setPosition(0);
        moveLift();
    }
}