package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MaxwellAutonomous")
public class MaxwellAutonomous extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    static final double     COUNTS_PER_MOTOR_REV    = 529 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 16/20 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double track_width = 12;

    BNO055IMU imu;

    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftFront = hardwareMap.get(DcMotor.class, "fl");
        leftBack = hardwareMap.get(DcMotor.class, "bl");
        rightFront = hardwareMap.get(DcMotor.class, "fr");
        rightBack = hardwareMap.get(DcMotor.class, "br");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        move(DRIVE_SPEED, DRIVE_SPEED, 24, 24);
        turn(-90, DRIVE_SPEED);
        move(DRIVE_SPEED, DRIVE_SPEED, 24, 24);
        turn(90, DRIVE_SPEED);
        move(DRIVE_SPEED, DRIVE_SPEED, 24, 24);
        turn(90, DRIVE_SPEED);
        move(DRIVE_SPEED, DRIVE_SPEED, 24, 24);
        turn(-90, DRIVE_SPEED);
        move(DRIVE_SPEED, DRIVE_SPEED, 24, 24);
        turn(-90, DRIVE_SPEED);
        move(DRIVE_SPEED, DRIVE_SPEED, 24, 24);
    }

    public void move(double leftPower, double rightPower, double leftInches, double rightInches) {
        int leftTicks =(int)(leftInches * COUNTS_PER_INCH);
        int rightTicks =(int)(rightInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + leftTicks);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftTicks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rightTicks);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rightTicks);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(leftPower));
        leftBack.setPower(Math.abs(leftPower));
        rightFront.setPower(Math.abs(rightPower));
        rightBack.setPower(Math.abs(rightPower));

        while(leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy()) {

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(250);
    }

    public void turn(double turn, double power) {
        while(Math.abs(turn - imu.getAngularOrientation().firstAngle*(180/3.14)) > 3) {
            if(turn - imu.getAngularOrientation().firstAngle*(180/3.14) < 0) {
                leftFront.setPower(-power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
                leftBack.setPower(power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
                rightFront.setPower(power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
                rightBack.setPower(-power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
            } else {
                leftFront.setPower(power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
                leftBack.setPower(-power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
                rightFront.setPower(-power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
                rightBack.setPower(power*Math.abs(((turn - imu.getAngularOrientation().firstAngle*(180/3.14)))/10)+.15);
            }
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(250);
    }
}
