package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class Testauton extends LinearOpMode {
    private DcMotor motors = null;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        motors = hardwareMap.get(DcMotor.class, "ld");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            telemetry.addData("Position", motors.getCurrentPosition());
            motors.setTargetPosition(4000);
            motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors.setPower(0.5);
            telemetry.update();

        }

    }


}
