package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class FINALRedAutonJelly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(40)
                .back(37)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(34)
                .back(37)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(28)
                .back(37)
                .build();

        waitForStart();

        if (!isStopRequested()) {
//            if (spikeMark() == 1) {
//                drive.followTrajectorySequence(left);
//            } else if (spikeMark() == 2) {
//                drive.followTrajectorySequence(middle);
//            } else if (spikeMark() == 3) {
//                drive.followTrajectorySequence(right);
//            }
            drive.followTrajectorySequence(left);
        }
    }
}