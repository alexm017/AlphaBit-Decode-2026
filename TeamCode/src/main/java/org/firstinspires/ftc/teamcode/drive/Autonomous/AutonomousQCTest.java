package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

//FTC Decode 2026 AutonomousQCTest
@Autonomous
public class AutonomousQCTest extends LinearOpMode {
    SampleMecanumDrive drive;
    TrajectorySequence trajectory;

    Pose2d startPose = new Pose2d(60.5, 20, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        trajectory = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .turn(Math.toRadians(90))
                .forward(15)
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(trajectory);

        while(opModeIsActive()) {
            drive.update();
        }
    }
}
