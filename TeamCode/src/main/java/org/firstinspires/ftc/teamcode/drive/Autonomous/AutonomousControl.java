package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;

//FTC Decode 2026 AutonomousControl
@Autonomous
public class AutonomousControl extends LinearOpMode {
    SampleMecanumDrive drive;
    MultipleTelemetry multipleTelemetry;
    ArtifactControl artifactControl;
    TrajectorySequence trajectoryRedBasket, trajectoryBlueBasket, trajectoryRedAudience, trajectoryBlueAudience;

    Pose2d startPose_RedBasket = new Pose2d(-57, 45, Math.toRadians(126.5));
    Pose2d startPose_BlueBasket = new Pose2d(-57, -43, Math.toRadians(-126.5));
    Pose2d startPose_RedAudience = new Pose2d(60.5, 20, Math.toRadians(90));
    Pose2d startPose_BlueAudience = new Pose2d(60.5, -20, Math.toRadians(-90));

    boolean blueAlliance = false;
    boolean nearBasket = false;
    int autoCase = 0;
    boolean firstButtonTrigger = false;
    boolean secondButtonTrigger = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        multipleTelemetry = new MultipleTelemetry();
        artifactControl = new ArtifactControl(hardwareMap, gamepad2, multipleTelemetry, 0);

        drive.setPoseEstimate(startPose_RedBasket);

        trajectoryRedBasket = drive.trajectorySequenceBuilder(startPose_RedBasket)
                .lineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(90)))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(50, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(12,25))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(12,45))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(4,52))
                .lineTo(new Vector2d(4,25))
                .lineTo(new Vector2d(-12,15))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(50, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(-12,45))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(-20,25))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(53, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .build();

        trajectoryBlueBasket = drive.trajectorySequenceBuilder(startPose_BlueBasket)
                .lineToLinearHeading(new Pose2d(-12, -15, Math.toRadians(-90)))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(50, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(12,-25))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(12,-45))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(4,-52))
                .lineTo(new Vector2d(4,-25))
                .lineTo(new Vector2d(-12,-15))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(50, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(-12,-45))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(-20,-25))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(53, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .build();

        trajectoryBlueAudience = drive.trajectorySequenceBuilder(startPose_BlueAudience)
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(71.5, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineToLinearHeading(new Pose2d(36,-25, Math.toRadians(-90)))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(36,-45))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(59,-20))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(71.5, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(59,-53))
                .lineTo(new Vector2d(60.2,-57))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(59,-53))
                .lineTo(new Vector2d(59,-20))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(71.5, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .build();

        trajectoryRedAudience = drive.trajectorySequenceBuilder(startPose_RedAudience)
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(71.5, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineToLinearHeading(new Pose2d(36,25, Math.toRadians(90)))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(36,45))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(59,20))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(71.5, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(59,55))
                .lineTo(new Vector2d(60.2,59))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(59,55))
                .lineTo(new Vector2d(59,20))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(71.5, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(5)
                .addTemporalMarker(() -> artifactControl.throwArtifacts())
                .waitSeconds(10)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .build();

        while(opModeInInit()) {
            if (gamepad1.dpad_left) {
                if (!firstButtonTrigger) {
                    if(!blueAlliance) {
                        autoCase = autoCase + 1;
                    }else{
                        autoCase = autoCase - 1;
                    }
                    firstButtonTrigger = true;
                }
            } else if (gamepad1.dpad_up) {
                if (!secondButtonTrigger) {
                    if(!nearBasket){
                        autoCase = autoCase + 2;
                    }else{
                        autoCase = autoCase - 2;
                    }
                    secondButtonTrigger = true;
                }
            }else{
                secondButtonTrigger = false;
            }

            telemetry.addData("[->] Case ", autoCase);
            telemetry.update();
        }

        waitForStart();

        VarStorage.autonomous_case = autoCase;

        switch(autoCase){
            case 0:
                drive.followTrajectorySequenceAsync(trajectoryRedAudience);
                break;
            case 1:
                drive.followTrajectorySequenceAsync(trajectoryBlueAudience);
                break;
            case 2:
                drive.followTrajectorySequenceAsync(trajectoryRedBasket);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(trajectoryBlueBasket);
                break;
        }

        while(opModeIsActive()) {
            drive.update();
            telemetry.addData("[->] X ",  drive.getPoseEstimate().getX());
            telemetry.addData("[->] Y ", drive.getPoseEstimate().getY());
            telemetry.addData("[->] HDG ", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
