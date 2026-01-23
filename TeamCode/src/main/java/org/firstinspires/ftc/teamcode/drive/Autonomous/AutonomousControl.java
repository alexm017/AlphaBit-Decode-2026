package org.firstinspires.ftc.teamcode.drive.Autonomous;

import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Audience_firstAngle;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Audience_secondAngle;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_cyclePath_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_cyclePath_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_getArtifact_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_getArtifact_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_getCornerArtifact_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_getCornerArtifact_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_mainShooting_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_mainShooting_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_preparePickup_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BA_preparePickup_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_getFirstPattern_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_getFirstPattern_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_getMiddlePattern_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_getMiddlePattern_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_lastShooting_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_lastShooting_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_mainShooting_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_mainShooting_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_openTunnel_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_openTunnel_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_returnTunnel_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_returnTunnel_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_startPickupMiddlePattern_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.BB_startPickupMiddlePattern_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Basket_firstAngle;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Basket_secondAngle;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_cyclePath_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_cyclePath_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_getArtifact_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_getArtifact_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_getCornerArtifact_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_getCornerArtifact_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_mainShooting_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_mainShooting_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_preparePickup_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RA_preparePickup_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_getFirstPattern_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_getFirstPattern_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_getMiddlePattern_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_getMiddlePattern_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_lastShooting_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_lastShooting_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_mainShooting_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_mainShooting_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_openTunnel_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_openTunnel_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_returnTunnel_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_returnTunnel_Y;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_startPickupMiddlePattern_X;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.RB_startPickupMiddlePattern_Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.ComputerVision.AprilTagIdentification;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;

//FTC Decode 2026 AutonomousControl
@Autonomous
public class AutonomousControl extends LinearOpMode {
    SampleMecanumDrive drive;
    MultipleTelemetry multipleTelemetry;
    ArtifactControl artifactControl;
    TrajectorySequence trajectoryRedBasket, trajectoryBlueBasket, trajectoryRedAudience, trajectoryBlueAudience;

    Pose2d startPose_RedBasket = new Pose2d(-57, 43, Math.toRadians(126.5));
    Pose2d startPose_BlueBasket = new Pose2d(-57, -43, Math.toRadians(-126.5));
    Pose2d startPose_RedAudience = new Pose2d(60.5, 11, Math.toRadians(90));
    Pose2d startPose_BlueAudience = new Pose2d(60.5, -11, Math.toRadians(-90));

    boolean blueAlliance = false;
    boolean nearBasket = false;
    boolean firstButtonTrigger = false;
    boolean secondButtonTrigger = false;
    boolean patternFound = false;
    int autoCase = 0;
    int currentId = 0;

    enum ObeliskPattern{
        UNKNOWN,
        GPP,
        PGP,
        PPG
    }

    ObeliskPattern currentPattern = ObeliskPattern.UNKNOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        artifactControl = new ArtifactControl(hardwareMap, gamepad2, multipleTelemetry);

        artifactControl.initServo();

        trajectoryRedBasket = drive.trajectorySequenceBuilder(startPose_RedBasket)
                .lineToLinearHeading(new Pose2d(RB_mainShooting_X, RB_mainShooting_Y, Math.toRadians(90)))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Basket_firstAngle, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .lineTo(new Vector2d(RB_startPickupMiddlePattern_X,RB_startPickupMiddlePattern_Y))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(RB_getMiddlePattern_X,RB_getMiddlePattern_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(RB_openTunnel_X,RB_openTunnel_Y))
                .lineTo(new Vector2d(RB_returnTunnel_X,RB_returnTunnel_Y))
                .lineTo(new Vector2d(RB_mainShooting_X,RB_mainShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Basket_firstAngle, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(RB_getFirstPattern_X,RB_getFirstPattern_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(RB_lastShooting_X,RB_lastShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Basket_secondAngle, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .build();

        trajectoryBlueBasket = drive.trajectorySequenceBuilder(startPose_BlueBasket)
                .lineToLinearHeading(new Pose2d(BB_mainShooting_X, BB_mainShooting_Y, Math.toRadians(-90)))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Basket_firstAngle, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .lineTo(new Vector2d(BB_startPickupMiddlePattern_X,BB_startPickupMiddlePattern_Y))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(BB_getMiddlePattern_X,BB_getMiddlePattern_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(BB_openTunnel_X,BB_openTunnel_Y))
                .lineTo(new Vector2d(BB_returnTunnel_X,BB_returnTunnel_Y))
                .lineTo(new Vector2d(BB_mainShooting_X,BB_mainShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Basket_firstAngle, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(BB_getFirstPattern_X,BB_getFirstPattern_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(BB_lastShooting_X,BB_lastShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Basket_secondAngle, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .build();

        trajectoryBlueAudience = drive.trajectorySequenceBuilder(startPose_BlueAudience)
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Audience_firstAngle, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .lineToLinearHeading(new Pose2d(BA_preparePickup_X, BA_preparePickup_Y, Math.toRadians(-90)))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(BA_getArtifact_X, BA_getArtifact_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(BA_mainShooting_X, BA_mainShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Audience_secondAngle, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(BA_cyclePath_X, BA_cyclePath_Y))
                .lineTo(new Vector2d(BA_getCornerArtifact_X, BA_getCornerArtifact_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(BA_cyclePath_X, BA_cyclePath_Y))
                .lineTo(new Vector2d(BA_mainShooting_X, BA_mainShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Audience_secondAngle, false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), false))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .build();

        trajectoryRedAudience = drive.trajectorySequenceBuilder(startPose_RedAudience)
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Audience_firstAngle, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .lineToLinearHeading(new Pose2d(RA_preparePickup_X, RA_preparePickup_Y, Math.toRadians(90)))
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(RA_getArtifact_X, RA_getArtifact_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(RA_mainShooting_X, RA_mainShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Audience_secondAngle, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .addTemporalMarker(() -> artifactControl.getArtifacts())
                .lineTo(new Vector2d(RA_cyclePath_X, RA_cyclePath_Y))
                .lineTo(new Vector2d(RA_getCornerArtifact_X, RA_getCornerArtifact_Y))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.stopIntakeOuttake())
                .lineTo(new Vector2d(RA_cyclePath_X, RA_cyclePath_Y))
                .lineTo(new Vector2d(RA_mainShooting_X, RA_mainShooting_Y))
                .addTemporalMarker(() -> artifactControl.setAutonomousShooter(Audience_secondAngle, true, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), true))
                .waitSeconds(2)
                .addTemporalMarker(() -> artifactControl.setAutonomousThrowFlags())
                .waitSeconds(8)
                .addTemporalMarker(() -> artifactControl.setAutonomousResetFlags())
                .build();

        artifactControl.manualControl = false;

        while(opModeInInit()) {
            if (gamepad1.dpad_left) {
                if (!firstButtonTrigger) {
                    if(!blueAlliance) {
                        autoCase = autoCase + 1;
                        blueAlliance = true;
                    }else{
                        autoCase = autoCase - 1;
                        blueAlliance = false;
                    }
                    firstButtonTrigger = true;
                }
            }else{
                firstButtonTrigger = false;
            }

            if (gamepad1.dpad_up) {
                if (!secondButtonTrigger) {
                    if(!nearBasket){
                        autoCase = autoCase + 2;
                        nearBasket = true;
                    }else{
                        autoCase = autoCase - 2;
                        nearBasket = false;
                    }
                    secondButtonTrigger = true;
                }
            }else{
                secondButtonTrigger = false;
            }

            if(!nearBasket){
                currentId = artifactControl.getCurrentTag();

                if(currentId != 0){
                    switch(currentId){
                        case 21:
                            currentPattern = ObeliskPattern.GPP;
                            break;
                        case 22:
                            currentPattern = ObeliskPattern.PGP;
                            break;
                        case 23:
                            currentPattern = ObeliskPattern.PPG;
                            break;
                    }
                    patternFound = true;
                }
            }

            multipleTelemetry.addData("[->] Pattern ", currentPattern);

            multipleTelemetry.addData("[->] Case ", autoCase);
            multipleTelemetry.update();
        }

        waitForStart();

        VarStorage.autonomous_case = autoCase;
        if(!nearBasket && patternFound){
            VarStorage.artifacts_pattern = currentId;
        }

        switch(autoCase){
            case 0:
                drive.setPoseEstimate(startPose_RedAudience);
                drive.followTrajectorySequenceAsync(trajectoryRedAudience);
                break;
            case 1:
                drive.setPoseEstimate(startPose_BlueAudience);
                drive.followTrajectorySequenceAsync(trajectoryBlueAudience);
                break;
            case 2:
                drive.setPoseEstimate(startPose_RedBasket);
                drive.followTrajectorySequenceAsync(trajectoryRedBasket);
                break;
            case 3:
                drive.setPoseEstimate(startPose_BlueBasket);
                drive.followTrajectorySequenceAsync(trajectoryBlueBasket);
                break;
        }

        while(opModeIsActive()) {
            drive.update();

            if(artifactControl.wantsToThrowArtifacts) {
                artifactControl.throwArtifacts(0, false, true);
            }

            if(!patternFound){
                currentId = artifactControl.getCurrentTag();

                if(currentId != 0){
                    switch(currentId){
                        case 21:
                            currentPattern = ObeliskPattern.GPP;
                            break;
                        case 22:
                            currentPattern = ObeliskPattern.PGP;
                            break;
                        case 23:
                            currentPattern = ObeliskPattern.PPG;
                            break;
                    }
                    patternFound = true;
                    VarStorage.artifacts_pattern = currentId;
                }
            }

            multipleTelemetry.addData("[->] X ",  drive.getPoseEstimate().getX());
            multipleTelemetry.addData("[->] Y ", drive.getPoseEstimate().getY());
            multipleTelemetry.addData("[->] HDG ", drive.getPoseEstimate().getHeading());
            multipleTelemetry.addData("[->] Pattern ", currentPattern);
            multipleTelemetry.update();
        }
    }
}
