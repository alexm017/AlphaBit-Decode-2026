package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

//FTC Decode 2026 AutonomousControl
@Autonomous
public class AutonomousControl extends LinearOpMode {
    SampleMecanumDrive drive;
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

        drive.setPoseEstimate(startPose_RedBasket);

        trajectoryRedBasket = drive.trajectorySequenceBuilder(startPose_RedBasket)
                .lineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(90)))
                .lineTo(new Vector2d(12,25))
                .lineTo(new Vector2d(12,45))
                .lineTo(new Vector2d(4,52))
                .lineTo(new Vector2d(4,25))
                .lineTo(new Vector2d(-12,15))
                .lineTo(new Vector2d(-12,45))
                .lineTo(new Vector2d(-20,25))
                .build();

        trajectoryBlueBasket = drive.trajectorySequenceBuilder(startPose_BlueBasket)
                .lineToLinearHeading(new Pose2d(-12, -15, Math.toRadians(-90)))
                .lineTo(new Vector2d(12,-25))
                .lineTo(new Vector2d(12,-45))
                .lineTo(new Vector2d(4,-52))
                .lineTo(new Vector2d(4,-25))
                .lineTo(new Vector2d(-12,-15))
                .lineTo(new Vector2d(-12,-45))
                .lineTo(new Vector2d(-20,-25))
                .build();

        trajectoryBlueAudience = drive.trajectorySequenceBuilder(startPose_BlueAudience)
                .lineToLinearHeading(new Pose2d(36,-25, Math.toRadians(-90)))
                .lineTo(new Vector2d(36,-45))
                .lineTo(new Vector2d(59,-20))
                .lineTo(new Vector2d(59,-53))
                .lineTo(new Vector2d(60.2,-57))
                .lineTo(new Vector2d(59,-53))
                .lineTo(new Vector2d(59,-20))
                .build();

        trajectoryRedAudience = drive.trajectorySequenceBuilder(startPose_RedAudience)
                .lineToLinearHeading(new Pose2d(36,25, Math.toRadians(90)))
                .lineTo(new Vector2d(36,45))
                .lineTo(new Vector2d(59,20))
                .lineTo(new Vector2d(59,55))
                .lineTo(new Vector2d(60.2,59))
                .lineTo(new Vector2d(59,55))
                .lineTo(new Vector2d(59,20))
                .build();

        if(gamepad1.dpad_left){
            if(!firstButtonTrigger) {
                blueAlliance = true;
                autoCase = autoCase + 1;
                firstButtonTrigger = true;
            }
        }else if(gamepad1.dpad_up){
            if(!secondButtonTrigger) {
                nearBasket = true;
                autoCase = autoCase + 2;
                secondButtonTrigger = true;
            }
        }

        waitForStart();

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
        }
    }
}
