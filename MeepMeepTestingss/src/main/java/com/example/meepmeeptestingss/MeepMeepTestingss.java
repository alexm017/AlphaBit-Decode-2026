package com.example.meepmeeptestingss;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTestingss {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose_RedBasket = new Pose2d(-57, 45, Math.toRadians(126.5));
        Pose2d startPose_BlueBasket = new Pose2d(-57, -43, Math.toRadians(-126.5));
        Pose2d startPose_RedAudience = new Pose2d(60.5, 20, Math.toRadians(90));
        Pose2d startPose_BlueAudience = new Pose2d(60.5, -20, Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose_RedAudience)
                        .lineToLinearHeading(new Pose2d(36,25, Math.toRadians(90)))
                        .lineTo(new Vector2d(36,45))
                        .lineTo(new Vector2d(59,20))
                        .lineTo(new Vector2d(59,55))
                        .lineTo(new Vector2d(60.2,59))
                        .lineTo(new Vector2d(59,55))
                        .lineTo(new Vector2d(59,20))

                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("D:\\AlphaBit\\2026\\MeepMeepBackgrounds\\field-2025-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}