package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VarStorage {
    public static int autonomous_case = 0;
    public static int artifacts_pattern = 0;

    public static double leftTurret_initPosition = 0.5;
    public static double rightTurret_initPosition = 0.5;
    public static double angleTurret_initPosition = 0.8;

    public static double min_leftturret_position = 0.0;
    public static double min_rightturret_position = 0.0;
    public static double min_angleturret_position = 0.8;

    public static double max_leftturret_position = 1.0;
    public static double max_rightturret_position = 1.0;
    public static double max_angleturret_position = 0.15;

    public static double artifact_block_position = 0.9;
    public static double artifact_unblock_position = 1.0;

    public static double pushArtifact_push_position = 0.55;
    public static double pushArtifact_retract_position = 0.8;

    public static double x_red_basket_angleTurret = -50.0;
    public static double x_blue_basket_angleTurret = -50.0;
    public static double y_red_basket_angleTurret = 48.0;
    public static double y_blue_basket_angleTurret = -48.0;

    public static double x_apriltag_position = -58.0;
    public static double red_y_apriltag_position = 55.0;
    public static double blue_y_apriltag_position = -55.0;

    public static double marginThreshold = 11.0;
    public static double turretServoPosToDegree = 1.0/300.0;
    public static double max_TurretAngleAuto = 0.8;
    public static double min_TurretAngleAuto = 0.25;
    public static double max_TurretAngleDistance = 140.0;
    public static double min_FlyWheelPower = 0.46;
    public static double max_FlyWheelPower = 1.0;
    public static double max_FlyWheelDistance = 95.0;
    public static double horizontalTurretDeadzone = 0.0015;
    public static double verticalTurretDeadzone = 0.0015;
    public static double targetFlyWheelSpeed = 1800.0;
    public static double intakeRunTime = 350.0;
    public static double intakeMaxIdleRunTime = 1000.0;
    public static double timeoutTime = 2000.0;
    public static double intakeRunTimeManual = 350.0;

    public static double angleTurretSafePosition = 0.4;
    public static double leftTurretSafePosition = 0.5;
    public static double rightTurretSafePosition = 0.5;
    public static double defaultFlyWheelSafePower = 0.65;
    public static double rightDirectionAutoTurretOffset = 0.035;
    public static double rightDirectionManualTurretOffset = 0.015;
    public static double leftDirectionAutoTurretOffset = 0.000;
    public static double reductionPercentage = 0.92;
    public static double pushBackTime = 70.0;
}
