package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VarStorage {
    public static int autonomous_case = 0;
    public static int artifacts_pattern = 0;

    public static int colorSensor_gain = 4;

    public static double leftTurret_initPosition = 0.68;
    public static double rightTurret_initPosition = 0.68;
    public static double angleTurret_initPosition = 0.45;

    public static double min_leftturret_position = 0.05;
    public static double min_rightturret_position = 0.05;
    public static double min_angleturret_position = 0.8;

    public static double max_leftturret_position = 0.875;
    public static double max_rightturret_position = 0.875;
    public static double max_angleturret_position = 0.15;

    public static double artifact_block_position = 0.15;
    public static double artifact_unblock_position = 0.02;

    public static double pushArtifact_push_position = 0.55;
    public static double pushArtifact_retract_position = 0.8;

    public static double x_red_basket_angleTurret = -50.0;
    public static double x_blue_basket_angleTurret = -50.0;
    public static double y_red_basket_angleTurret = 48.0;
    public static double y_blue_basket_angleTurret = -48.0;

    public static double marginThreshold = 10.5;
    public static double turretServoPosToDegree = 0.85/333.33;
    public static double max_TurretAngleAuto = 0.75;
    public static double min_TurretAngleAuto = 0.25;
    public static double max_TurretAngleDistance = 124.0;
    public static double min_FlyWheelPower = 0.6;
    public static double max_FlyWheelPower = 0.87;
    public static double max_FlyWheelDistance = 124.0;
    public static double horizontalTurretDeadzone = 0.0015;
    public static double verticalTurretDeadzone = 0.0015;
    public static double targetFlyWheelSpeed = 2400.0;
    public static double intakeRunTime = 300.0;
    public static double intakeFirstRunTime = 550.0;
    public static double intakeMaxIdleRunTime = 1000.0;
    public static double timeoutTime = 2000.0;
    public static double intakeRunTimeManual = 300.0;
    public static double intakeFirstRunTimeManual = 550.0;

    public static double angleTurretSafePosition = 0.45;
    public static double leftTurretSafePosition = 0.67;
    public static double rightTurretSafePosition = 0.67;
    public static double defaultFlyWheelSafePower = 0.80;
    public static double defaultFlyWheelPowerAuto = 0.86;
    public static double rightDirectionAutoTurretOffset = 0.0;
    public static double rightDirectionManualTurretOffset = 0.0;
    public static double leftDirectionAutoTurretOffset = 0.000;
    public static double robotVelocityThreshold = 2.5;
    public static double robotAngularVelocityThreshold = 0.15;
    public static double minimumBasketDistance = 0.0;
    public static double flyWheelAggressiveAcceleration = 0.15;
    public static double leftDistanceThreshold = 20.0;
    public static double rightDistanceThreshold = 20.0;
    public static double autoStartIntakeTempTimer = 1000.0;
    public static double lightIntensityThreshold = 0.7;
}
