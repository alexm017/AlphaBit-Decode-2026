package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VarStorage {
    public static int autonomous_case = 0;
    public static int artifacts_pattern = 0;

    public static double leftTurret_initPosition = 0.5;
    public static double rightTurret_initPosition = 0.5;
    public static double angleTurret_initPosition = 0.9;
    public static double angleTurret_manualPosition = 0.7;

    public static double min_leftturret_position = 1.0;
    public static double min_rightturret_position = 1.0;
    public static double min_angleturret_position = 0.95;

    public static double max_leftturret_position = 0.0;
    public static double max_rightturret_position = 0.0;
    public static double max_angleturret_position = 0.15;

    public static double artifact_block_position = 0.9;
    public static double artifact_unblock_position = 1.0;

    public static double x_red_basket_angleTurret = -50.0;
    public static double x_blue_basket_angleTurret = -50.0;
    public static double y_red_basket_angleTurret = 48.0;
    public static double y_blue_basket_angleTurret = -48.0;

    public static double x_apriltag_position = -58.0;
    public static double red_y_apriltag_position = 55.0;
    public static double blue_y_apriltag_position = -55.0;

    public static double marginThreshold = 10.0;
    public static double turretServoPosToDegree = 1.0/300.0;
    public static double max_TurretAngleDistance = 130;
    public static double min_FlyWheelPower = 0.99;
    public static double max_FlyWheelPower = 1.0;
    public static double max_FlyWheelDistance = 130;
    public static double horizontalTurretDeadzone = 0.005;
    public static double verticalTurretDeadzone = 0.003;
    public static double targetFlyWheelSpeed = 1600.0;
    public static double intakeRunTime = 100.0;
}
