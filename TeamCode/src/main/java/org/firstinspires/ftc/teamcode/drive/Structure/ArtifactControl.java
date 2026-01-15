package org.firstinspires.ftc.teamcode.drive.Structure;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ComputerVision.AprilTagIdentification;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.GyroscopeBHIMU;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;

public class ArtifactControl {
    Gamepad gamepad2;
    AprilTagIdentification aprilTagIdentification = new AprilTagIdentification();
    MultipleTelemetry telemetry;
    GyroscopeBHIMU gyroscope = new GyroscopeBHIMU();
    SampleMecanumDrive drive;

    DcMotor Intake_LeftMotor;
    DcMotor Intake_RightMotor;
    DcMotor Outtake_LeftMotor;
    DcMotor Outtake_RightMotor;

    Servo LeftTurret;
    Servo RightTurret;
    Servo AngleTurret;
    Servo BlockArtifact;

    Pose2d endPose_RedBasket = new Pose2d(-20, 25, Math.toRadians(90));
    Pose2d endPose_BlueBasket = new Pose2d(-20, -25, Math.toRadians(-90));
    Pose2d endPose_RedAudience = new Pose2d(59, 20, Math.toRadians(90));
    Pose2d endPose_BlueAudience = new Pose2d(59, -20, Math.toRadians(-90));

    double targetAngle;
    public boolean isRedAlliance = false;

    public enum fieldPattern{
        UNKNOWN,
        GPP,
        PGP,
        PPG
    }

    public fieldPattern artifactPattern = fieldPattern.UNKNOWN;

    public ArtifactControl(HardwareMap hwdmap, Gamepad gmpd, MultipleTelemetry telemetrys){
        gamepad2 = gmpd;
        telemetry = telemetrys;
        aprilTagIdentification.init(hwdmap, telemetrys);
        gyroscope.gyroscope_init(hwdmap);

        if(VarStorage.artifacts_pattern != 0){
            switch(VarStorage.artifacts_pattern){
                case 21:
                    artifactPattern = fieldPattern.GPP;
                    break;
                case 22:
                    artifactPattern = fieldPattern.PGP;
                    break;
                case 23:
                    artifactPattern = fieldPattern.PPG;
                    break;
            }
        }

        drive = new SampleMecanumDrive(hwdmap);

        switch(VarStorage.autonomous_case){
            case 0:
                drive.setPoseEstimate(endPose_RedAudience);
                break;
            case 1:
                drive.setPoseEstimate(endPose_BlueAudience);
                break;
            case 2:
                drive.setPoseEstimate(endPose_RedBasket);
                break;
            case 3:
                drive.setPoseEstimate(endPose_BlueBasket);
                break;
        }

        if(VarStorage.autonomous_case == 0 || VarStorage.autonomous_case == 2){
            isRedAlliance = true;
        }

        Intake_LeftMotor = hwdmap.get(DcMotor.class, "Intake_LeftMotor");
        Intake_RightMotor = hwdmap.get(DcMotor.class, "Intake_RightMotor");

        Outtake_LeftMotor = hwdmap.get(DcMotor.class, "Outtake_LeftMotor");
        Outtake_RightMotor = hwdmap.get(DcMotor.class, "Outtake_RightMotor");

        Intake_LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake_RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake_LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake_RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftTurret = hwdmap.get(Servo.class,"LeftTurret");
        RightTurret = hwdmap.get(Servo.class,"RightTurret");

        AngleTurret = hwdmap.get(Servo.class,"AngleTurret");

        BlockArtifact = hwdmap.get(Servo.class,"BlockArtifact");
    }

    double leftTurret_initPosition = 0.5; // needs to be changed
    double rightTurret_initPosition = 0.5; // needs to be changed
    double angleTurret_initPosition = 0.9;

    double min_leftturret_position = 0;
    double min_rightturret_position = 0;
    double min_angleturret_position = 0.9;

    double max_leftturret_position = 1;
    double max_rightturret_position = 1;
    double max_angleturret_position = 0.2;

    public double current_rightturret_position= rightTurret_initPosition;
    public double current_leftturret_position = leftTurret_initPosition;
    public double current_angleturret_position = angleTurret_initPosition;

    public boolean artifact_status_blocked = false;
    double artifact_block_position = 0.9;
    double artifact_unblock_position = 1.0;

    public double headingAngle = 0.0;
    public double x_position = 0.0;
    public double y_position = 0.0;

    boolean toggleButton = false;
    boolean stoggleButton = false;
    boolean manualResetPoseToggle = false;

    double turretServoPosToDegree = 1.0/300.0; // needs to be changed
    public boolean allowedToShoot = false;
    boolean rotateToLeft = false;

    double x_red_basket_angleTurret = -50.0;
    double x_blue_basket_angleTurret = -50.0;
    double y_red_basket_angleTurret = 48.0;
    double y_blue_basket_angleTurret = -48.0;

    double x_apriltag_position = -58.0;
    double red_y_apriltag_position = 55.0;
    double blue_y_apriltag_position = -55.0;

    public boolean manualControl = false;
    boolean toggleS = false;
    boolean oneTimeRumble = false;

    public void initServo(){
        AngleTurret.setPosition(angleTurret_initPosition);
        LeftTurret.setPosition(leftTurret_initPosition);
        RightTurret.setPosition(rightTurret_initPosition);
    }

    public void updateAprilTag(){
        aprilTagIdentification.telemetryAprilTag();
    }

    public void Run(){
        updateAprilTag();
        drive.update();

        Pose2d robotPose = drive.getPoseEstimate();

        headingAngle = gyroscope.getHeading();
        x_position = robotPose.getX();
        y_position = robotPose.getY();

        if(!manualControl) {
            areaOfThrowing();
            updateShooter();
        }

        if(gamepad2.a){
            getArtifacts();
        }else if(gamepad2.y){
            if(allowedToShoot && !manualControl) {
                throwArtifacts(getFlyWheelPower(0,0,false,false), true);
            }else if(manualControl){
                throwArtifacts(0, false);
            }
        }

        if(gamepad2.b){
            stopIntakeOuttake();
        }

        if (gamepad2.left_bumper && current_leftturret_position > min_leftturret_position && current_rightturret_position > min_rightturret_position && manualControl) {
            if(!toggleButton) {
                current_leftturret_position = current_leftturret_position - 0.05;
                current_rightturret_position = current_rightturret_position - 0.05;
                LeftTurret.setPosition(current_leftturret_position);
                RightTurret.setPosition(current_rightturret_position);
                toggleButton = true;
            }
        }else if (gamepad2.right_bumper && current_leftturret_position < max_leftturret_position && current_rightturret_position < max_rightturret_position && manualControl) {
            if(!toggleButton) {
                current_leftturret_position = current_leftturret_position + 0.05;
                current_rightturret_position = current_rightturret_position + 0.05;
                LeftTurret.setPosition(current_leftturret_position);
                RightTurret.setPosition(current_rightturret_position);
                toggleButton = true;
            }
        }else{
            toggleButton = false;
        }

        if (gamepad2.dpad_up && current_angleturret_position > max_angleturret_position && manualControl) {
            if(!stoggleButton) {
                current_angleturret_position = current_angleturret_position - 0.05;
                AngleTurret.setPosition(current_angleturret_position);
                stoggleButton = true;
            }
        } else if (gamepad2.dpad_down && current_angleturret_position < min_angleturret_position && manualControl) {
            if(!stoggleButton) {
                current_angleturret_position = current_angleturret_position + 0.05;
                AngleTurret.setPosition(current_angleturret_position);
                stoggleButton = true;
            }
        }else{
            stoggleButton = false;
        }

        if(gamepad2.left_trigger > 0.75 && gamepad2.right_trigger > 0.75){
            if(!toggleS) {
                manualControl = !manualControl;
                toggleS = true;
            }
        }else{
            toggleS = false;
        }

        if(gamepad2.dpad_left && !manualResetPoseToggle && manualControl){
            manuallyResetPose();
            manualResetPoseToggle = true;
        }else{
            manualResetPoseToggle = false;
        }
    }

    public void getArtifacts(){
        BlockArtifact.setPosition(artifact_block_position);
        Intake_LeftMotor.setPower(1);
        Intake_RightMotor.setPower(1);
        artifact_status_blocked = true;
    }

    public void throwArtifacts(double customFlyWheelPower, boolean useCustomPower){
        BlockArtifact.setPosition(artifact_unblock_position);
        if(useCustomPower) {
            Outtake_LeftMotor.setPower(customFlyWheelPower);
            Outtake_RightMotor.setPower(customFlyWheelPower);
        }else{
            Outtake_LeftMotor.setPower(1);
            Outtake_RightMotor.setPower(1);
        }
        Intake_LeftMotor.setPower(1);
        Intake_RightMotor.setPower(1);
        artifact_status_blocked = false;
    }

    public void stopIntakeOuttake(){
        Intake_LeftMotor.setPower(0);
        Intake_RightMotor.setPower(0);
        Outtake_LeftMotor.setPower(0);
        Outtake_RightMotor.setPower(0);
    }

    public void areaOfThrowing(){
        double marginThreshold = 7;

        if(y_position >= 0 && x_position < (0 + marginThreshold)){
            if((Math.abs(x_position)+marginThreshold) > y_position){
                allowedToShoot = true;
            }else{
                allowedToShoot = false;
            }
        }else if(y_position <= 0 && x_position < (0 + marginThreshold)){
            if((Math.abs(x_position)+marginThreshold) > Math.abs(y_position)){
                allowedToShoot = true;
            }else{
                allowedToShoot = false;
            }
        }else if(x_position > (50-marginThreshold)){
            if(((x_position - 50) + marginThreshold) > Math.abs(y_position)){
                allowedToShoot = true;
            }else{
                allowedToShoot = false;
            }
        }else{
            allowedToShoot = false;
        }

        if(allowedToShoot && !oneTimeRumble){
            gamepad2.rumble(1000);
            oneTimeRumble = true;
        }else if(!allowedToShoot && oneTimeRumble){
            gamepad2.rumble(500);
            oneTimeRumble = false;
        }
    }

    public void dynamicTargetAngle(){
        double positive_x_position = x_position + 70;
        double positive_y_position;
        if(isRedAlliance){
            positive_y_position = Math.abs((y_position -70));
        }else{
            positive_y_position = Math.abs((y_position + 70));
        }

        double calculatedAngle = Math.abs(Math.toDegrees(Math.atan2(positive_x_position, positive_y_position)));

        if(isRedAlliance){
            targetAngle = calculatedAngle;
        }else{
            targetAngle = 360 - calculatedAngle;
        }
    }

    public double getBasketDirection(){
        double basketAngle;

        dynamicTargetAngle();

        if(targetAngle - headingAngle > 0){
            basketAngle = targetAngle - headingAngle;
            if(targetAngle - headingAngle >= 180){
                rotateToLeft = false;
            }else{
                rotateToLeft = true;
            }
        }else{
            basketAngle = 360 - Math.abs((headingAngle - targetAngle));
            if(basketAngle >= 180){
                rotateToLeft = false;
            }else{
                rotateToLeft = true;
            }
        }

        if(basketAngle >= 180){
            basketAngle = 360 - basketAngle;
        }

        return basketAngle;
    }

    public double getBasketDistance(double custom_x_pos, double custom_y_pos, boolean redAlliance, boolean useCustomPos){
        double distanceToBasket;
        if(!useCustomPos) {
            if (isRedAlliance) {
                distanceToBasket = Math.sqrt(Math.pow(x_red_basket_angleTurret - x_position, 2) + Math.pow(y_red_basket_angleTurret - y_position, 2));
            } else {
                distanceToBasket = Math.sqrt(Math.pow(x_blue_basket_angleTurret - x_position, 2) + Math.pow(y_blue_basket_angleTurret - y_position, 2));
            }
        }else{
            if(redAlliance){
                distanceToBasket = Math.sqrt(Math.pow(x_red_basket_angleTurret - custom_x_pos, 2) + Math.pow(y_red_basket_angleTurret - custom_y_pos, 2));
            }else{
                distanceToBasket = Math.sqrt(Math.pow(x_blue_basket_angleTurret - custom_x_pos, 2) + Math.pow(y_blue_basket_angleTurret - custom_y_pos, 2));
            }
        }
        return distanceToBasket;
    }

    public double getTurretPosition(){
        double servoAngleToPosition;
        servoAngleToPosition = getBasketDirection() * turretServoPosToDegree;

        return servoAngleToPosition;
    }

    public double getTurretAngle(){
        double angleToCm;
        double max_angle = 0.2;
        double min_angle = 0.9;
        double max_distance = 130;
        double anglePerInch = Math.abs(((max_angle-min_angle)/max_distance));
        angleToCm = getBasketDistance(0,0,false,false) * anglePerInch;

        if(angleToCm > 0.7){
            angleToCm = 0.7;
        }

        return angleToCm;
    }

    public double getFlyWheelPower(double custom_x_pos, double custom_y_pos, boolean redAlliance, boolean useCustomPos){
        double min_power = 0.5;
        double max_power = 1.0;
        double max_distance = 130;
        double powerPerInch = Math.abs((max_power-min_power)/max_distance);
        double distance;
        if(!useCustomPos) {
            distance = getBasketDistance(0, 0, false, false);
        }else{
            distance = getBasketDistance(custom_x_pos, custom_y_pos, redAlliance, true);
        }

        double finalPower = min_power + (distance * powerPerInch);

        if(finalPower > 1.0){
            finalPower = 1.0;
        }

        return finalPower;
    }

    public void manuallyResetPose(){
        if(isRedAlliance) {
            drive.setPoseEstimate(new Pose2d(-9, 9, Math.toRadians(90)));
        }else{
            drive.setPoseEstimate(new Pose2d(-9, 9, Math.toRadians(-90)));
        }
    }

    public void updateShooter() {
        double servoPos = getTurretPosition();
        current_angleturret_position = 0.9 - getTurretAngle();
        if(rotateToLeft){
            if(((leftTurret_initPosition-servoPos) >= min_leftturret_position) && ((rightTurret_initPosition-servoPos) >= min_rightturret_position)) {
                current_leftturret_position = leftTurret_initPosition - servoPos;
                current_rightturret_position = rightTurret_initPosition - servoPos;
            }else{
                current_leftturret_position = 0;
                current_rightturret_position = 0;
            }
        }else{
            if(((leftTurret_initPosition+servoPos) <= max_leftturret_position) && ((rightTurret_initPosition+servoPos) <= max_rightturret_position)) {
                current_leftturret_position = leftTurret_initPosition + servoPos;
                current_rightturret_position = rightTurret_initPosition + servoPos;
            }else{
                current_leftturret_position = 1;
                current_rightturret_position = 1;
            }
        }
        LeftTurret.setPosition(current_leftturret_position);
        RightTurret.setPosition(current_rightturret_position);
        AngleTurret.setPosition(current_angleturret_position);
    }

    void setTurretAngle(double angle, boolean rotateLeft){
        double servoPosition = angle * turretServoPosToDegree;
        if(rotateLeft){
            current_leftturret_position = leftTurret_initPosition - servoPosition;
            current_rightturret_position = rightTurret_initPosition - servoPosition;
        }else{
            current_leftturret_position = leftTurret_initPosition + servoPosition;
            current_rightturret_position = rightTurret_initPosition + servoPosition;
        }
        LeftTurret.setPosition(current_leftturret_position);
        RightTurret.setPosition(current_rightturret_position);
    }

    void setAngleTurretAngle(double x_pos, double y_pos, boolean redAlliance){
        double distanceToBasket;
        if(redAlliance) {
            distanceToBasket = Math.sqrt(Math.pow(x_red_basket_angleTurret - x_pos, 2) + Math.pow(y_red_basket_angleTurret - y_pos, 2));
        }else{
            distanceToBasket = Math.sqrt(Math.pow(x_blue_basket_angleTurret - x_pos, 2) + Math.pow(y_blue_basket_angleTurret - y_pos, 2));
        }

        double max_angle = 0.2;
        double min_angle = 0.9;
        double max_distance = 130;
        double anglePerInch = Math.abs(((max_angle-min_angle)/max_distance));
        double angleToCm = distanceToBasket * anglePerInch;

        if(angleToCm > 0.7){
            angleToCm = 0.7;
        }

        double angleTurretPosition = 0.9 - angleToCm;

        AngleTurret.setPosition(angleTurretPosition);
    }

    public void setAutonomousShooter(double angle, boolean rotateLeft, double x_pos, double y_pos, boolean redAlliance){
        setTurretAngle(angle, rotateLeft);
        setAngleTurretAngle(x_pos, y_pos, redAlliance);
    }
}
