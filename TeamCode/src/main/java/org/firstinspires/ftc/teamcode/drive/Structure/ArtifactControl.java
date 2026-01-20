package org.firstinspires.ftc.teamcode.drive.Structure;

import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.angleTurret_initPosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.angleTurret_manualPosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.horizontalTurretDeadzone;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.intakeMaxIdleRunTime;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.intakeRunTime;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.leftTurret_initPosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.marginThreshold;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_FlyWheelDistance;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_FlyWheelPower;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_TurretAngleDistance;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.min_FlyWheelPower;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.pushArtifact_push_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.pushArtifact_retract_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.rightTurret_initPosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.min_leftturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.min_rightturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.min_angleturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_leftturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_rightturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_angleturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.artifact_block_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.artifact_unblock_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.targetFlyWheelSpeed;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.timeoutTime;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.turretServoPosToDegree;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.verticalTurretDeadzone;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.x_red_basket_angleTurret;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.x_blue_basket_angleTurret;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.y_red_basket_angleTurret;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.y_blue_basket_angleTurret;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx Intake_LeftMotor;
    DcMotorEx Intake_RightMotor;
    DcMotorEx Outtake_LeftMotor;
    DcMotorEx Outtake_RightMotor;

    Servo LeftTurret;
    Servo RightTurret;
    Servo AngleTurret;
    Servo BlockArtifact;
    Servo PushArtifactServo;

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

        Intake_LeftMotor = hwdmap.get(DcMotorEx.class, "Intake_LeftMotor");
        Intake_RightMotor = hwdmap.get(DcMotorEx.class, "Intake_RightMotor");

        Outtake_LeftMotor = hwdmap.get(DcMotorEx.class, "Outtake_LeftMotor");
        Outtake_RightMotor = hwdmap.get(DcMotorEx.class, "Outtake_RightMotor");

        Intake_LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake_RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake_LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake_RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake_LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake_RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Outtake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        LeftTurret = hwdmap.get(Servo.class,"LeftTurret");
        RightTurret = hwdmap.get(Servo.class,"RightTurret");

        AngleTurret = hwdmap.get(Servo.class,"AngleTurret");

        BlockArtifact = hwdmap.get(Servo.class,"BlockArtifact");

        PushArtifactServo = hwdmap.get(Servo.class, "PushLastArtifact");
    }

    public double current_rightturret_position= rightTurret_initPosition;
    public double current_leftturret_position = leftTurret_initPosition;
    public double current_angleturret_position = angleTurret_initPosition;

    public double headingAngle = 0.0;
    public double x_position = 0.0;
    public double y_position = 0.0;
    double lastLeftTurretPos = 2.0;
    double lastRightTurretPos = 2.0;
    double lastVerticalPos = 2.0;
    public double leftFlyWheelSpeed = 0.0;
    public double rightFlyWheelSpeed = 0.0;

    boolean toggleButton = false;
    boolean stoggleButton = false;
    boolean manualResetPoseToggle = false;
    public boolean artifact_status_blocked = false;
    public boolean manualControl = true;
    public boolean wantsToThrowArtifacts = false;

    public boolean allowedToShoot = false;
    boolean rotateToLeft = false;
    public double defaultFlyWheelPower = 1.0;
    double basketDistance = 0.0;

    boolean flyToggle = false;
    boolean toggleS = false;
    boolean oneTimeRumble = false;
    boolean firstTimeManual = false;
    boolean switchFromManualMode = false;
    boolean artifactToggle = false;
    boolean oneTimeBurst = false;
    boolean intakeRunning = false;
    boolean pushArtifactToggle = false;
    public boolean pushArtifact = false;

    public int burstCounter = 0;
    public int forceActivationOfIntake_counter = 0;

    public void initServo(){
        AngleTurret.setPosition(angleTurret_initPosition);
        LeftTurret.setPosition(leftTurret_initPosition);
        RightTurret.setPosition(rightTurret_initPosition);
        PushArtifactServo.setPosition(pushArtifact_retract_position);

        gyroscope.resetHeading();
        pushArtifact = false;
    }

    public void initRobotPose(){
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
    }

    public void updateAprilTag(){
        aprilTagIdentification.telemetryAprilTag();
    }

    public int getCurrentTag(){
        return aprilTagIdentification.getPatternId();
    }

    public void manualModeInit(){
        LeftTurret.setPosition(leftTurret_initPosition);
        RightTurret.setPosition(rightTurret_initPosition);
        AngleTurret.setPosition(angleTurret_manualPosition);
        PushArtifactServo.setPosition(pushArtifact_retract_position);

        defaultFlyWheelPower = 1.0;
        pushArtifact = false;
    }

    public void Run(){
        updateAprilTag();
        drive.update();

        Pose2d robotPose = drive.getPoseEstimate();

        headingAngle = gyroscope.getHeading();
        x_position = robotPose.getX();
        y_position = robotPose.getY();

        leftFlyWheelSpeed = Outtake_LeftMotor.getVelocity();
        rightFlyWheelSpeed = Outtake_RightMotor.getVelocity();

        if(!firstTimeManual && manualControl){
            manualModeInit();
            firstTimeManual = true;
            switchFromManualMode = true;
        }else if(firstTimeManual && !manualControl){
            firstTimeManual = false;
        }

        if(!manualControl) {
            basketDistance = getBasketDistance(0,0,false,false);
            areaOfThrowing();
            updateShooter();
        }

        if(gamepad2.a){
            if(!artifactToggle) {
                getArtifacts();
                artifactToggle = true;
            }
        }else if(gamepad2.y){
            if(!artifactToggle) {
                if (allowedToShoot && !manualControl) {
                    wantsToThrowArtifacts = true;
                    oneTimeBurst = false;
                    burstCounter = 0;
                    throwArtifacts(getFlyWheelPower(0, 0, false, false), true);
                    forceActivationOfIntake_counter = forceActivationOfIntake_counter + 1;
                } else if (manualControl) {
                    throwArtifacts(0, false);
                }
                artifactToggle = true;
            }
        }else{
            artifactToggle = false;
        }

        if(wantsToThrowArtifacts){
            throwArtifacts(getFlyWheelPower(0,0,false,false), true);
        }

        if(gamepad2.b){
            stopIntakeOuttake();
            wantsToThrowArtifacts = false;
            oneTimeBurst = false;
            burstCounter = 0;
            PushArtifactServo.setPosition(pushArtifact_retract_position);
            pushArtifact = false;
        }

        if(gamepad2.x){
            if(!pushArtifactToggle){
                if(!pushArtifact){
                    PushArtifactServo.setPosition(pushArtifact_push_position);
                    pushArtifact = true;
                }else{
                    PushArtifactServo.setPosition(pushArtifact_retract_position);
                    pushArtifact = false;
                }
                pushArtifactToggle = true;
            }
        }else{
            pushArtifactToggle = false;
        }

        if (gamepad2.left_bumper && current_leftturret_position < min_leftturret_position && current_rightturret_position < min_rightturret_position && manualControl) {
            if(!toggleButton) {
                current_leftturret_position = current_leftturret_position + 0.05;
                current_rightturret_position = current_rightturret_position + 0.05;
                LeftTurret.setPosition(current_leftturret_position);
                RightTurret.setPosition(current_rightturret_position);
                toggleButton = true;
            }
        }else if (gamepad2.right_bumper && current_leftturret_position > max_leftturret_position && current_rightturret_position > max_rightturret_position && manualControl) {
            if(!toggleButton) {
                current_leftturret_position = current_leftturret_position - 0.05;
                current_rightturret_position = current_rightturret_position - 0.05;
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

        if(gamepad2.left_trigger > 0.75 && gamepad2.right_trigger < 0.75 && manualControl){
            if(!flyToggle && defaultFlyWheelPower > 0.0) {
                defaultFlyWheelPower = defaultFlyWheelPower - 0.05;
                flyToggle = true;
            }
        }else if(gamepad2.left_trigger < 0.75 && gamepad2.right_trigger > 0.75 && manualControl){
            if(!flyToggle && defaultFlyWheelPower < 1.0){
                defaultFlyWheelPower = defaultFlyWheelPower + 0.05;
                flyToggle = true;
            }
        }else{
            flyToggle = false;
        }

        if(gamepad2.left_trigger > 0.75 && gamepad2.right_trigger > 0.75){
            if(!toggleS) {
                manualControl = !manualControl;
                toggleS = true;
            }
        }else{
            toggleS = false;
        }

        if(gamepad2.dpad_left){
            if(!manualResetPoseToggle) {
                manuallyResetPose();
                manualResetPoseToggle = true;
            }
        }else{
            manualResetPoseToggle = false;
        }
    }

    public void getArtifacts(){
        BlockArtifact.setPosition(artifact_block_position);
        PushArtifactServo.setPosition(pushArtifact_retract_position);
        Intake_LeftMotor.setPower(1);
        Intake_RightMotor.setPower(1);
        artifact_status_blocked = true;
    }

    public void throwArtifacts(double customFlyWheelPower, boolean useCustomPower){
        if(useCustomPower) {
            Outtake_LeftMotor.setPower(customFlyWheelPower);
            Outtake_RightMotor.setPower(customFlyWheelPower);
        }else{
            Outtake_LeftMotor.setPower(defaultFlyWheelPower);
            Outtake_RightMotor.setPower(defaultFlyWheelPower);
        }

        if(wantsToThrowArtifacts && !manualControl && forceActivationOfIntake_counter < 3) {
            burstShootingArtifacts();
        }else if(forceActivationOfIntake_counter >= 3){
            BlockArtifact.setPosition(artifact_unblock_position);
            PushArtifactServo.setPosition(pushArtifact_retract_position);
            Intake_LeftMotor.setPower(1);
            Intake_RightMotor.setPower(1);
            artifact_status_blocked = false;
            wantsToThrowArtifacts = false;
            pushArtifact = false;
            forceActivationOfIntake_counter = 0;
        } else if(manualControl){
            BlockArtifact.setPosition(artifact_unblock_position);
            PushArtifactServo.setPosition(pushArtifact_retract_position);
            Intake_LeftMotor.setPower(1);
            Intake_RightMotor.setPower(1);
            artifact_status_blocked = false;
            pushArtifact = false;
        }
    }

    public void burstShootingArtifacts(){
        if(burstCounter < 3) {
            if(!oneTimeBurst && ((Outtake_LeftMotor.getVelocity() > targetFlyWheelSpeed) || (Outtake_RightMotor.getVelocity() > targetFlyWheelSpeed) || timer.milliseconds() > timeoutTime) ){
                timer.reset();
                oneTimeBurst = true;
                intakeRunning = true;
            }

            if(burstCounter == 2 && intakeRunning && timer.milliseconds() < intakeMaxIdleRunTime){
                BlockArtifact.setPosition(artifact_unblock_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                PushArtifactServo.setPosition(pushArtifact_push_position);
            }else if(burstCounter == 2 && intakeRunning && timer.milliseconds() > intakeMaxIdleRunTime){
                Intake_LeftMotor.setPower(0);
                Intake_RightMotor.setPower(0);
                BlockArtifact.setPosition(artifact_block_position);
                PushArtifactServo.setPosition(pushArtifact_retract_position);
                artifact_status_blocked = true;
                if(intakeRunning) {
                    burstCounter = burstCounter + 1;
                    intakeRunning = false;
                }
            }

            if(timer.milliseconds() < intakeRunTime && intakeRunning && burstCounter < 2){
                BlockArtifact.setPosition(artifact_unblock_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                artifact_status_blocked = false;
            }else if(timer.milliseconds() > intakeRunTime && intakeRunning && burstCounter < 2){
                Intake_LeftMotor.setPower(0);
                Intake_RightMotor.setPower(0);
                BlockArtifact.setPosition(artifact_block_position);
                artifact_status_blocked = true;
                if(intakeRunning) {
                    burstCounter = burstCounter + 1;
                    intakeRunning = false;
                }
            }

            if(((Outtake_LeftMotor.getVelocity() > targetFlyWheelSpeed || Outtake_RightMotor.getVelocity() > targetFlyWheelSpeed) && !intakeRunning && oneTimeBurst && (timer.milliseconds() > intakeRunTime) && burstCounter < 3) || (timer.milliseconds() > timeoutTime)){
                oneTimeBurst = false;
            }

        }else{
            oneTimeBurst = false;
            burstCounter = 0;
            wantsToThrowArtifacts = false;
            forceActivationOfIntake_counter = 0;
        }
    }

    public void stopIntakeOuttake(){
        Intake_LeftMotor.setPower(0);
        Intake_RightMotor.setPower(0);
        Outtake_LeftMotor.setPower(0);
        Outtake_RightMotor.setPower(0);
    }

    public void areaOfThrowing(){
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
            if(basketAngle >= 180){
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
        double dx;
        double dy;

        if(!useCustomPos) {
            if (isRedAlliance) {
                dx = x_red_basket_angleTurret - x_position;
                dy = y_red_basket_angleTurret - y_position;
            } else {
                dx = x_blue_basket_angleTurret - x_position;
                dy = y_blue_basket_angleTurret - y_position;
            }
        }else{
            if(redAlliance){
                dx = x_red_basket_angleTurret - custom_x_pos;
                dy = y_red_basket_angleTurret - custom_y_pos;
            }else{
                dx = x_blue_basket_angleTurret - custom_x_pos;
                dy = y_blue_basket_angleTurret - custom_y_pos;
            }
        }

        distanceToBasket = Math.sqrt((dx*dx) + (dy*dy));

        return distanceToBasket;
    }

    public double getTurretPosition(){
        double servoAngleToPosition;
        servoAngleToPosition = getBasketDirection() * turretServoPosToDegree;

        return servoAngleToPosition;
    }

    public double getTurretAngle(){
        double angleToCm;
        double anglePerInch = Math.abs(((max_angleturret_position-min_angleturret_position)/max_TurretAngleDistance));
        angleToCm = basketDistance * anglePerInch;

        if(angleToCm > 0.7){
            angleToCm = 0.7;
        }

        return angleToCm;
    }

    public double getFlyWheelPower(double custom_x_pos, double custom_y_pos, boolean redAlliance, boolean useCustomPos){
        double powerPerInch = Math.abs((max_FlyWheelPower-min_FlyWheelPower)/max_FlyWheelDistance);
        double distance;
        if(!useCustomPos) {
            distance = basketDistance;
        }else{
            distance = getBasketDistance(custom_x_pos, custom_y_pos, redAlliance, true);
        }

        double finalPower = min_FlyWheelPower + (distance * powerPerInch);

        if(finalPower > 1.0){
            finalPower = 1.0;
        }

        return finalPower;
    }

    public void manuallyResetPose(){
        if(isRedAlliance) {
            drive.setPoseEstimate(new Pose2d(-57, 45, Math.toRadians(126.5)));
        }else{
            drive.setPoseEstimate(new Pose2d(-57, -43, Math.toRadians(-126.5)));
        }
    }

    public void updateShooter() {
        double servoPos = getTurretPosition();
        current_angleturret_position = 0.9 - getTurretAngle();
        if(rotateToLeft){
            if(((leftTurret_initPosition+servoPos) <= max_leftturret_position) && ((rightTurret_initPosition+servoPos) <= max_rightturret_position)) {
                current_leftturret_position = leftTurret_initPosition + servoPos;
                current_rightturret_position = rightTurret_initPosition + servoPos;
            }else{
                current_leftturret_position = max_leftturret_position;
                current_rightturret_position = max_rightturret_position;
            }
        }else{
            if(((leftTurret_initPosition-servoPos) >= min_leftturret_position) && ((rightTurret_initPosition-servoPos) >= min_rightturret_position)) {
                current_leftturret_position = leftTurret_initPosition - servoPos;
                current_rightturret_position = rightTurret_initPosition - servoPos;
            }else{
                current_leftturret_position = max_leftturret_position;
                current_rightturret_position = max_rightturret_position;
            }
        }

        if((Math.abs(lastLeftTurretPos-current_leftturret_position) > horizontalTurretDeadzone) || (Math.abs(lastRightTurretPos-current_rightturret_position) > horizontalTurretDeadzone)) {
            LeftTurret.setPosition(current_leftturret_position);
            RightTurret.setPosition(current_rightturret_position);
            lastLeftTurretPos = current_leftturret_position;
            lastRightTurretPos = current_rightturret_position;
        }else if(switchFromManualMode){
            LeftTurret.setPosition(current_leftturret_position);
            RightTurret.setPosition(current_rightturret_position);
            lastLeftTurretPos = current_leftturret_position;
            lastRightTurretPos = current_rightturret_position;
        }

        if((Math.abs(lastVerticalPos-current_angleturret_position) > verticalTurretDeadzone)) {
            AngleTurret.setPosition(current_angleturret_position);
            lastVerticalPos = current_angleturret_position;
        }else if(switchFromManualMode){
            AngleTurret.setPosition(current_angleturret_position);
            lastVerticalPos = current_angleturret_position;
        }

        if(switchFromManualMode){
            switchFromManualMode = false;
        }
    }

    void setTurretAngle(double angle, boolean rotateLeft){
        double servoPosition = angle * turretServoPosToDegree;
        if(rotateLeft){
            current_leftturret_position = leftTurret_initPosition + servoPosition;
            current_rightturret_position = rightTurret_initPosition + servoPosition;
        }else{
            current_leftturret_position = leftTurret_initPosition - servoPosition;
            current_rightturret_position = rightTurret_initPosition - servoPosition;
        }
        LeftTurret.setPosition(current_leftturret_position);
        RightTurret.setPosition(current_rightturret_position);
    }

    void setAngleTurretAngle(double x_pos, double y_pos, boolean redAlliance){
        double distanceToBasket;

        distanceToBasket = getBasketDistance(x_pos,y_pos,redAlliance,true);

        double anglePerInch = Math.abs(((max_angleturret_position-min_angleturret_position)/max_TurretAngleDistance));
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
