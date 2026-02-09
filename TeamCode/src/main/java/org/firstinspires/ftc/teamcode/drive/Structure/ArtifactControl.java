package org.firstinspires.ftc.teamcode.drive.Structure;

import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.angleTurretSafePosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.autoStartIntakeTempTimer;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.colorSensor_gain;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.defaultFlyWheelPowerAuto;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.defaultFlyWheelSafePower;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.flyWheelAggressiveAcceleration;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.horizontalTurretDeadzone;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.intakeFirstRunTime;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.intakeFirstRunTimeManual;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.intakeMaxIdleRunTime;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.intakeRunTime;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.intakeRunTimeManual;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.leftDirectionAutoTurretOffset;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.leftDistanceThreshold;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.leftTurretSafePosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.leftTurret_initPosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.lightIntensityThreshold;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.marginThreshold;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.minimumBasketDistance;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.pushArtifact_push_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.pushArtifact_retract_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.rightDirectionAutoTurretOffset;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.rightDirectionManualTurretOffset;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.rightDistanceThreshold;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.rightTurretSafePosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.rightTurret_initPosition;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.min_leftturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.min_rightturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.min_angleturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_leftturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_rightturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.max_angleturret_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.artifact_block_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.artifact_unblock_position;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.robotAngularVelocityThreshold;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage.robotVelocityThreshold;
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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public ElapsedTime timer = new ElapsedTime();

    DcMotorEx Intake_LeftMotor;
    DcMotorEx Intake_RightMotor;
    DcMotorEx Outtake_LeftMotor;
    DcMotorEx Outtake_RightMotor;

    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;

    NormalizedColorSensor colorSensor;

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
        Outtake_LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake_RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake_RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Outtake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        LeftTurret = hwdmap.get(Servo.class,"LeftTurret");
        RightTurret = hwdmap.get(Servo.class,"RightTurret");

        AngleTurret = hwdmap.get(Servo.class,"AngleTurret");

        BlockArtifact = hwdmap.get(Servo.class,"BlockArtifact");

        PushArtifactServo = hwdmap.get(Servo.class, "PushLastArtifact");

        leftDistanceSensor = hwdmap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hwdmap.get(DistanceSensor.class, "rightDistanceSensor");

        colorSensor = hwdmap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensor.setGain(colorSensor_gain);
    }

    public double current_rightturret_position= rightTurret_initPosition + rightDirectionManualTurretOffset;
    public double current_leftturret_position = leftTurret_initPosition + rightDirectionManualTurretOffset;
    public double current_angleturret_position = angleTurretSafePosition;

    public double headingAngle = 0.0;
    public double x_position = 0.0;
    public double y_position = 0.0;
    double lastLeftTurretPos = 2.0;
    double lastRightTurretPos = 2.0;
    double lastVerticalPos = 2.0;
    public double leftFlyWheelSpeed = 0.0;
    public double rightFlyWheelSpeed = 0.0;
    public double calculatedRobotPose_X = 0.0;
    public double calculatedRobotPose_Y = 0.0;

    boolean toggleButton = false;
    boolean stoggleButton = false;
    boolean manualResetPoseToggle = false;
    public boolean artifact_status_blocked = false;
    public boolean manualControl = true;
    public boolean wantsToThrowArtifacts = false;

    public boolean allowedToShoot = false;
    boolean rotateToLeft = false;
    public double defaultFlyWheelPower = defaultFlyWheelSafePower;
    double basketDistance = 0.0;
    public double robotVelocity = 0.0;
    public double robotAngularVelocity = 0.0;
    public double robotAngleAprilTag = 0.0;
    public double currentTargetFlyWheelVelocity = targetFlyWheelSpeed;
    public double leftDistanceSensor_distance = 0.0;
    public double rightDistanceSensor_distance = 0.0;
    public double redNorm = 0.0;
    public double greenNorm = 0.0;
    public double blueNorm = 0.0;
    public double lightIntensity = 0.0;
    double distanceTimeSnapshot = 0.0;

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
    boolean tempThrowing = false;
    int tempCounter = 0;
    boolean generalManualModeCall = false;
    boolean getPoseToggle = false;
    public boolean automatedRobotPoseReset = false;
    public boolean isRobotStationary = false;
    boolean autoPoseResetToggle = false;
    boolean firstPoseReset = false;
    public boolean robotWantsToAutoShoot = false;
    public boolean robotWantsToStartIntake = false;
    public boolean oneCheckPerArtifact = false;
    public boolean generalIntakeActivaton = false;
    public boolean robotAutoShootToggle = false;
    public boolean robotAutoIntakeToggle = true;

    public int burstCounter = 0;
    public int forceActivationOfIntake_counter = 0;
    public int artifactCounter = 0;

    public void initServo(){
        AngleTurret.setPosition(angleTurretSafePosition);
        LeftTurret.setPosition(leftTurret_initPosition+rightDirectionManualTurretOffset);
        RightTurret.setPosition(rightTurret_initPosition+rightDirectionManualTurretOffset);
        PushArtifactServo.setPosition(pushArtifact_retract_position);
        BlockArtifact.setPosition(artifact_block_position);

        pushArtifact = false;
    }

    public void resetYaw(){
        gyroscope.resetHeading();
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

        drive.update();
    }

    public void updateArtifactPose(){ aprilTagIdentification.getArtifactPose();}

    public void updateAprilTag(){
        aprilTagIdentification.telemetryAprilTag();
    }

    public int getCurrentTag(){
        return aprilTagIdentification.getPatternId();
    }

    public void manualModeInit(){
        LeftTurret.setPosition(leftTurretSafePosition+rightDirectionManualTurretOffset);
        RightTurret.setPosition(rightTurretSafePosition+rightDirectionManualTurretOffset);
        AngleTurret.setPosition(angleTurretSafePosition);
        PushArtifactServo.setPosition(pushArtifact_retract_position);
        BlockArtifact.setPosition(artifact_block_position);

        defaultFlyWheelPower = defaultFlyWheelSafePower;
        pushArtifact = false;
    }

    public void Run(){
        updateAprilTag();
        updateArtifactPose();
        drive.update();

        Pose2d robotPose = drive.getPoseEstimate();
        Pose2d robotPoseVelocity = drive.getPoseVelocity();

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        lightIntensity = ((OpticalDistanceSensor) colorSensor).getLightDetected();

        leftDistanceSensor_distance = leftDistanceSensor.getDistance(DistanceUnit.CM);
        rightDistanceSensor_distance = rightDistanceSensor.getDistance(DistanceUnit.CM);

        redNorm = colors.red / colors.alpha;
        greenNorm = colors.green / colors.alpha;
        blueNorm = colors.blue / colors.alpha;

        headingAngle = gyroscope.getHeading();
        x_position = robotPose.getX();
        y_position = robotPose.getY();

        if(robotPoseVelocity != null) {
            robotVelocity = Math.abs(robotPoseVelocity.getX()) + Math.abs(robotPoseVelocity.getY());
            robotAngularVelocity = Math.abs(robotPoseVelocity.getHeading());

            if(robotVelocity < robotVelocityThreshold && robotAngularVelocity < robotAngularVelocityThreshold){
                isRobotStationary = true;
            }else{
                isRobotStationary = false;
            }
        }else{
            isRobotStationary = false;
        }

        leftFlyWheelSpeed = Outtake_LeftMotor.getVelocity();
        rightFlyWheelSpeed = Outtake_RightMotor.getVelocity();

        if(!firstTimeManual && manualControl){
            manualModeInit();
            firstTimeManual = true;
            switchFromManualMode = true;
            current_angleturret_position = angleTurretSafePosition;
        }else if(firstTimeManual && !manualControl){
            firstTimeManual = false;
        }

        if(!manualControl) {
            basketDistance = getBasketDistance(0,0,false,false);
            areaOfThrowing();
            if(allowedToShoot) {
                updateShooter();
            }

            if(robotAutoIntakeToggle && !wantsToThrowArtifacts) {
                autoArtifactsIntake();
            }

            if(robotAutoShootToggle) {
                autoArtifactsShooting();

                autoArtifactCounter();
            }
        }

        if(gamepad2.a){
            if(!artifactToggle) {
                getArtifacts(false);
                artifactToggle = true;
            }
        }else if(gamepad2.dpad_up){
            if(!artifactToggle) {
                if (allowedToShoot && !manualControl) {
                    wantsToThrowArtifacts = true;
                    oneTimeBurst = false;
                    burstCounter = 0;

                    if(forceActivationOfIntake_counter == 0) {
                        timer.reset();
                    }

                    forceActivationOfIntake_counter = forceActivationOfIntake_counter + 1;

                    throwArtifacts(getFlyWheelPower(0, 0, false, false), true, false);
                } else if (manualControl) {
                    tempCounter = tempCounter + 1;
                    generalManualModeCall = true;
                    if(tempCounter > 1){
                        tempThrowing = true;
                        timer.reset();
                    }
                    throwArtifacts(0, false, false);
                }

                artifactToggle = true;
                artifactCounter = 0;
            }
        }else{
            artifactToggle = false;
        }

        if(generalIntakeActivaton){
            getArtifacts(true);
        }

        if(generalManualModeCall){
            throwArtifacts(0,false,false);
        }

        if(wantsToThrowArtifacts){
            throwArtifacts(getFlyWheelPower(0,0,false,false), true, false);
        }

        if(gamepad2.b){
            stopIntakeOuttake();
            wantsToThrowArtifacts = false;
            oneTimeBurst = false;
            intakeRunning = false;
            forceActivationOfIntake_counter = 0;
            burstCounter = 0;
            PushArtifactServo.setPosition(pushArtifact_retract_position);
            pushArtifact = false;
            tempCounter = 0;
            tempThrowing = false;
            generalManualModeCall = false;
            generalIntakeActivaton = false;
        }

        if(gamepad2.x){
            if(!pushArtifactToggle){
                robotAutoShootToggle = !robotAutoShootToggle;
                pushArtifactToggle = true;
            }
        }else{
            pushArtifactToggle = false;
        }

        if (gamepad2.right_bumper && current_leftturret_position < max_leftturret_position && current_rightturret_position < max_rightturret_position && manualControl) {
            if(!toggleButton) {
                current_leftturret_position = current_leftturret_position + 0.01;
                current_rightturret_position = current_rightturret_position + 0.01;
                LeftTurret.setPosition(current_leftturret_position);
                RightTurret.setPosition(current_rightturret_position);
                toggleButton = true;
            }
        }else if (gamepad2.left_bumper && current_leftturret_position > min_leftturret_position && current_rightturret_position > min_rightturret_position && manualControl) {
            if(!toggleButton) {
                current_leftturret_position = current_leftturret_position - 0.01;
                current_rightturret_position = current_rightturret_position - 0.01;
                LeftTurret.setPosition(current_leftturret_position);
                RightTurret.setPosition(current_rightturret_position);
                toggleButton = true;
            }
        }else{
            toggleButton = false;
        }

        if (gamepad2.dpad_right && current_angleturret_position-0.05 >= max_angleturret_position && manualControl) {
            if(!stoggleButton) {
                current_angleturret_position = current_angleturret_position - 0.05;
                AngleTurret.setPosition(current_angleturret_position);
                stoggleButton = true;
            }
        } else if (gamepad2.dpad_down && current_angleturret_position+0.05 <= min_angleturret_position && manualControl) {
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

        if(gamepad2.dpad_down){
            if(!manualResetPoseToggle) {
                manuallyResetPose();
                manualResetPoseToggle = true;
            }
        }else if(gamepad2.dpad_left && !manualControl){
            if(!manualResetPoseToggle) {
                manuallyExtraResetPose(true);
                manualResetPoseToggle = true;
            }
        }else if(gamepad2.dpad_right && !manualControl){
            if(!manualResetPoseToggle) {
                manuallyExtraResetPose(false);
                manualResetPoseToggle = true;
            }
        } else{
            manualResetPoseToggle = false;
        }

        if(gamepad2.left_stick_button){
            if(!getPoseToggle){
                aprilTagIdentification.getRobotPose();

                calculatedRobotPose_X = aprilTagIdentification.robotPose_x;
                calculatedRobotPose_Y = aprilTagIdentification.robotPose_y;
                robotAngleAprilTag = aprilTagIdentification.bearingAngle;

                robotAutoIntakeToggle = !robotAutoIntakeToggle;

                getPoseToggle = true;
            }
        }else{
            getPoseToggle = false;
        }

        if(gamepad2.right_stick_button){
            if(!autoPoseResetToggle){
                automatedRobotPoseReset = !automatedRobotPoseReset;
                autoPoseResetToggle = true;
            }
        }else{
            autoPoseResetToggle = false;
        }

        if(automatedRobotPoseReset){
            if(allowedToShoot && !manualControl && isRobotStationary){
                if(!firstPoseReset) {
                    aprilTagIdentification.getRobotPose();

                    if(aprilTagIdentification.locTagFound) {

                        calculatedRobotPose_X = aprilTagIdentification.robotPose_x;
                        calculatedRobotPose_Y = aprilTagIdentification.robotPose_y;
                        robotAngleAprilTag = aprilTagIdentification.bearingAngle;

                        if (robotAngleAprilTag >= 0) {
                            gyroscope.resetHeading();
                            if (isRedAlliance) {
                                gyroscope.setAngleOffset(36.5 - robotAngleAprilTag);
                                drive.setPoseEstimate(new Pose2d(calculatedRobotPose_X, calculatedRobotPose_Y, Math.toRadians(126.5 - robotAngleAprilTag)));
                            } else {
                                gyroscope.setAngleOffset(-36.5 - robotAngleAprilTag);
                                drive.setPoseEstimate(new Pose2d(calculatedRobotPose_X, calculatedRobotPose_Y, Math.toRadians(-126.5 - robotAngleAprilTag)));
                            }
                        } else if (robotAngleAprilTag < 0) {
                            gyroscope.resetHeading();
                            if (isRedAlliance) {
                                gyroscope.setAngleOffset(36.5 + Math.abs(robotAngleAprilTag));
                                drive.setPoseEstimate(new Pose2d(calculatedRobotPose_X, calculatedRobotPose_Y, Math.toRadians(126.5 + Math.abs(robotAngleAprilTag))));
                            } else {
                                gyroscope.setAngleOffset(-36.5 + Math.abs(robotAngleAprilTag));
                                drive.setPoseEstimate(new Pose2d(calculatedRobotPose_X, calculatedRobotPose_Y, Math.toRadians(-126.5 + Math.abs(robotAngleAprilTag))));
                            }
                        }

                        gamepad2.rumble(1000);
                        firstPoseReset = true;
                    }
                }
            }else{
                firstPoseReset = false;
            }
        }
    }

    public void getArtifacts(boolean inAutoMode){
        if(inAutoMode){
            pushArtifact = false;
        }else{
            BlockArtifact.setPosition(artifact_block_position);
            PushArtifactServo.setPosition(pushArtifact_retract_position);
            Intake_LeftMotor.setPower(1);
            Intake_RightMotor.setPower(1);
            artifact_status_blocked = true;
            pushArtifact = false;
        }
    }

    public void autoArtifactsIntake(){
        if((leftDistanceSensor_distance < leftDistanceThreshold || rightDistanceSensor_distance < rightDistanceThreshold) && artifactCounter < 3){
            robotWantsToStartIntake = true;
            getArtifacts(false);
            distanceTimeSnapshot = timer.milliseconds();
        }else{
            robotWantsToStartIntake = false;
            if(distanceTimeSnapshot+autoStartIntakeTempTimer < timer.milliseconds()) {
                stopIntakeOuttake();
            }
        }
    }

    public void autoArtifactsShooting(){
        if(isRobotStationary && allowedToShoot){
            robotWantsToAutoShoot = true;

                /*wantsToThrowArtifacts = true;
                oneTimeBurst = false;
                burstCounter = 0;

                if(forceActivationOfIntake_counter == 0) {
                    timer.reset();
                }

                forceActivationOfIntake_counter = forceActivationOfIntake_counter + 1;

                throwArtifacts(getFlyWheelPower(0, 0, false, false), true, false);
            */}else{
            robotWantsToAutoShoot = false;

                /*wantsToThrowArtifacts = false;
                oneTimeBurst = false;
                burstCounter = 0;

                forceActivationOfIntake_counter = 0;

                PushArtifactServo.setPosition(pushArtifact_retract_position);
                pushArtifact = false;*/
        }
    }

    public void autoArtifactCounter(){
        if(lightIntensity > lightIntensityThreshold){
            if(!oneCheckPerArtifact){
                //if(artifactCounter < 3) {
                artifactCounter = artifactCounter + 1;
//                        generalIntakeActivaton = true;
//                        timer.reset();
//                        switch(artifactCounter){
//                            case 1:
//                                targetIntakeTime = autoIntakeFirstArtifactTimer;
//                                break;
//                            case 2:
//                                targetIntakeTime = autoIntakeSecondArtifactTimer;
//                                break;
//                            case 3:
//                                targetIntakeTime = autoIntakeThirdArtifactTimer;
//                                break;
//                        }

//                    }
                oneCheckPerArtifact = true;
            }
        }else{
            oneCheckPerArtifact = false;
        }
    }

    public void setCustomTargetFlyWheelVelocity(double flyWheelPower){
        currentTargetFlyWheelVelocity = (targetFlyWheelSpeed * flyWheelPower) - 100.0;
    }

    public void throwArtifacts(double customFlyWheelPower, boolean useCustomPower, boolean autonomousMode){
        if(useCustomPower) {
            setCustomTargetFlyWheelVelocity(customFlyWheelPower);

            if(Outtake_LeftMotor.getVelocity() > currentTargetFlyWheelVelocity-75.0 || Outtake_LeftMotor.getVelocity() > currentTargetFlyWheelVelocity-75.0) {
                Outtake_LeftMotor.setPower(customFlyWheelPower);
                Outtake_RightMotor.setPower(customFlyWheelPower);
            }else{
                if (customFlyWheelPower + flyWheelAggressiveAcceleration <= 1.0) {
                    Outtake_LeftMotor.setPower(customFlyWheelPower + flyWheelAggressiveAcceleration);
                    Outtake_RightMotor.setPower(customFlyWheelPower + flyWheelAggressiveAcceleration);
                } else {
                    Outtake_LeftMotor.setPower(1.0);
                    Outtake_RightMotor.setPower(1.0);
                }
            }
        }else{
            if(autonomousMode){
                setCustomTargetFlyWheelVelocity(defaultFlyWheelPowerAuto);

                if(Outtake_LeftMotor.getVelocity() > currentTargetFlyWheelVelocity-75.0 || Outtake_LeftMotor.getVelocity() > currentTargetFlyWheelVelocity-75.0) {
                    Outtake_LeftMotor.setPower(defaultFlyWheelPowerAuto);
                    Outtake_RightMotor.setPower(defaultFlyWheelPowerAuto);
                }else{
                    Outtake_LeftMotor.setPower(defaultFlyWheelPowerAuto + 0.13);
                    Outtake_RightMotor.setPower(defaultFlyWheelPowerAuto + 0.13);
                }
            }else{
                Outtake_LeftMotor.setPower(defaultFlyWheelPower);
                Outtake_RightMotor.setPower(defaultFlyWheelPower);
                setCustomTargetFlyWheelVelocity(defaultFlyWheelPower);
            }
        }

        if(autonomousMode && wantsToThrowArtifacts){
            burstShootingArtifacts();
        }else if(wantsToThrowArtifacts && !manualControl && forceActivationOfIntake_counter < 3) {
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
            if(timer.milliseconds() < intakeFirstRunTimeManual && tempCounter == 2 && tempThrowing ){
                BlockArtifact.setPosition(artifact_unblock_position);
                PushArtifactServo.setPosition(pushArtifact_retract_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                artifact_status_blocked = false;
                pushArtifact = false;
            }else if(timer.milliseconds() < intakeRunTimeManual && tempCounter == 3 && tempThrowing) {
                BlockArtifact.setPosition(artifact_unblock_position);
                PushArtifactServo.setPosition(pushArtifact_retract_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                artifact_status_blocked = false;
                pushArtifact = false;
            }else if(timer.milliseconds() < intakeRunTimeManual && tempCounter >= 4 && tempThrowing){
                BlockArtifact.setPosition(artifact_unblock_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                PushArtifactServo.setPosition(pushArtifact_push_position);
                pushArtifact = true;
                artifact_status_blocked = false;
            } else{
                tempThrowing = false;
                Intake_LeftMotor.setPower(0);
                Intake_RightMotor.setPower(0);
                PushArtifactServo.setPosition(pushArtifact_retract_position);
                pushArtifact = false;
            }

            if(tempCounter >= 4 && timer.milliseconds() > intakeRunTimeManual){
                generalManualModeCall = false;
                artifact_status_blocked = true;
                BlockArtifact.setPosition(artifact_block_position);
            }
        }
    }

    public void burstShootingArtifacts(){
        if(burstCounter < 3) {
            if(!oneTimeBurst && ((Outtake_LeftMotor.getVelocity() > currentTargetFlyWheelVelocity || Outtake_RightMotor.getVelocity() > currentTargetFlyWheelVelocity) || timer.milliseconds() > timeoutTime) ){
                timer.reset();
                oneTimeBurst = true;
                intakeRunning = true;
            }

            if(burstCounter == 2 && intakeRunning && timer.milliseconds() < intakeMaxIdleRunTime){
                BlockArtifact.setPosition(artifact_unblock_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                PushArtifactServo.setPosition(pushArtifact_push_position);
                artifact_status_blocked = false;
                pushArtifact = true;
            }else if(burstCounter == 2 && intakeRunning && timer.milliseconds() > intakeMaxIdleRunTime){
                BlockArtifact.setPosition(artifact_block_position);
                PushArtifactServo.setPosition(pushArtifact_retract_position);
                artifact_status_blocked = true;
                pushArtifact = false;
                if(intakeRunning) {
                    burstCounter = burstCounter + 1;
                    intakeRunning = false;
                }
            }

            if(timer.milliseconds() < intakeFirstRunTime && intakeRunning && burstCounter == 0){
                BlockArtifact.setPosition(artifact_unblock_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                artifact_status_blocked = false;
            }else if(timer.milliseconds() > intakeFirstRunTime && intakeRunning && burstCounter == 0){
                Intake_LeftMotor.setPower(0);
                Intake_RightMotor.setPower(0);
                if(intakeRunning) {
                    burstCounter = burstCounter + 1;
                    intakeRunning = false;
                }
            }

            if(timer.milliseconds() < intakeRunTime && intakeRunning && burstCounter == 1){
                BlockArtifact.setPosition(artifact_unblock_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                artifact_status_blocked = false;
            }else if(timer.milliseconds() > intakeRunTime && intakeRunning && burstCounter == 1){
                Intake_LeftMotor.setPower(0);
                Intake_RightMotor.setPower(0);
                if(intakeRunning) {
                    burstCounter = burstCounter + 1;
                    intakeRunning = false;
                }
            }

            if(((Outtake_LeftMotor.getVelocity() > currentTargetFlyWheelVelocity || Outtake_RightMotor.getVelocity() > currentTargetFlyWheelVelocity) && !intakeRunning && oneTimeBurst && (timer.milliseconds() > intakeRunTime) && burstCounter < 3) || (timer.milliseconds() > timeoutTime)){
                oneTimeBurst = false;
            }

        }else{
            oneTimeBurst = false;
            burstCounter = 0;
            wantsToThrowArtifacts = false;
            forceActivationOfIntake_counter = 0;
            stopIntakeOuttake();
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
            gamepad2.rumble(500);
            oneTimeRumble = true;
        }else if(!allowedToShoot && oneTimeRumble){
            gamepad2.rumble(500);
            oneTimeRumble = false;
        }
    }

    public void dynamicTargetAngle(){
        double positive_x_position = Math.abs(x_position + 61);
        double positive_y_position;
        if(isRedAlliance){
            positive_y_position = Math.abs((y_position - 61));
        }else{
            positive_y_position = Math.abs((y_position + 61));
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
        double angleTurretPosition;

        angleTurretPosition = (0.0000207725 * (basketDistance*basketDistance)) - (0.00755001*basketDistance) + 0.865169;

        if(angleTurretPosition > 0.75){
            angleTurretPosition = 0.75;
        }else if(angleTurretPosition < 0.25){
            angleTurretPosition = 0.25;
        }
        return angleTurretPosition;
    }

    public double getFlyWheelPower(double custom_x_pos, double custom_y_pos, boolean redAlliance, boolean useCustomPos){
        double distance;
        if(!useCustomPos) {
            distance = basketDistance-minimumBasketDistance;
        }else{
            distance = getBasketDistance(custom_x_pos, custom_y_pos, redAlliance, true)-minimumBasketDistance;
        }

        double flyWheelPower = ((-3.15936e-7) * distance * distance * distance) + (0.000074273 * distance * distance) - (0.00230794 * distance) + 0.606381;

        if(flyWheelPower > 0.87){
            flyWheelPower = 0.87;
        }else if(flyWheelPower < 0.6){
            flyWheelPower = 0.6;
        }

        return flyWheelPower;
    }

    public void manuallyResetPose(){
        if(isRedAlliance) {
            drive.setPoseEstimate(new Pose2d(-55.5, 43.5, Math.toRadians(126.5)));
            gyroscope.resetHeading();
            gyroscope.setAngleOffset(36.5);
        }else{
            drive.setPoseEstimate(new Pose2d(-55.5, -43.5, Math.toRadians(-126.5)));
            gyroscope.resetHeading();
            gyroscope.setAngleOffset(-36.5);
        }
    }

    public void manuallyExtraResetPose(boolean leftField){
        if(isRedAlliance) {
            if(leftField){
                drive.setPoseEstimate(new Pose2d(60.5, -60.0, Math.toRadians(-90)));
                gyroscope.resetHeading();
                gyroscope.setAngleOffset(-180.0);
            }else{
                drive.setPoseEstimate(new Pose2d(60.5, 60.0, Math.toRadians(90)));
                gyroscope.resetHeading();
                gyroscope.setAngleOffset(0.0);
            }
        }else{
            if(leftField){
                drive.setPoseEstimate(new Pose2d(60.5, -60.0, Math.toRadians(-90)));
                gyroscope.resetHeading();
                gyroscope.setAngleOffset(0.0);
            }else{
                drive.setPoseEstimate(new Pose2d(60.5, 60.0, Math.toRadians(90)));
                gyroscope.resetHeading();
                gyroscope.setAngleOffset(-180.0);
            }
        }
    }

    public void updateShooter() {
        double servoPos = getTurretPosition();
        current_angleturret_position = getTurretAngle();
        if(!rotateToLeft){
            if(((leftTurret_initPosition+servoPos) <= max_leftturret_position) && ((rightTurret_initPosition+servoPos) <= max_rightturret_position)) {
                current_leftturret_position = leftTurret_initPosition + servoPos + leftDirectionAutoTurretOffset;
                current_rightturret_position = rightTurret_initPosition + servoPos + leftDirectionAutoTurretOffset;
            }else{
                current_leftturret_position = max_leftturret_position;
                current_rightturret_position = max_rightturret_position;
            }
        }else{
            if(((leftTurret_initPosition-servoPos+rightDirectionAutoTurretOffset) >= min_leftturret_position) && ((rightTurret_initPosition-servoPos+rightDirectionAutoTurretOffset) >= min_rightturret_position)) {
                current_leftturret_position = leftTurret_initPosition - servoPos + rightDirectionAutoTurretOffset;
                current_rightturret_position = rightTurret_initPosition - servoPos + rightDirectionAutoTurretOffset;
            }else{
                current_leftturret_position = min_leftturret_position;
                current_rightturret_position = min_rightturret_position;
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
        if(!rotateLeft){
            current_leftturret_position = leftTurret_initPosition + servoPosition;
            current_rightturret_position = rightTurret_initPosition + servoPosition;
        }else{
            current_leftturret_position = leftTurret_initPosition - servoPosition + rightDirectionAutoTurretOffset;
            current_rightturret_position = rightTurret_initPosition - servoPosition + rightDirectionAutoTurretOffset;
        }
        LeftTurret.setPosition(current_leftturret_position);
        RightTurret.setPosition(current_rightturret_position);
    }

    void setAngleTurretAngle(double x_pos, double y_pos, boolean redAlliance){
        double distanceToBasket;

        distanceToBasket = getBasketDistance(x_pos,y_pos,redAlliance,true);

        double angleTurretPosition;

        angleTurretPosition = (0.0000207725 * (distanceToBasket*distanceToBasket)) - (0.00755001*distanceToBasket) + 0.865169;

        if(angleTurretPosition > 0.75){
            angleTurretPosition = 0.75;
        }else if(angleTurretPosition < 0.25){
            angleTurretPosition = 0.25;
        }

        AngleTurret.setPosition(angleTurretPosition);
    }

    void manuallySetAngleTurret(double position){
        AngleTurret.setPosition(position);
    }

    public void setAutonomousShooter(double angle, boolean rotateLeft, double x_pos, double y_pos, boolean redAlliance, boolean audience){
        setTurretAngle(angle, rotateLeft);
        if(!audience) {
            setAngleTurretAngle(x_pos, y_pos, redAlliance);
        }else{
            manuallySetAngleTurret(x_pos);
        }
    }

    public void setAutonomousThrowFlags(){
        wantsToThrowArtifacts = true;

        oneTimeBurst = false;
        burstCounter = 0;
        forceActivationOfIntake_counter = 0;
        intakeRunning = false;

        pushArtifact = false;
        PushArtifactServo.setPosition(pushArtifact_retract_position);

        artifact_status_blocked = false;
        BlockArtifact.setPosition(artifact_unblock_position);

        timer.reset();
    }

    public void setAutonomousResetFlags(){
        wantsToThrowArtifacts = false;

        oneTimeBurst = false;
        intakeRunning = false;
        burstCounter = 0;
        forceActivationOfIntake_counter = 0;

        pushArtifact = false;
        PushArtifactServo.setPosition(pushArtifact_retract_position);

        stopIntakeOuttake();
    }
}
