package org.firstinspires.ftc.teamcode.drive.Structure;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.drive.ComputerVision.AprilTagIdentification;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;

public class ArtifactControl {
    Gamepad gamepad2;
    AprilTagIdentification aprilTagIdentification = new AprilTagIdentification();
    MultipleTelemetry telemetry;
    Gyroscope gyroscope = new Gyroscope();
    SampleMecanumDrive drive;
    TwoWheelTrackingLocalizer twoWheelTrackingLocalizer;

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

    public ArtifactControl(HardwareMap hwdmap, Gamepad gmpd, MultipleTelemetry telemetrys, int fieldCase){
        gamepad2 = gmpd;
        telemetry = telemetrys;
        aprilTagIdentification.init(hwdmap, telemetrys);
        gyroscope.Init(hwdmap);

        drive = new SampleMecanumDrive(hwdmap);
        switch(fieldCase){
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

        twoWheelTrackingLocalizer = new TwoWheelTrackingLocalizer(hwdmap, drive);

        Intake_LeftMotor = hwdmap.get(DcMotor.class, "Intake_LeftMotor");
        Intake_RightMotor = hwdmap.get(DcMotor.class, "Intake_RightMotor");

        Outtake_LeftMotor = hwdmap.get(DcMotor.class, "Outtake_LeftMotor");
        Outtake_RightMotor = hwdmap.get(DcMotor.class, "Outtake_RightMotor");

        Intake_LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake_RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        Outtake_LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        Outtake_RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake_RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftTurret = hwdmap.get(Servo.class,"LeftTurret");
        RightTurret = hwdmap.get(Servo.class,"RightTurret");

        AngleTurret = hwdmap.get(Servo.class,"AngleTurret");

        BlockArtifact = hwdmap.get(Servo.class,"BlockArtifact");
    }

    double min_leftturret_position = 0;
    double min_rightturret_position = 0;
    double min_angleturret_position = 0;

    double max_leftturret_position = 1;
    double max_rightturret_position = 1;
    double max_angleturret_position = 1;

    public double current_rightturret_position= 0.5;
    public double current_leftturret_position = 0.5;
    public double current_angleturret_position = 0.5;

    public boolean artifact_status_blocked = false;
    boolean block_artifact_toggle = false;
    double artifact_block_position = 0.0;
    double artifact_unblock_position = 1.0;

    public double headingAngle = 0.0;
    public double x_position = 0.0;
    public double y_position = 0.0;
    public double rr_headingAngle = 0.0;

    boolean testingMode = true;
    boolean toggleButton = false;
    boolean stoggleButton = false;
    public void Run(){
        gyroscope.updateOrientation();
        aprilTagIdentification.telemetryAprilTag();
        drive.update();

        Pose2d robotPose = drive.getPoseEstimate();

        headingAngle = gyroscope.getHeading();
        x_position = robotPose.getX();
        y_position = robotPose.getY();
        rr_headingAngle = robotPose.getHeading();

        if(testingMode) {
            if (gamepad2.a) {
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
            } else if (gamepad2.b) {
                Intake_LeftMotor.setPower(-1);
                Intake_LeftMotor.setPower(-1);
            } else {
                Intake_LeftMotor.setPower(0);
                Intake_RightMotor.setPower(0);
            }

            if (gamepad2.y) {
                Outtake_LeftMotor.setPower(1);
                Outtake_RightMotor.setPower(1);
            } else if (gamepad2.x) {
                Outtake_LeftMotor.setPower(-1);
                Outtake_RightMotor.setPower(-1);
            } else {
                Outtake_RightMotor.setPower(0);
                Outtake_LeftMotor.setPower(0);
            }

            if (gamepad2.left_bumper && current_leftturret_position > min_leftturret_position && current_rightturret_position > min_rightturret_position) {
                if(!toggleButton) {
                    current_leftturret_position = current_leftturret_position - 0.1;
                    current_rightturret_position = current_rightturret_position - 0.1;
                    LeftTurret.setPosition(current_leftturret_position);
                    RightTurret.setPosition(current_rightturret_position);
                    toggleButton = true;
                }
            }else if (gamepad2.right_bumper && current_leftturret_position < max_leftturret_position && current_rightturret_position < max_rightturret_position) {
                if(!toggleButton) {
                    current_leftturret_position = current_leftturret_position + 0.1;
                    current_rightturret_position = current_rightturret_position + 0.1;
                    LeftTurret.setPosition(current_leftturret_position);
                    RightTurret.setPosition(current_rightturret_position);
                    toggleButton = true;
                }
            }else{
                toggleButton = false;
            }

            if (gamepad2.dpad_up && current_angleturret_position < max_angleturret_position) {
                if(!stoggleButton) {
                    current_angleturret_position = current_angleturret_position + 0.1;
                    AngleTurret.setPosition(current_angleturret_position);
                    stoggleButton = true;
                }
            } else if (gamepad2.dpad_down && current_angleturret_position > min_angleturret_position) {
                if(!stoggleButton) {
                    current_angleturret_position = current_angleturret_position - 0.1;
                    AngleTurret.setPosition(current_angleturret_position);
                    stoggleButton = true;
                }
            }else{
                stoggleButton = false;
            }

            if (gamepad2.dpad_right) {
                if(!block_artifact_toggle) {
                    if (!artifact_status_blocked) {
                        BlockArtifact.setPosition(artifact_block_position);
                        artifact_status_blocked = true;
                    } else {
                        BlockArtifact.setPosition(artifact_unblock_position);
                        artifact_status_blocked = false;
                    }
                    block_artifact_toggle = true;
                }
            } else {
                block_artifact_toggle = false;
            }
        }else{
            if(gamepad2.a){
                BlockArtifact.setPosition(artifact_block_position);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                artifact_status_blocked = true;
            }else if(gamepad2.y){
                BlockArtifact.setPosition(artifact_unblock_position);
                Outtake_LeftMotor.setPower(1);
                Outtake_RightMotor.setPower(1);
                Intake_LeftMotor.setPower(1);
                Intake_RightMotor.setPower(1);
                artifact_status_blocked = false;
            }

            if(gamepad2.b){
                Intake_LeftMotor.setPower(0);
                Intake_RightMotor.setPower(0);
                Outtake_LeftMotor.setPower(0);
                Outtake_RightMotor.setPower(0);
            }

            if (gamepad2.left_bumper && current_leftturret_position > min_leftturret_position && current_rightturret_position > min_rightturret_position) {
                if(!toggleButton) {
                    current_leftturret_position = current_leftturret_position - 0.1;
                    current_rightturret_position = current_rightturret_position - 0.1;
                    LeftTurret.setPosition(current_leftturret_position);
                    RightTurret.setPosition(current_rightturret_position);
                    toggleButton = true;
                }
            }else if (gamepad2.right_bumper && current_leftturret_position < max_leftturret_position && current_rightturret_position < max_rightturret_position) {
                if(!toggleButton) {
                    current_leftturret_position = current_leftturret_position + 0.1;
                    current_rightturret_position = current_rightturret_position + 0.1;
                    LeftTurret.setPosition(current_leftturret_position);
                    RightTurret.setPosition(current_rightturret_position);
                    toggleButton = true;
                }
            }else{
                toggleButton = false;
            }

            if (gamepad2.dpad_up && current_angleturret_position < max_angleturret_position) {
                if(!stoggleButton) {
                    current_angleturret_position = current_angleturret_position + 0.1;
                    AngleTurret.setPosition(current_angleturret_position);
                    stoggleButton = true;
                }
            } else if (gamepad2.dpad_down && current_angleturret_position > min_angleturret_position) {
                if(!stoggleButton) {
                    current_angleturret_position = current_angleturret_position - 0.1;
                    AngleTurret.setPosition(current_angleturret_position);
                    stoggleButton = true;
                }
            }else{
                stoggleButton = false;
            }
        }
    }
}
