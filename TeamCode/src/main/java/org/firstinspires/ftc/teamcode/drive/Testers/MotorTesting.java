package org.firstinspires.ftc.teamcode.drive.Testers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//FTC Decode 2026 MotorTesting
public class MotorTesting {
    DcMotorEx MotorTest;

    MultipleTelemetry multipleTelemetry;

    Gamepad gmpd;

    public MotorTesting(HardwareMap hwdmap, Gamepad gmpdd, MultipleTelemetry multipleTelemetrys){
        multipleTelemetry = multipleTelemetrys;
        gmpd = gmpdd;
        MotorTest = hwdmap.get(DcMotorEx.class, "Back_Left");

        MotorTest.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double motor_position;
    public double current_power = 0.5;
    boolean triggerButton = false;

    public void Run(){
        motor_position = MotorTest.getCurrentPosition();

        if(gmpd.left_bumper){
            if(!triggerButton) {
                current_power = current_power - 0.05;
                triggerButton = true;
            }
        }else if(gmpd.right_bumper){
            if(!triggerButton) {
                current_power = current_power + 0.05;
                triggerButton = true;
            }
        }else{
            triggerButton = false;
        }

        if(gmpd.dpad_up){
            MotorTest.setPower(current_power);
        }else if(gmpd.dpad_down){
            MotorTest.setPower(-current_power);
        }else{
            MotorTest.setPower(0);
        }
    }
}
