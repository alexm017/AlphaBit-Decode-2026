package org.firstinspires.ftc.teamcode.drive.Structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

public class ChasisInit {

    public DcMotorEx BackLeft;
    public DcMotorEx FrontRight;

    public DcMotorEx FrontLeft;
    public DcMotorEx BackRight;
    HardwareMap hwMap;

    public ChasisInit(HardwareMap ahwMap){

        hwMap = ahwMap;

        BackLeft = hwMap.get(DcMotorEx.class, "Back_Left");
        FrontRight = hwMap.get(DcMotorEx.class, "Front_Right");
        FrontLeft = hwMap.get(DcMotorEx.class, "Front_Left");
        BackRight = hwMap.get(DcMotorEx.class, "Back_Right");


        ArrayList<DcMotorEx> motors = new ArrayList<DcMotorEx>(Arrays.asList(FrontLeft, FrontRight, BackRight, BackLeft));

        for (DcMotorEx motor : motors) {

          motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

          motor.setPower(0);

          motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

          motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

    }

    public double FrontLeftVelocity(){
        return FrontLeft.getVelocity();
    }

    public double FrontRightVelocity(){
        return  FrontRight.getVelocity();
    }

    public double BackLeftVelocity(){
        return BackLeft.getVelocity();
    }

    public double BackRightVelocity(){
        return BackRight.getVelocity();
    }

    public void  setMotorPower (double BackLeftPower ,double FrontRightPower,double FrontLeftPower,double BackRightPower)
    {
        BackLeft.setPower(BackRightPower);
        FrontRight.setPower(FrontRightPower);
        FrontLeft.setPower(FrontLeftPower);
        BackRight.setPower(BackLeftPower);
    }

}
