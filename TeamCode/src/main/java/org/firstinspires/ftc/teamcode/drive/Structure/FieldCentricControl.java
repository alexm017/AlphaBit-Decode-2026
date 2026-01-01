package org.firstinspires.ftc.teamcode.drive.Structure;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;

public class FieldCentricControl {

    Gamepad gamepad;

    ChasisInit csint;

    Gyroscope gyroscope = new Gyroscope();

    public double Limit = 1;

    public FieldCentricControl(HardwareMap hwmap, Gamepad gamepad)
    {
        this.gamepad = gamepad;

        gyroscope.Init(hwmap);

        csint = new ChasisInit(hwmap);
    }

    public void Run()
    {

        gyroscope.updateOrientation();

        double x = Range.clip(gamepad.left_stick_x * 1.1,-Limit,Limit);
        double y = Range.clip(-gamepad.left_stick_y,-Limit,Limit);
        double rx = Range.clip(gamepad.right_stick_x,-Limit,Limit);

        double angle = -Math.toRadians(gyroscope.getHeading());

        double rotatedX = x * Math.cos(angle) - y * Math.sin(angle);
        double rotatedY = x * Math.sin(angle) + y * Math.cos(angle);

        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1);

        double Drive3 = (rotatedY + rotatedX - rx) / denominator;
        double Drive1 = (rotatedY - rotatedX - rx) / denominator;
        double Drive2 = (rotatedY - rotatedX + rx) / denominator;
        double Drive4 = (rotatedY + rotatedX + rx) / denominator;

        MS(Drive1, Drive2, Drive3, Drive4);
    }



    public void MS(double x1, double x2, double x3, double x4){
        csint.BackLeft.setPower(x2);
        csint.FrontRight.setPower(x1);
        csint.FrontLeft.setPower(x4);
        csint.BackRight.setPower(x3);
    }
}
