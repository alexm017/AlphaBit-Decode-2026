package org.firstinspires.ftc.teamcode.drive.Structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ChasisControl {

    Gamepad gamepad;

    ChasisInit chasis_init;
    static double Limit = 1;
    public double multiplier = 1;

    public double FrontLeft;
    public double FrontRight;
    public double BackLeft;
    public double BackRight;

    public ChasisControl(HardwareMap hwmap, Gamepad gamepad)
    {
        this.gamepad = gamepad;
        chasis_init = new ChasisInit(hwmap);
    }

    public void Run()
    {

        double rx = Range.clip(gamepad.right_stick_x * 1.1,-Limit,Limit);
        double y = -Range.clip(gamepad.left_stick_y,-Limit,Limit);
        double x = Range.clip(gamepad.left_stick_x,-Limit,Limit);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double Drive3 = (y + x - rx) / denominator;
        double Drive1 = (y - x - rx) / denominator;
        double Drive2 = (y - x + rx) / denominator;
        double Drive4 = (y + x + rx) / denominator;

        FrontLeft = Drive4;
        FrontRight = Drive1;
        BackLeft = Drive2;
        BackRight = Drive3;

        ChasisPower(Drive1 * multiplier, Drive2 * multiplier, Drive3 * multiplier, Drive4 * multiplier);
    }

    public void ChasisPower(double x1, double x2, double x3, double x4){
        chasis_init.BackLeft.setPower(x2);
        chasis_init.FrontRight.setPower(x1);
        chasis_init.FrontLeft.setPower(x4);
        chasis_init.BackRight.setPower(x3);
    }

}
