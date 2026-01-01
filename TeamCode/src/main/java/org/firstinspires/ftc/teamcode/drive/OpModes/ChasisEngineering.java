package org.firstinspires.ftc.teamcode.drive.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Testers.MotorTesting;
import org.firstinspires.ftc.teamcode.drive.Testers.ServoTesting;
import org.firstinspires.ftc.teamcode.drive.Structure.ChasisControl;

//FTC Decode 2026 Chasis Engineering
@TeleOp
public class ChasisEngineering extends LinearOpMode {

    MultipleTelemetry telemetrys;

    ChasisControl chasis_control;
    ServoTesting intakeTesting;
    MotorTesting motorTesting;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chasis_control = new ChasisControl(hardwareMap, gamepad1);
        motorTesting = new MotorTesting(hardwareMap, gamepad1, telemetrys);
        intakeTesting = new ServoTesting(hardwareMap, gamepad2);

        waitForStart();

        while(opModeIsActive()){
            chasis_control.Run();
            motorTesting.Run();
            intakeTesting.Run();
            telemetrys.addData("[!]Chasis on gamepad1 is ", " enabled[!]");
            telemetrys.addData("[->] To test/set servo position use ","gamepad2");
            telemetrys.addData("[->] To test motor position use ","gamepad1");

            telemetrys.addData("[Intake] Press Left Bumber/Right Bumber "," to change Current Position");
            telemetrys.addData("[Intake] Press A "," to set the Position");
            telemetrys.addData("[Intake] Intake Current Position ", intakeTesting.current_position);
            telemetrys.addData("[Intake] Intake Last Position ", intakeTesting.last_position);


            telemetrys.addData("[Motor] Press Left Bumber/Right Bumber "," to change Current Power");
            telemetrys.addData("[Motor] Press Dpad Up/Down "," to give power to motor");
            telemetrys.addData("[Motor] Motor Current Power ", motorTesting.current_power);
            telemetrys.addData("[Motor] Motor Current Position ", motorTesting.motor_position);

            telemetrys.update();
        }
    }
}
