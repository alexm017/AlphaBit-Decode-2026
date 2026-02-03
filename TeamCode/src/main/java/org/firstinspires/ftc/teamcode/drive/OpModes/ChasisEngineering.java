package org.firstinspires.ftc.teamcode.drive.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
    GoBildaPinpointDriver pinpointDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chasis_control = new ChasisControl(hardwareMap, gamepad1);
        motorTesting = new MotorTesting(hardwareMap, gamepad1, telemetrys);
        intakeTesting = new ServoTesting(hardwareMap, gamepad2);
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        waitForStart();

        while(opModeIsActive()){
            chasis_control.Run();
            motorTesting.Run();
            intakeTesting.Run();

            pinpointDriver.update();
            telemetry.addData("rawX", pinpointDriver.getEncoderX());
            telemetry.addData("rawY", pinpointDriver.getEncoderY());
            telemetry.addData("status", pinpointDriver.getDeviceStatus());

            telemetrys.addData("[!] Chassis on gamepad1 is ", " enabled [!]");
            telemetrys.addData("[!] Set the name in DriverHub to the Servo you want to Test as ", "ServoTest [!]");
            telemetrys.addData("[!] Set the name in DriverHub to the Motor you want to Test as ", "MotorTest [!]");

            telemetrys.addData(" ", " ");

            telemetrys.addData("[->] To test/set Servo position use ", "gamepad2");
            telemetrys.addData("[->] To test Motor position use ", "gamepad1");

            telemetrys.addData(" ", " ");

            telemetrys.addData("[Servo] Press Left Bumper/Right Bumper ", " to change Current Position");
            telemetrys.addData("[Servo] Press A ", " to set the Position");
            telemetrys.addData("[Servo] Intake Current Position ", intakeTesting.current_position);
            telemetrys.addData("[Servo] Intake Last Position ", intakeTesting.last_position);

            telemetrys.addData(" ", " ");

            telemetrys.addData("[Motor] Press Left Bumper/Right Bumper ", " to change Current Power");
            telemetrys.addData("[Motor] Press Dpad Up/Down ", " to give power to motor");
            telemetrys.addData("[Motor] Motor Current Power ", motorTesting.current_power);
            telemetrys.addData("[Motor] Motor Current Position ", motorTesting.motor_position);

            telemetrys.update();
        }
    }
}
