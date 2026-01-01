package org.firstinspires.ftc.teamcode.drive.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.drive.Structure.ChasisControl;

//FTC Decode 2026 TeleOp_Decode
@TeleOp
public class TeleOp_Decode extends LinearOpMode {

    MultipleTelemetry telemetrys;
    ChasisControl chasis_control;
    ArtifactControl artifactControl;

    int endCase = 0;
    boolean buttonTrigger = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chasis_control = new ChasisControl(hardwareMap, gamepad1);

        while(opModeInInit() && !gamepad1.dpad_left){
            if(gamepad1.dpad_down && endCase < 3){
                if(!buttonTrigger) {
                    endCase = endCase + 1;
                    buttonTrigger = true;
                }
            }else if(gamepad1.dpad_right && endCase > 0){
                if(!buttonTrigger) {
                    endCase = endCase - 1;
                    buttonTrigger = true;
                }
            }else{
                buttonTrigger = false;
            }

            telemetrys.addData("[+] 0. Red Audience, 1. Blue Audience, 2. Red Basket, 3. Blue Basket","[+]");
            telemetrys.addData("[->] Case : ", endCase);
            telemetrys.update();
        }

        artifactControl = new ArtifactControl(hardwareMap, gamepad2, telemetrys,endCase);

        while(opModeInInit()){
            telemetrys.addData("[->] Case selected ", endCase);
            artifactControl.updateAprilTag();
            telemetrys.update();
        }

        waitForStart();

        while(opModeIsActive()){
            chasis_control.Run();
            artifactControl.Run();

            telemetrys.addData("[Artifact] Current Left Turret Position ", artifactControl.current_leftturret_position);
            telemetrys.addData("[Artifact] Current Right Turret Position ", artifactControl.current_rightturret_position);
            telemetrys.addData("[Artifact] Current Angle Turret Position ", artifactControl.current_angleturret_position);
            telemetrys.addData("[Artifact] Current Block Position ", artifactControl.artifact_status_blocked);
            telemetrys.addData("[Artifact] Current Heading Angle ", artifactControl.headingAngle);
            telemetrys.addData("[Artifact] X Position: ", artifactControl.x_position);
            telemetrys.addData("[Artifact] Y Position: ", artifactControl.y_position);
            telemetrys.addData("[Artifact] RR Heading Angle: ", artifactControl.rr_headingAngle);

            telemetrys.update();
        }
    }
}
