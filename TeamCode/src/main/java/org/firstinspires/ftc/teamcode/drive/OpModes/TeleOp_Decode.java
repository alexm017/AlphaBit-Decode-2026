package org.firstinspires.ftc.teamcode.drive.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.drive.Structure.ChasisControl;

//FTC Decode 2026 TeleOp_Decode
@TeleOp
public class TeleOp_Decode extends LinearOpMode {

    MultipleTelemetry telemetrys;
    ChasisControl chasis_control;
    ArtifactControl artifactControl;

    int endCase = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        endCase = VarStorage.autonomous_case;

        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chasis_control = new ChasisControl(hardwareMap, gamepad1);
        artifactControl = new ArtifactControl(hardwareMap, gamepad2, telemetrys,endCase);

        while(opModeInInit()){
            telemetrys.addData("[->] Case selected ", endCase);
            artifactControl.updateAprilTag();
            telemetrys.update();
        }

        waitForStart();

        artifactControl.initServo();

        while(opModeIsActive()){
            chasis_control.Run();
            artifactControl.Run();

            if(artifactControl.manualControl){
                telemetrys.addData("[->] MANUAL CONTROL ", " ACTIVATED [<-]");
                telemetrys.addData(" ", " ");
            }

            telemetrys.addData("[Artifact] Current Left Turret Position ", artifactControl.current_leftturret_position);
            telemetrys.addData("[Artifact] Current Right Turret Position ", artifactControl.current_rightturret_position);
            telemetrys.addData("[Artifact] Current Angle Turret Position ", artifactControl.current_angleturret_position);
            telemetrys.addData("[Artifact] Current Block Position ", artifactControl.artifact_status_blocked);
            telemetrys.addData("[Artifact] Current Heading Angle ", artifactControl.headingAngle);
            telemetrys.addData("[Artifact] X Position: ", artifactControl.x_position);
            telemetrys.addData("[Artifact] Y Position: ", artifactControl.y_position);
            telemetrys.addData("[Artifact] allowedToShoot ", artifactControl.allowedToShoot);
            telemetrys.addData("[Artifact] Basket angle ", artifactControl.getBasketDirection());
            telemetrys.addData("[Artifact] Basket distance ", artifactControl.getBasketDistance());
            telemetrys.addData("[Artifact] Turret position ", artifactControl.getTurretPosition());
            telemetrys.addData("[Artifact] Turret angle ", artifactControl.getTurretAngle());
            telemetrys.addData("[Artifact] Detection Id ", artifactControl.detectionId);

            telemetrys.update();
        }
    }
}
