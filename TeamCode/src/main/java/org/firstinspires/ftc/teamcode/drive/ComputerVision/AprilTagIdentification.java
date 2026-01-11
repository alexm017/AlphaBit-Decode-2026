package org.firstinspires.ftc.teamcode.drive.ComputerVision;

import android.util.Size;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//FTC Decode 2026 AprilTagIdentification
public class AprilTagIdentification {
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    MultipleTelemetry telemetry;

    public int detectionId = 0;
    public void init(HardwareMap hwdmap, MultipleTelemetry telemetrys){
        telemetry = telemetrys;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwdmap.get(WebcamName.class, "AlphaBit_Webcam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public int getPatternId(){
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.metadata != null){
                if(detection.id == 21 || detection.id == 22 || detection.id == 23){
                    detectionId = detection.id;
                }
            }
        }
        return detectionId;
    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("[->] AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

    }

    public void close(){
        visionPortal.close();
    }
}
