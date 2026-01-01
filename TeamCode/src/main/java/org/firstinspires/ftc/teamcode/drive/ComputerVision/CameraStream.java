package org.firstinspires.ftc.teamcode.drive.ComputerVision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Camera Stream")
public class CameraStream extends LinearOpMode {
    private CameraServer server;
    private OpenCvWebcam webcam;
    private Mat rgbMat = new Mat();

    @Override
    public void runOpMode() {
        server = new CameraServer(8082);
        new Thread(server).start();

        int cameraMonitorId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "AICam"),
                cameraMonitorId
        );

        webcam.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Mat rotated = new Mat();
                Core.rotate(input, rotated, Core.ROTATE_90_CLOCKWISE);

                Imgproc.cvtColor(rotated, rgbMat, Imgproc.COLOR_BGR2RGB);

                MatOfByte matOfByte = new MatOfByte();
                Imgcodecs.imencode(".jpg", rgbMat, matOfByte);
                server.updateFrame(matOfByte.toArray());

                rotated.release();

                return input;
            }
        });

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Streaming at http://192.168.49.1:8082");
        telemetry.update();

        double nothing=0;

        waitForStart();

        while (opModeIsActive()) {
            nothing=1;
        }

        webcam.stopStreaming();
        server.stop();
    }
}