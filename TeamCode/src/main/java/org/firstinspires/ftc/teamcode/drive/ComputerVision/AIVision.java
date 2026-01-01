package org.firstinspires.ftc.teamcode.drive.ComputerVision;

import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.Interpreter;

import android.graphics.Canvas;
import android.util.Size;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "AI Vision", group = "Competition")
public class AIVision extends LinearOpMode {

    private static final String MODEL_FILE = "best_float16.tflite";
    private static final int MODEL_SIZE = 320;
    private static final float CONF_THRESHOLD = 0.6f;
    private static final float IOU_THRESHOLD = 0.5f;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        YoloProcessor frameProcessor = new YoloProcessor(hardwareMap.appContext.getAssets());

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "AICam"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(320, 240))
                .addProcessor(frameProcessor)
                .build();

        waitForStart();

        try {
            while (opModeIsActive()) {
                List<Detection> detections = frameProcessor.getLatestDetections();
                telemetry.addData("Detections", detections.size());
                int bestConfidence = -1;
                double confCounter=0;
                for(int i=0; i<detections.size(); i++){
                    Detection d = detections.get(i);
                    if(confCounter < (d.confidence * 100)){
                        confCounter = (d.confidence * 100);
                        bestConfidence = i;
                    }
                }
                if(detections.size() > 0){
                    Detection d = detections.get(bestConfidence);
                    telemetry.addData(String.format("Obj %d", bestConfidence),
                            "%s %.1f%% | W: %.1f H: %.1f Angle: %.1f",
                            d.className(),
                            d.confidence * 100,
                            d.width,
                            d.height);
                }

                /*for (int i = 0; i < detections.size(); i++) {
                    Detection d = detections.get(i);
                    telemetry.addData(String.format("Obj %d", i),
                            "%s %.1f%% | W: %.1f H: %.1f",
                            d.className(),
                            d.confidence * 100,
                            d.width,
                            d.height);
                }*/
                telemetry.update();
            }
        } finally {
            visionPortal.close();
            frameProcessor.close();
        }
    }

    static class YoloProcessor implements VisionProcessor {
        private final Interpreter tflite;
        private final Object syncLock = new Object();
        private List<Detection> detections = new ArrayList<>();

        private final float[][][] outputBuffer = new float[1][7][2100];
        private final ByteBuffer inputBuffer;
        private final Mat resizedMat = new Mat();
        private final Mat rgbMat = new Mat();

        public YoloProcessor(AssetManager assets) {
            try {
                Interpreter.Options options = new Interpreter.Options();
                options.setNumThreads(4);
                options.setUseXNNPACK(true);
                tflite = new Interpreter(loadModelFile(assets), options);

                inputBuffer = ByteBuffer.allocateDirect(MODEL_SIZE * MODEL_SIZE * 3 * 4);
                inputBuffer.order(ByteOrder.nativeOrder());
            } catch (IOException e) {
                throw new RuntimeException("Model loading failed", e);
            }
        }

        private ByteBuffer loadModelFile(AssetManager assets) throws IOException {
            try (AssetFileDescriptor afd = assets.openFd(MODEL_FILE)) {
                try (FileInputStream fis = new FileInputStream(afd.getFileDescriptor())) {
                    FileChannel channel = fis.getChannel();
                    return channel.map(FileChannel.MapMode.READ_ONLY, afd.getStartOffset(), afd.getDeclaredLength());
                }
            }
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat rotated = new Mat();
            Core.rotate(frame, rotated, Core.ROTATE_90_CLOCKWISE);

            Imgproc.resize(rotated  , resizedMat, new org.opencv.core.Size(MODEL_SIZE, MODEL_SIZE));
            Imgproc.cvtColor(resizedMat, rgbMat, Imgproc.COLOR_BGR2RGB);
            rgbMat.convertTo(rgbMat, CvType.CV_32FC3, 1.0 / 255.0);

            float[] floatBuffer = new float[MODEL_SIZE * MODEL_SIZE * 3];
            rgbMat.get(0, 0, floatBuffer);
            inputBuffer.rewind();
            inputBuffer.asFloatBuffer().put(floatBuffer);

            tflite.run(inputBuffer, outputBuffer);

            List<Detection> newDetections = processOutput(frame.width(), frame.height());
            synchronized (syncLock) {
                detections = newDetections;
            }
            return null;
        }

        private List<Detection> processOutput(int frameWidth, int frameHeight) {
            List<Detection> rawDetections = new ArrayList<>();
            final float[] xCenterArray = outputBuffer[0][0];
            final float[] yCenterArray = outputBuffer[0][1];
            final float[] widthArray = outputBuffer[0][2];
            final float[] heightArray = outputBuffer[0][3];
            final float[] class0Array = outputBuffer[0][4];
            final float[] class1Array = outputBuffer[0][5];
            final float[] class2Array = outputBuffer[0][6];

            for (int j = 0; j < 2100; j++) {
                float maxScore = Math.max(class0Array[j], Math.max(class1Array[j], class2Array[j]));
                if (maxScore < CONF_THRESHOLD) continue;

                int classId = 0;
                if (maxScore == class1Array[j]) classId = 1;
                else if (maxScore == class2Array[j]) classId = 2;

                rawDetections.add(new Detection(
                        xCenterArray[j] * frameWidth,
                        yCenterArray[j] * frameHeight,
                        widthArray[j] * frameWidth,
                        heightArray[j] * frameHeight,
                        maxScore,
                        classId
                ));
            }
            return nms(rawDetections);
        }

        private List<Detection> nms(List<Detection> detections) {
            List<Detection> results = new ArrayList<>(10);
            detections.sort((d1, d2) -> Float.compare(d2.confidence, d1.confidence));

            while (!detections.isEmpty()) {
                Detection best = detections.remove(0);
                results.add(best);
                detections.removeIf(d -> iou(best, d) > IOU_THRESHOLD);
            }
            return results;
        }

        private float iou(Detection a, Detection b) {
            float intersectionLeft = Math.max(a.x1, b.x1);
            float intersectionTop = Math.max(a.y1, b.y1);
            float intersectionRight = Math.min(a.x2, b.x2);
            float intersectionBottom = Math.min(a.y2, b.y2);

            if (intersectionRight < intersectionLeft || intersectionBottom < intersectionTop)
                return 0.0f;

            float intersectionArea = (intersectionRight - intersectionLeft) * (intersectionBottom - intersectionTop);
            float areaA = a.width * a.height;
            float areaB = b.width * b.height;

            return intersectionArea / (areaA + areaB - intersectionArea);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int width, int height, float scale, float density, Object tag) {}

        public List<Detection> getLatestDetections() {
            synchronized (syncLock) {
                return new ArrayList<>(detections);
            }
        }

        public void close() {
            tflite.close();
            resizedMat.release();
            rgbMat.release();
        }
    }

    static class Detection {
        final float x1, y1, x2, y2;
        final float confidence;
        final int classId;
        public final float width;
        public final float height;

        public Detection(float cx, float cy, float w, float h, float conf, int cls) {
            width = w;
            height = h;
            x1 = cx - width/2;
            y1 = cy - height/2;
            x2 = cx + width/2;
            y2 = cy + height/2;
            confidence = conf;
            classId = cls;
        }

        String className() {
            switch (classId) {
                case 0: return "Yellow";
                case 1: return "Blue";
                case 2: return "Red";
                default: return "Unknown";
            }
        }
    }
}