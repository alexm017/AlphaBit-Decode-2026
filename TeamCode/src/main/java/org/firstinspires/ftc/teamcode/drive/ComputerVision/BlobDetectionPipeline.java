package org.firstinspires.ftc.teamcode.drive.ComputerVision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class BlobDetectionPipeline extends OpenCvPipeline {
    static int first = 100;
    static int second = 40;
    static int third = 85;
    private final Scalar lowerBlue = new Scalar(first, second, third);

    private final Scalar upperBlue = new Scalar(140, 255, 255);

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    private double angle = 0;
    private Point[] boxPoints = null;
    public double y2;
    public boolean isOrientationToRight = false;
    public double distanceBetweenTwoPoints = 0.00;
    public double centerOfObjectX = 0.00;
    public double centerOfObjectY = 0.00;
    public double x1,x2,x3,x4;
    public double y1,y3,y4;
    public double topLeftX, bottomLeftX;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);

        contours.clear();
        Imgproc.findContours(mask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            double maxArea = 0;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);

                Point[] box = new Point[4];
                rect.points(box);

                x1 = box[0].x;
                x2 = box[1].x;
                x3 = box[2].x;
                x4 = box[3].x;

                bottomLeftX = x1;
                topLeftX = x4;

                y1 = box[0].y;
                y2 = box[1].y;
                y3 = box[2].y;
                y4 = box[3].y;

                distanceBetweenTwoPoints = Math.sqrt(Math.pow(box[3].x - box[2].x, 2) + Math.pow(box[3].y - box[2].y, 2));

                if(distanceBetweenTwoPoints > 105){
                    isOrientationToRight = true;
                    centerOfObjectY = y2;
                }else{
                    isOrientationToRight = false;
                    centerOfObjectY = y2;
                }

                centerOfObjectX = box[1].x - (Math.abs(box[3].x - box[1].x))/2;

                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, box[i], box[(i + 1) % 4], new Scalar(255, 0, 0), 2);
                }

                if(!isOrientationToRight) {
                    angle = -(90 - rect.angle);
                }else{
                    angle = rect.angle;
                }

                Imgproc.putText(input, String.format("CY: %.2f deg", centerOfObjectY),
                        new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0, 255, 0), 1);
                Imgproc.putText(input, String.format("CX: %.2f deg", centerOfObjectX),
                        new Point(10, 55), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0, 255, 0), 1);

                boxPoints = box;
            }
        }
        return input;
    }

    public double getDistanceToSample(){
        double distanceInPixels = 140.00/8.00;
        double distanceInCm = (centerOfObjectY)/distanceInPixels;
        return distanceInCm;
    }

    public double getXDistanceToSample(){
        double maxDistanceFromCenter = 7.00;
        double distanceInPixels = 120.00/maxDistanceFromCenter;
        double distanceInCm = 0.00;
        if(centerOfObjectX <= 120){
            distanceInCm = -(centerOfObjectX/distanceInPixels);
        }else if(centerOfObjectX > 120){
            distanceInCm = (centerOfObjectX-120)/distanceInPixels;
        }
        return distanceInCm;
    }

    public double getAngle() {
        return angle;
    }
    public Point[] getBoxPoints() {
        return boxPoints;
    }
}
