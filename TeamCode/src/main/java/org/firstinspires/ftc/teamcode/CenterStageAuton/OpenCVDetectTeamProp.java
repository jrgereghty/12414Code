package org.firstinspires.ftc.teamcode.CenterStageAuton;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
@Config
public class OpenCVDetectTeamProp extends OpenCvPipeline {
    public static final Scalar green = new Scalar(0, 255, 0);
    public static final Scalar blue = new Scalar(0, 0, 255);
    public static final Scalar red = new Scalar(255, 0, 0);
    public static final Scalar pink = new Scalar(255, 0, 142);
    public static final Scalar white = new Scalar(255, 255, 255);
    public static final Scalar black = new Scalar(0, 0, 0);
    public static double webcamSplitDist = 160;
    public static boolean isDetected = false;
    public static double minArea = 40;
    public static double maxAreaCon = 2000;
    public static double maxWidth = 70;
    public static double maxHeight = 70;
    public static double minWidth = 20;
    public static double minHeight = 27;
    public int[] lowerColor = {0, 0, 0};
    public int[] upperColor = {255, 255, 255};

    public static int centerX;
    public static int centerY;
    public static int xDist;
    public static int yDist;
    public static double thetaX;

    public static int rectArea;
    Rect zone1 = new Rect(0,6, 60, 100);

    Telemetry telemetry;
    // Make a Constructor
    public OpenCVDetectTeamProp(Telemetry telemetry, int[] lowerColor, int[] upperColor) {
        this.telemetry = telemetry;
        System.arraycopy(lowerColor, 0, this.lowerColor, 0, this.lowerColor.length);
        System.arraycopy(upperColor, 0, this.upperColor, 0, this.upperColor.length);


    }

    @Override
    public void init(Mat input) {
    }

    @Override
    public Mat processFrame(Mat frame) {
//            telemetry.addLine("running");

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV_FULL);
        Rect box2Rect = new Rect(new Point(70, 20), new Point(90, 50));
        Rect box1Rect = new Rect(new Point(230, 140), new Point(270, 80));

        Mat fakehsv = hsv;

        //EXPERIMENTAL ZONE
       // Mat area1 = new Mat(hsv, new Rect(0, 60, 60, 180));
        Mat area1 = new Mat(hsv, new Rect(new Point(0,60), new Point(60,160)));
        //Mat area1 = new Mat(hsv, new Rect(0,6, 60, 100));
        Mat area2 = new Mat(hsv, new Rect(new Point(135,60), new Point(180,160)));
        Mat area3 = new Mat(hsv, new Rect(new Point(190,60), new Point(240,160)));
        //Mat area4 = new Mat(fakehsv, new Rect(new Point(190,60), new Point(240,160)));
        //Mat area3 = new Mat(hsv, new Rect(x, y, width, height));
        Mat area4 = new Mat(fakehsv, zone1);
        Imgproc.rectangle(hsv, new Point(0,160), new Point(320,240), black, Core.FILLED);
        Imgproc.rectangle(hsv, new Point(0,0), new Point(320,80), black, Core.FILLED);
        Imgproc.rectangle(hsv, new Point(60,0), new Point(130,240), black, Core.FILLED);
        Imgproc.rectangle(hsv, new Point(190,0), new Point(240,240), black, Core.FILLED);



        //END ZONE



//        Mat box1 = new Mat(frame, box1Rect);
//        Mat box2 = new Mat(frame, box2Rect);
//
//        Imgproc.cvtColor(frame, box1, Imgproc.COLOR_RGB2HSV_FULL);
//        Imgproc.cvtColor(frame, box2, Imgproc.COLOR_RGB2HSV_FULL);
//
//        Imgproc.rectangle(frame, box1Rect, new Scalar(0, 255, 0), 2);
//        Imgproc.rectangle(frame, box2Rect, new Scalar(0, 255, 0), 2);

        Mat mask = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();

        //EXPRIMENTAL ZONE
        Mat maskzone1 = new Mat();
        Mat mask1zone1 = new Mat();
        Mat mask2zone1 = new Mat();

        Mat maskzone2 = new Mat();
        Mat mask1zone2 = new Mat();
        Mat mask2zone2 = new Mat();

        Mat maskzone3 = new Mat();
        Mat mask1zone3 = new Mat();
        Mat mask2zone3 = new Mat();
        //END ZONE

        Scalar secondLower;
        Scalar secondUpper;
        if(upperColor[0] > 180){
            int offsetForNewRanges = upperColor[0]-180;
            secondLower = new Scalar(0, lowerColor[1], lowerColor[2]);
            secondUpper = new Scalar(offsetForNewRanges, upperColor[1], upperColor[2]);
            telemetry.addData("secondLower", secondLower);
            telemetry.addData("secondUpper", secondUpper);


            Core.inRange(hsv, new Scalar(lowerColor[0], lowerColor[1], lowerColor[2]), new Scalar(upperColor[0], upperColor[1], upperColor[2]), mask1);//Uses originals for the filter
            Core.inRange(hsv, secondLower, secondUpper, mask2);//Uses the adjusted hue
            Core.bitwise_or(mask2, mask1, mask);//Combines the masks including all values of 255





        }
        else{
            Core.inRange(hsv, new Scalar(lowerColor[0], lowerColor[1], lowerColor[2]), new Scalar(upperColor[0], upperColor[1], upperColor[2]), mask);
            telemetry.addLine("Didn't do anything");
        }


        // Find the contours of the objects in the frame that are within the mask
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskzone1, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskzone2, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskzone3, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.findContours();

        // Draw the contours on the original frame
//      Imgproc.drawContours(hsv, contours, -1, white, 10);
        telemetry.addData("Contours found: ", contours.size());
        telemetry.addData("Con found: ", contours.size());

        // Find the contour with the largest area
        double maxArea = 0;
        MatOfPoint maxContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect boundingBox = Imgproc.boundingRect(contour);
            int w = boundingBox.width;
            int h = boundingBox.height;
            if (area > maxArea && area >= minArea && w >= minWidth && h >= minHeight) {
                maxArea = area;
                maxContour = contour;
            }
        }

        // Draw a bounding box around the object with the largest area
        if (maxContour != null) {
            isDetected = true;
            telemetry.addLine("Contours found!!! :)");
            Rect boundingBox = Imgproc.boundingRect(maxContour);
            int x = boundingBox.x;
            int y = boundingBox.y;
            int w = boundingBox.width;
            int h = boundingBox.height;

            rectArea = w*h;

            Imgproc.rectangle(hsv, new Point(x, y), new Point(x + w, y + h), green, 2);



            // Display the coordinates of the center of the bounding box
            int center_x = x + w/2;
            int center_y = y + h/2;

            centerX = center_x;
            centerY = center_y;
            xDist = center_x - (frame.width()/2);
            yDist = center_y - (frame.height()/2);
            thetaX = (double)((Math.abs(xDist)-160)/4)+30;
            if(xDist > 0){
                thetaX *= -1;
            }

//            int circRad;
//            if(Math.abs(x-(x+w)) > Math.abs(y-(y+h))){
//                circRad = Math.abs(x-(x+w))/2;
//            }
//            else{
//                circRad = Math.abs(y-(y+h))/2;
//            }

//                Imgproc.circle(hsv, new Point(center_x, center_y), circRad, red, 2);
//                Imgproc.circle(hsv, new Point(center_x, center_y), 3, red, -1);


            Imgproc.line(hsv, new Point(x, y), new Point(Math.abs(x + w), Math.abs(y + h)), red, 1);
            Imgproc.line(hsv, new Point(x, Math.abs(y + h)), new Point(Math.abs(x + w), y), red, 1);

//                Imgproc.putText(hsv, "(" + center_x + ", " + center_y + ")", new Point(frame.width(), frame.height() - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            telemetry.addData("Center X: ", center_x + " Center Y: " + center_y);
        }
        else{
            isDetected = false;
            telemetry.addLine("None found :(");
        }
        telemetry.update();
        Imgproc.rectangle(hsv, new Point(0,80), new Point(60,160), pink, 2);
        Imgproc.rectangle(hsv, new Point(130,60), new Point(190,160), pink, 2);
        Imgproc.rectangle(hsv, new Point(260,80), new Point(320,160), pink, 2);

        //return area4;

        return hsv;
    }
}