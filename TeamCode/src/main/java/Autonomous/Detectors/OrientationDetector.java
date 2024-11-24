package Autonomous.Detectors;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class OrientationDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    //region for reference only (0,0) in top left
    static final Rect SCREENSIZEBOX = new Rect( //make this the correct area
            new Point(0, 60),
            new Point(320, 240));
    //endregion

    //region roi actual boxes
    static final Rect LEFT_ROI = new Rect( //make this the correct area
            new Point(100, 60),
            new Point(170, 120));
    static final Rect CENTER_ROI = new Rect( //make this the correct area
            new Point(230, 70),
            new Point(292, 170));
    //endregion

    int centerLineOffset = 0;
    Vector2D xLine, yLine, zLine;

    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public OrientationDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        //region FOR REFERENCE
        /*Scalar lowHSVYELLOW= new Scalar(10, 0, 0);
        Scalar highHSVYELLOW = new Scalar(25, 255, 255);

        Scalar lowHSVBLUE = new Scalar(70, 100, 0);
        Scalar highHSVBLUE = new Scalar(110, 255, 200);

        Scalar lowHSVREDD = new Scalar(10, 0, 0);
        Scalar highHSVREDD = new Scalar(175, 255, 255);

        Scalar lowHSVGREEN= new Scalar(45, 0, 0);
        Scalar highHSVGREEN = new Scalar(76, 255, 255);

        Scalar lowHSVPURPLE= new Scalar(120, 50, 50);
        Scalar highHSVPURPLE = new Scalar(170, 255, 255);

        Scalar lowHSVWHITE= new Scalar(0, 0, 80);
        Scalar highHSVWHITE = new Scalar(180, 30, 255);*/
        //endregion
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //GET COLORS
        Scalar lowHSVRED = new Scalar(-20, 60, 30);
        Scalar highHSVRED = new Scalar(7, 255, 255);
        Core.inRange(mat, lowHSVRED, highHSVRED, mat);

        //GET CONTOURS
        Mat canMat = new Mat();
        Imgproc.GaussianBlur(mat, canMat, new Size(5.0, 15.0), 0.00);
        Imgproc.Canny(canMat, canMat, 100, 200);

        Mat dilatedOutput = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));  // Define a kernel
        //Imgproc.dilate(canMat, dilatedOutput, kernel);

        Mat erodedOutput = new Mat();
        Imgproc.erode(canMat, erodedOutput, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //FILTER CONTOURS
        MatOfPoint recordMat = new MatOfPoint();
        double recordArea = 0;
        for(MatOfPoint cont: contours)
        {
            if(Imgproc.contourArea(cont)>recordArea)
            {
                recordArea = Imgproc.contourArea(cont);
                recordMat = cont;
            }
        }
        List<MatOfPoint> recordMatList = new ArrayList<>();
        recordMatList.add(recordMat);
        Mat contourImage = new Mat(canMat.size(), CvType.CV_8UC3, new Scalar(0, 0, 0)); // Create a black image
        Imgproc.drawContours(contourImage, recordMatList, -1, new Scalar(255, 255, 255), 1);  // Draw contours in red color

        //EXTRA EXTRA CONTOUR REFINEMENT
        List<Point> pts = new ArrayList<>();
        for (int y = 0; y < contourImage.rows(); y++) {
            for (int x = 0; x < contourImage.cols(); x++) {
                if (contourImage.get(y, x)[0] > 0)
                {
                    pts.add(new Point(x, y));
                }
            }
        }

        //DRAW THE OUTLINE AND OUTPUT
        if(pts.size()>3)
        {
            //Imgproc.cvtColor(contourImage, contourImage, Imgproc.COLOR_GRAY2RGB);
            MatOfInt hullIndices = new MatOfInt();
            Imgproc.convexHull(new MatOfPoint(pts.toArray(new Point[0])), hullIndices);
            List<Integer> hull = new ArrayList<>();
            for (int i = 0; i < hullIndices.rows(); i++) {
                hull.add((int) hullIndices.get(i, 0)[0]);
            }
            List<Point> hullPoints = new ArrayList<>();
            for (int index : hull) {
                hullPoints.add(pts.get(index));
            }
            MatOfPoint[] hullArray = new MatOfPoint[1];
            hullArray[0] = new MatOfPoint();
            hullArray[0].fromList(hullPoints);
            Mat temp = new Mat();
            Imgproc.polylines(contourImage, java.util.Arrays.asList(hullArray), true, new Scalar(0, 0, 255), 2);
        }

        //FIND X Y Z VECTORS


        return contourImage;
    }

    public double distance(Point P, Point Q)
    {
        return Math.sqrt(((Q.x-P.x)*(Q.x-P.x)) + ((Q.y-P.y)*(Q.y-P.y)));
    }

    public int centerLineOffset()
    {
        return centerLineOffset;
    }

    public Vector2D getxLine()
    {
        return xLine;
    }
    public Vector2D getyLine() {
        return yLine;
    }
    public Vector2D getzLine() {
        return zLine;
    }
}

////region trashy simplification
//        /*
//        List<Integer> ranges = new ArrayList<>();
//        int count = 1;
//        int record = 0;
//        int currCount = 0;
//        int recordStart = 0;
//        List<Point> correspondingPts = new ArrayList<>();
//        if(!nonZeroPoints.empty())
//        {
//            //find all seperate contour ranges
//            Point lastP = new Point(nonZeroPoints.get(0, 0)[0], nonZeroPoints.get(0, 0)[1]);
//            correspondingPts.add(lastP);
//            for (int i = 1; i < nonZeroPoints.rows(); i++) {
//                Point p = new Point(nonZeroPoints.get(i, 0)[0], nonZeroPoints.get(i, 0)[1]);
//                correspondingPts.add(p);
//
//                double pointsDistance = Math.abs(distance(p, lastP));
//                lastP = p;
//                if(pointsDistance<20)
//                {
//                    count++;
//                }
//                else {
//                    ranges.add(count);
//                    currCount+=count;
//                    if(count>record)
//                    {
//                        record = count;
//                        recordStart = currCount-count;
//                    }
//                    count = 1;
//                }
//            }
//
//            //get the points in the longest contour
//            for (int i = recordStart; i < Math.min(recordStart + record, correspondingPts.size()); i++)
//            {
//                pts.add(correspondingPts.get(i));
//            }
//
//        }*/
//        //endregion