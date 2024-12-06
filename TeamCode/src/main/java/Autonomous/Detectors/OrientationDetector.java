package Autonomous.Detectors;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OrientationDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public OrientationDetector(Telemetry t) { telemetry = t; }

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
    Vector2D xLine, yLine, zLine;
    double objectDistance, centerLineOffset = 0, objectAngle;
    double recordArea, currArea = 0;
    double maxX, maxY,minX,minY;
    @Override
    public Mat processFrame(Mat input) {
        //region GET COLORS
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVRED = new Scalar(-20, 60, 30);
        Scalar highHSVRED = new Scalar(7, 255, 255);
        Core.inRange(mat, lowHSVRED, highHSVRED, mat);
        //endregion

        //region GET CONTOURS
        Mat canMat = new Mat();
        Imgproc.GaussianBlur(mat, canMat, new Size(5.0, 15.0), 0.00);
        Imgproc.Canny(canMat, canMat, 10, 10); //100,200

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));  // Define a kernel

        Mat erodedOutput = new Mat();
        Imgproc.erode(canMat, erodedOutput, kernel);

        Mat dilatedOutput = new Mat();
        Imgproc.dilate(canMat, dilatedOutput, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //endregion

        //region FILTER OUT EXTERNAL CONTOURS
        MatOfPoint recordMat = new MatOfPoint();
        recordArea = 0;
        for(MatOfPoint cont: contours)
        {
            if(Imgproc.contourArea(cont)>recordArea)
            {
                recordArea = Imgproc.contourArea(cont);
                recordMat = cont;
            }
        }
        //setExtremes(recordMat.toList());
        List<MatOfPoint> recordMatList = new ArrayList<>();
        recordMatList.add(recordMat);

        List<MatOfPoint> innerConts = new ArrayList<>();
        innerConts.add(recordMat);
        /*for(MatOfPoint cont: contours)
        {
            if(isInside(cont.toList()))
            {
                innerConts.add(cont);
            }
        }*/
        Mat contourImage = new Mat(canMat.size(), CvType.CV_8UC3, new Scalar(0, 0, 0)); // Create a black image
        Imgproc.drawContours(contourImage, innerConts, -1, new Scalar(255, 255, 255), 1);  // Draw contours in red color
        //endregion

        //region GET ALL CONTOUR POINTS
        /*Imgproc.threshold(contourImage, contourImage, 100, 255, Imgproc.THRESH_BINARY);
        List<Point> pts = new ArrayList<>();
        for (int y = 0; y < contourImage.rows(); y++) {
            for (int x = 0; x < contourImage.cols(); x++) {
                if (contourImage.get(y, x)[0]!= 0)
                {
                    pts.add(new Point(x, y));
                }
            }
        }*/
        //endregion

        if(!contours.isEmpty())
        {
            List<Point> pts = new ArrayList<>(contours.get(0).toList());
            //region FIND HULL POINTS
            /*MatOfInt hullIndices = new MatOfInt();
            Imgproc.convexHull(new MatOfPoint(pts.toArray(new Point[0])), hullIndices);
            List<Integer> hull = new ArrayList<>();
            for (int i = 0; i < hullIndices.rows(); i++) {
                hull.add((int) hullIndices.get(i, 0)[0]);
            }
            List<Point> hullPoints = new ArrayList<>();
            for (int index : hull) {
                hullPoints.add(pts.get(index));
            }*/
            //endregion

            //region ANGLE BASED HULL SIMPLIFICATION
            Point prevPts = pts.get(1);
            double previousAngle = angleBtwn(pts.get(0), prevPts);
            for(int i=2; i<pts.size(); i++)
            {
                double currAngle = angleBtwn(prevPts, pts.get(i));
                if(Math.abs(currAngle-previousAngle)>30) //big change
                {
                    previousAngle = currAngle;
                }
                else { //remove redundant
                    i--;
                    pts.remove(i);
                    prevPts = pts.get(i);
                    previousAngle = angleBtwn(pts.get(i-1), pts.get(i));
                }
            }
            //endregion

            //region DISTANCE BASED HULL SIMPLIFICATION
            /*prevPts = hullPoints.get(1);
            double previousDistance = distance(hullPoints.get(0), prevPts);
            for(int i=2; i<hullPoints.size(); i++)
            {
                double currDistance = distance(prevPts, hullPoints.get(i));
                if(Math.abs(currDistance-previousDistance)>50) //big change
                {
                    previousDistance = currDistance;
                }
                else { //remove redundant
                    i--;
                    hullPoints.remove(i);
                    prevPts = hullPoints.get(i);
                    previousDistance= distance(hullPoints.get(i-1), hullPoints.get(i));
                }
            }*/
            //endregion

            //repeat to get only 4 or 6 hull pts

            //region DRAW VECTORS
            prevPts = pts.get(0);
            for(int i=1; i<pts.size(); i++)
            {
                Imgproc.line(contourImage, prevPts, pts.get(i), new Scalar((int)(Math.random()*255), (int)(Math.random()*255), (int)(Math.random()*255)), 2);
                prevPts = pts.get(i);
            }
            Imgproc.line(contourImage, pts.get(pts.size()-1), pts.get(0), new Scalar((int)(Math.random()*255), (int)(Math.random()*255), (int)(Math.random()*255)), 2);
            //endregion
        }
        return contourImage;
    }
    public double angleBtwn(Point a, Point b)
    {
        double angle = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
        if (angle < 0) angle += 360;
        return angle;
    }
    public double distance(Point P, Point Q)
    {
        return Math.sqrt(((Q.x-P.x)*(Q.x-P.x)) + ((Q.y-P.y)*(Q.y-P.y)));
    }
    public boolean isInside(List<Point> points)
    {
        int innerPtsCnt = 0;
        for(int i=0; i< points.size(); i++)
        {
            if(points.get(i).x<maxX && points.get(i).x>minX && points.get(i).y<maxY && points.get(i).y>minY)
            {
                innerPtsCnt++;
            }
        }
        if(innerPtsCnt==points.size())
        {
            return true;
        }
        return false;
    }
    public void setExtremes(List<Point> points)
    {
        maxX = 0;
        maxY = 0;
        minX = Integer.MAX_VALUE;
        minY = Integer.MAX_VALUE;
        for(int i=0; i< points.size(); i++)
        {
            Point pts = points.get(i);
            if(pts.x>maxX)
            {
                maxX = pts.x;
            }
            if(pts.x<minX)
            {
                minX = pts.x;
            }
            if(pts.y>maxY)
            {
                maxY = pts.y;
            }
            if(pts.y<minY)
            {
                minY = pts.y;
            }
        }
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
    public double getCenterLineOffset()
    {
        return centerLineOffset;
    }
    public double getArea(){
        return recordArea;
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
