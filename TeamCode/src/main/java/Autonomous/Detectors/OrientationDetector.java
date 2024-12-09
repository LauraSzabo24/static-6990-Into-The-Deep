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
        recordArea = 400;
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
        Imgproc.drawContours(contourImage, recordMatList, -1, new Scalar(255, 255, 255), 1);
        //endregion

        if(!contours.isEmpty() && recordMatList.get(0).toList().size()>10)
        {
            //region IMPORTANT POINTS
            Point lowest = new Point();
            double lowRecord = 0;
            Point highest = new Point();
            double highRecord = Integer.MAX_VALUE;

            int startCol = 0;
            Point highStart = new Point();
            Point lowStart = new Point();

            int endCol = 0;
            Point highEnd = new Point();
            Point lowEnd = new Point();
            //endregion

            //GET EVERYTHING ELSE
            List<Point> pts = new ArrayList<>();
            for(int c=0; c<contourImage.cols(); c++)
            {
                for(int r=0; r<contourImage.rows(); r++)
                {
                    if(contourImage.get(r,c)[0]==255 && contourImage.get(r,c)[1]==255 && contourImage.get(r,c)[2]==255)
                    {
                        if(startCol==0)
                        {
                            startCol = c;
                        }

                        if(r>lowRecord)
                        {
                            lowRecord = r;
                            lowest = new Point(c,r);
                            if(Math.abs(startCol-c)<4)
                            {
                                lowStart = lowest;
                            }
                        }
                        if(r<highRecord)
                        {
                            highRecord = r;
                            highest = new Point(c,r);
                            if(Math.abs(startCol-c)<4)
                            {
                                highStart = highest;
                            }
                        }
                    }
                }
            }

            //GET END POINTS
            lowRecord = 0;
            highRecord = Integer.MAX_VALUE;
            boolean found = false;
            IfoundIt:
            for(int c=contourImage.cols()-1; c>0; c--)
            {
                for(int r=contourImage.rows()-1; r>0; r--)
                {
                    if(contourImage.get(r,c)[0]==255 && contourImage.get(r,c)[1]==255 && contourImage.get(r,c)[2]==255)
                    {
                        if(endCol==0)
                        {
                            endCol = c;
                        }

                        if(r>lowRecord)
                        {
                            lowRecord = r;
                            if(Math.abs(endCol-c)<15)
                            {
                                lowEnd = new Point(c,r);
                                found = true;
                            }
                            else if(found){
                                break IfoundIt;
                            }
                        }
                        if(r<highRecord)
                        {
                            highRecord = r;
                            if(Math.abs(endCol-c)<3)
                            {
                                highEnd = new Point(c,r);
                            }
                            else if(found){
                                break IfoundIt;
                            }
                        }
                    }
                }
            }
            pts.add(lowest);
            pts.add(highest);
            pts.add(highStart);
            pts.add(lowStart);
            pts.add(highEnd);
            pts.add(lowEnd);

            //region DRAW VECTORS
            if(!pts.isEmpty()) {
                Imgproc.drawMarker(contourImage, pts.get(0), new Scalar((int) (255), (int) (0), (int) (0)));
                Imgproc.drawMarker(contourImage, pts.get(1), new Scalar((int) (255), (int) (0), (int) (0)));
                Imgproc.drawMarker(contourImage, pts.get(2), new Scalar((int) (0), (int) (255), (int) (0)));
                Imgproc.drawMarker(contourImage, pts.get(3), new Scalar((int) (0), (int) (255), (int) (0)));
                Imgproc.drawMarker(contourImage, pts.get(4), new Scalar((int) (0), (int) (0), (int) (255)));
                Imgproc.drawMarker(contourImage, pts.get(5), new Scalar((int) (255), (int) (0), (int) (255)));
                //Imgproc.line(contourImage, filtered.get(filtered.size() - 1), filtered.get(0), new Scalar((int) (255), (int) (0), (int) (0)), 3);
            }
            //endregion
        }
        return contourImage;
    }
    /*public static List<Point> ramerDouglas(List<Point> points, double epsilon) {
        double maxDistance = 0;
        int index = 0;
        for (int i = 1; i < points.size() - 1; i++) {
            double d = perpendicularDistance(points.get(i), points.get(0), points.get(points.size() - 1));
            if (d > maxDistance) {
                maxDistance = d;
                index = i;
            }
        }
        List<Point> result = new ArrayList<>();
        if (maxDistance > epsilon) {
            List<Point> segment1 = new ArrayList<>(points.subList(0, index + 1));
            List<Point> segment2 = new ArrayList<>(points.subList(index, points.size()));
            List<Point> recursiveResult1 = ramerDouglas(segment1, epsilon);
            List<Point> recursiveResult2 = ramerDouglas(segment2, epsilon);
            result.addAll(recursiveResult1.subList(0, recursiveResult1.size() - 1)); // Avoid duplicating the middle point
            result.addAll(recursiveResult2);
        } else {
            result.add(points.get(0));
            result.add(points.get(points.size() - 1));
        }
        return result;
    }
    private static double perpendicularDistance(Point p, Point start, Point end) {
        double numer = Math.abs((end.y - start.y) * p.x - (end.x - start.x) * p.y + end.x * start.y - end.y * start.x);
        double denom = Math.sqrt(Math.pow(end.y - start.y, 2) + Math.pow(end.x - start.x, 2));
        return numer / denom;
    }*/

    public double angleBtwn(Point a, Point b)
    {
        double angle = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
        if (angle < 0)
        {
            angle += 360;
        }
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
