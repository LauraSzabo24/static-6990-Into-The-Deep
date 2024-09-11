package Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.opencv.core.Core;
        import org.opencv.core.Mat;
        import org.opencv.core.Point;
        import org.opencv.core.Rect;
        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvPipeline;

        @Config
public class PropDetectorRED extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        CENTER,
        NOT_FOUND
    }
    private Location location;

    //for reference only (0,0) in top left
    static final Rect SCREENSIZEBOX = new Rect( //make this the correct area
            new Point(0, 0),
            new Point(320, 240));

    //actual boxes

    public static Point leftCorner1 = new Point(100,40);
    public static Point leftCorner2 = new Point(150,80);
    public static Point rightCorner1 = new Point(210,60);
    public static Point rightCorner2 = new Point(260,100);
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    static final Rect LEFT_ROI = new Rect( //make this the correct area
            leftCorner1,
            leftCorner2);
    static final Rect CENTER_ROI = new Rect( //make this the correct area
            rightCorner1, //143 0
            rightCorner2); //282 100
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public PropDetectorRED(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        //test to see where the boxes go, once the boxes are in correct location, then continue
        Scalar testColor = new Scalar(255, 0, 0);
        Imgproc.rectangle(mat, LEFT_ROI, testColor);
        Imgproc.rectangle(mat, CENTER_ROI, testColor);


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //FOR REFERENCE
        Scalar lowHSVYELLOW= new Scalar(10, 0, 0);
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
        Scalar highHSVWHITE = new Scalar(180, 30, 255);
        //REFERENCE ENDS HERE

        /*Scalar lowHSVREDDD = new Scalar(10, 0, 0);
        Scalar highHSVREDDD = new Scalar(175, 255, 255);*/
        Scalar lowHSVRED = new Scalar(10, 0, 0);
        Scalar highHSVRED = new Scalar(150, 255, 255);
        /*Scalar lowHSVRED = new Scalar(0, 0, 0);
        Scalar highHSVRED = new Scalar(150, 255, 255);*/

        Core.inRange(mat, lowHSVRED, highHSVRED, mat);
        Core.bitwise_not(mat, mat); //for red only


        Mat left = mat.submat(LEFT_ROI); //the area on the camera that would be the left prop if it's there
        Mat center = mat.submat(CENTER_ROI); //area on the camera that would be the right prop if it's there

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255; //percentage red
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255; //percentage red

        left.release();
        center.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");

        //is it there or is it not (true = it is there)
        boolean propLeft = (leftValue > PERCENT_COLOR_THRESHOLD);
        boolean propCenter = (centerValue > PERCENT_COLOR_THRESHOLD);

        if (propLeft && propCenter) { //if both are there (assume none are there)
            location = Location.NOT_FOUND;
            telemetry.addData("Prop Location", "both center and left");
        }
        else if(propCenter) { //Center
            location = Location.CENTER; //CENTER
            telemetry.addData("Prop Location", "center");
        }
        else if (propLeft) { //Left
            location = Location.LEFT; //LEFT
            telemetry.addData("Prop Location", "left");
        }
        else {
            location = Location.NOT_FOUND;
            telemetry.addData("Prop Location", "right");
        }
        telemetry.update();

        //for display purposes
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar propBoxColor = new Scalar(255, 0, 0); //for box drawing purposes

        //draws rectangle
        Imgproc.rectangle(mat, CENTER_ROI, propBoxColor);
        Imgproc.rectangle(mat, LEFT_ROI, propBoxColor);
        /*if(location == Location.LEFT)
        {
            Imgproc.rectangle(mat, CENTER_ROI, propBoxColor);
        }
        else if(location == Location.CENTER)
        {
            Imgproc.rectangle(mat, LEFT_ROI, propBoxColor);
        }
        else if (location == Location.NOT_FOUND)
        {

        }
        else{
            Imgproc.rectangle(mat, CENTER_ROI, propBoxColor);
            Imgproc.rectangle(mat, LEFT_ROI, propBoxColor);
        }*/

        return mat;
    }

    public Location getLocation()
    {
        return location;
    }
}