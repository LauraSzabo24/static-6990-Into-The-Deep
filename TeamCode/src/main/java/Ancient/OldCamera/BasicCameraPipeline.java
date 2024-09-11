package Ancient.OldCamera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BasicCameraPipeline extends OpenCvPipeline {
    Mat video = new Mat();
    public enum Location
    {
        LEFT,
        RIGHT,
        NOT_HERE
    }
    private Location location;
    int result;
    Telemetry telemetry;
    static final Rect LEFT_ROI = new Rect(
            new Point(60,35),
            new Point(120,75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140,35),
            new Point(200,75));
    static double PERCENT_COLOR_THRESH = 0.4;

    @Override
    public void init(Mat firstFrame)
    {
        video = firstFrame.submat(0,50,0,50);
        result = 0;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, video, Imgproc.COLOR_RGB2HSV);
        Scalar lowTotallyNotRGB = new Scalar(13, 31, 77);
        Scalar highTotallyNotRGB = new Scalar(0, 0, 255);

        Core.inRange(video, lowTotallyNotRGB, highTotallyNotRGB, video);

        Mat left = video.submat(LEFT_ROI);
        Mat right = video.submat(RIGHT_ROI);

        double leftVal = Core.sumElems(left).val[0] / LEFT_ROI.area()/255;
        double rightVal = Core.sumElems(right).val[0] / RIGHT_ROI.area()/255;

        left.release();
        right.release();

        telemetry.addData("Left value", (int) Core.sumElems(left).val[0]);
        telemetry.addData( "Right value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftVal*100)+"%");
        telemetry.addData("Left percentage", Math.round(rightVal*100)+"%");

        boolean idkLeft = leftVal > PERCENT_COLOR_THRESH;
        boolean idkRight = rightVal > PERCENT_COLOR_THRESH;

        if(idkLeft && idkRight)
        {
            location = Location.NOT_HERE;
            telemetry.addData("prop location", "not here");
        }
        if(idkLeft){
            location = Location.RIGHT;
            telemetry.addData("prop location", "right");
        }
        else{
            location = Location.LEFT;
            telemetry.addData("prop location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(video, video, Imgproc.COLOR_GRAY2BGR);
        Scalar colorProp = new Scalar(255,0,0);
        Scalar colorPulp = new Scalar(0,255,0);

        Imgproc.rectangle(video, LEFT_ROI, location == Location.LEFT? colorProp: colorPulp);
        Imgproc.rectangle(video, RIGHT_ROI, location == Location.RIGHT? colorProp: colorPulp);

        return video;
    }

    public Location getLocation()
    {
        return location;
    }

    public int getLatestResults()
    {
        return result;
    }

}
