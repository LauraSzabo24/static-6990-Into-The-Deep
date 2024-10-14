package Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class Mailbox
{
    public static double autoEndHead;
    public static ArrayList<Pose2d> listMain;
    public static Pose2d currentPose = new Pose2d();
    public Mailbox()
    {
        listMain = new ArrayList<Pose2d>();
    }
    public void setAutoEnd(Pose2d end){
        currentPose = end;
        listMain.add(end);
    }
    public static ArrayList<Pose2d> getPoses()
    {
        return listMain;
    }
    public void setAutoEndHead(double end){
        autoEndHead = end;
    }
    public double getAutoEndHead(){
        return autoEndHead;
    }
}