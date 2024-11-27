package TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Log;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

import java.io.File;
import java.io.IOException;

import Autonomous.Mailbox;

@TeleOp
@Config
public class RedTele extends LinearOpMode {
    //region FLIPPER CONTROLLER
    //POSITION
    ElapsedTime timer = new ElapsedTime();
    private double flpPosError = 0;
    private double flpPosISum = 0;

    public static double flpPP = 0.1, flpPI = 0, flpPD = 0;
    public static int flpPosTarget = 0;

    //VELOCITY
    private double flpVeloError = 0;
    private double flpVeloISum = 0;
    public static int flpVeloTarget = 0;
    public static int testTarget = 1200;
    public static double flpVP = 0.0002, flpVI = 0.004, flpVD = 0.0000001;  //RISING
    //endregion

    //region EXTENDER CONTROLLER
    public static double ticksPerDegree = 537.7;
    private PIDController ext;
    public static double extP = 0.004, extI = 0.00, extD = 0.00015, extF = 0.003;
    public static int extTarget;
    FtcDashboard dashboard;
    //endregion

    //region DRIVER A MATERIAL
    IMU imu;
    IMU.Parameters parameters;
    NewMecanumDrive drive;
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
    //endregion

    //region DRIVER B MATERIAL
    private Servo wristServo, spinnerServo, clawServo;
    DcMotorEx flipMotor, armMotor;
    boolean clawIH;
    boolean pickupTwo = false;
    ElapsedTime jerkTimer = new ElapsedTime();
    boolean jerked = false;
    //endregion

    //region GAMEPADS
    Gamepad currG1;
    Gamepad oldG1;
    Gamepad currG2;
    Gamepad oldG2;

    //endregion

    //region CONTROL STATE
    private enum controlStateB
    {
        FREE,
        PICKUP,
        LOW,
        HIGH
    }
    private enum controlStateA
    {
        UNLIMITED,
        LIMITED,
        HANG
    }
    controlStateB gameModeB;
    controlStateA gameModeA;
    //endregion

    //INITIALIZATIONS
    public void hardwareInit()
    {
        //RANDOM
        dashboard = FtcDashboard.getInstance();

        //PID
        ext = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motors
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        wristServo = hardwareMap.get(Servo.class, "wrist");
        spinnerServo = hardwareMap.get(Servo.class, "spinner");
        clawServo = hardwareMap.get(Servo.class, "claw");

        //gamepads
        currG1 = new Gamepad();
        oldG1 = new Gamepad();
        currG2 = new Gamepad();
        oldG2 = new Gamepad();


        //mailbox
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void movementInit()
    {

        wristServo.setPosition(0.2); //0.8
        clawServo.setPosition(1);

        //spinnerServo.setPosition(0.1522);
        //flpTarget = -200;
        //extTarget = 0;
    }

    //BASICS
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();

        //movementInit();
       // clawIH = true;
        gameModeB = controlStateB.FREE;
        gameModeA = controlStateA.UNLIMITED;

        waitForStart();

        double cumutime = 0;
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainLoop();
            /*double time = timer.seconds();
            timer.reset();
            cumutime+=time;

            int targetPositionYay = 0;
            if(gamepad1.a)
            {
                targetPositionYay = -testTarget;
            }
            if(gamepad1.b)
            {
                targetPositionYay = testTarget;
            }
            //flpCONTROLLER(targetPositionYay, flipMotor.getCurrentPosition());
            double needPower = flpVelocityCONTROLLER(targetPositionYay,flipMotor.getVelocity(), time);
            flipMotor.setPower(needPower);
            //telemetry.addData("targets", targetPositionYay);
            telemetry.addData("power",needPower);
            telemetry.addData("time",cumutime);
            telemetry.update();*/
        }
    }
    public void mainLoop()
    {
        //UPDATES
        oldG1.copy(currG1);
        oldG2.copy(currG2);
        currG1.copy(gamepad1);
        currG2.copy(gamepad2);
        telemetry.update();
        drive.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());

        //CONTROLS
        switch(gameModeA){
            case UNLIMITED:
                driverAControls();
                break;
            case LIMITED:
                break;
        }
        driverBControls();

        telemetry.addData("GAMEMODE B", gameModeB);
        //telemetry.addData("GAMEMODE A", gameModeA);
        stateCheck();

        //EXTENDER & FLIPPER
        extCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }

    //DRIVING CONTROLS
    public void driverAControls()
    {
        //SPEED CHANGES | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
        multiply = 1;
        speed = 3;
        if (gamepad1.right_trigger > 0) {
            multiply = 0.5;
            speed = 3;
        }
        if (gamepad1.left_trigger > 0) {
            multiply =0.7;
            speed = 1;
        }

        //FIELD CENTRIC
        poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -((gamepad1.left_stick_y)* multiply)/speed,
                -((gamepad1.left_stick_x)* multiply)/speed
        ).rotated(-poseEstimate.getHeading());
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        ((gamepad1.right_stick_x * multiply)/speed)
                )
        );
        drive.update();
    }
    public void driverBControls()
    {
        setStates();
        //region CLAW
        if (currG2.b && !oldG2.b)
        {
            if(clawIH)
            {
                clawServo.setPosition(0.6);
            }
            else {
                clawServo.setPosition(0.2);
            }
            clawIH = !clawIH;
        }
        //endregion

        //region WRIST
        if(gamepad2.left_stick_x>0 || gamepad2.left_stick_x<0)
        {
            telemetry.addLine("WRIST MOVEMENT");
            wristServo.setPosition(wristServo.getPosition() - (currG2.left_stick_x * 0.05));
            gameModeB = controlStateB.FREE;
        }
        if(gamepad2.left_stick_button)
        {
            telemetry.addLine("WRIST MOVEMENT");
            wristServo.setPosition(0);//0.8
            gameModeB = controlStateB.FREE;
        }
        //endregion

        //region SPINNER
        if(gamepad2.right_stick_x>0 || gamepad2.right_stick_x<0)
        {
            telemetry.addLine("SPINNER MOVEMENT");
            spinnerServo.setPosition(spinnerServo.getPosition() + (gamepad2.right_stick_x * 0.05));
            gameModeB = controlStateB.FREE;
        }
        if(gamepad2.right_stick_button)
        {
            spinnerServo.setPosition(0.1522);
        }
        //endregion

        //region EXTENDER
        if(gamepad2.dpad_up && extTarget<=1500)
        {
            telemetry.addLine("ext UP");
            gameModeB = controlStateB.FREE;
            if(extTarget+50>=1340)
            {
                extTarget+=Math.abs(Math.abs(extTarget)-1500);
            }
            else {
                extTarget+=50;
            }
        }
        else if(gamepad2.dpad_down && extTarget>=0)
        {
            telemetry.addLine("ext DOWN");
            gameModeB = controlStateB.FREE;
            if(extTarget-50<=0)
            {
                extTarget-=Math.abs(extTarget);
            }
            else {
                extTarget-=50;
            }
        }
        //endregion

        //region FLIPPER
        if(gamepad2.dpad_left && flpPosTarget<=-150)
        {
            telemetry.addLine("flp DOWN");
            gameModeB = controlStateB.FREE;

            if(flpPosTarget+40>=-150)
            {
                flpPosTarget+=Math.abs(flpPosTarget+150);
            }
            else {
                flpPosTarget+=40;
            }
        }
        else if(gamepad2.dpad_right && flpPosTarget>=-1650)
        {
            telemetry.addLine("flp UP");
            gameModeB = controlStateB.FREE;

            if(flpPosTarget-40<=-1650)
            {
                flpPosTarget-=Math.abs(flpPosTarget+1650);
            }
            else {
                flpPosTarget-=40;
            }
        }
        //endregion
    }
    public void setStates()
    {/*
        //region HIGH POSITION
        if(gameModeB!= controlStateB.HIGH && currG2.y && !oldG2.y)
        {
            divider = 4;
            telemetry.addLine("TO HIGH POSITION");
            if(flpTarget>-700) {
                jerkTimer.reset();
                while(jerkTimer.time() < 0.2) {
                    flpTarget = -740;
                    flpCONTROLLER();
                }
            }
            else {
                gameModeB = controlStateB.HIGH;
                jerkTimer.reset();
                while (jerkTimer.time() < 0.5) {
                    extTarget = 0;
                    flpTarget = -740;
                    flpCONTROLLER();
                    extCONTROLLER();
                }

                jerkTimer.reset();
                while (jerkTimer.time() < 0.5) {
                    extTarget = 1500;
                    spinnerServo.setPosition(0.6989);
                    wristServo.setPosition(0.174); //0.6183
                    extCONTROLLER();
                }
            }
        }
        //endregion

        //region LOW POSITION
        else if(gameModeB!= controlStateB.LOW && currG2.x && !oldG2.x)
        {
            divider = 4;
            telemetry.addLine("TO LOW POSITION");
            gameModeB = controlStateB.LOW;

            jerkTimer.reset();
            while (jerkTimer.time() < 0.5) {
                extTarget = 0;
                flpTarget = -250;
                wristServo.setPosition(0); //0.8
                spinnerServo.setPosition(0.1522);
                flpCONTROLLER();
                extCONTROLLER();
            }
        }
        //endregion

        //region PICKUP POSITION
        else if(gameModeB!= controlStateB.PICKUP && currG2.a && !oldG2.a)
        {
            divider = 4;
            jerkTimer.reset();
            while(jerkTimer.time() < 1.0) {
                extTarget = 0;
                extCONTROLLER();
            }
            clawServo.setPosition(0.6);

            if(!pickupTwo) {
                telemetry.addLine("PICKUP LOW");
                wristServo.setPosition(0.2);//0.8
                pickupTwo = true;
            }
            else if( gameModeB != controlStateB.PICKUP) {
                telemetry.addLine("PICKUP HANG");
                wristServo.setPosition(0.0556); //0.822
                pickupTwo = false;
            }
            gameModeB = controlStateB.PICKUP;
            spinnerServo.setPosition(0.1522);
            flpTarget = -1500;
            clawServo.setPosition(0.6);
        }
        //endregion

        //region JERK
        if(currG2.right_bumper && !oldG2.right_bumper)
        {
            divider = 4;
            jerkTimer.reset();
            wristServo.setPosition(0.8);//0.2
            flpTarget = -600;

            jerkTimer.reset();
            while(jerkTimer.time() < 1.0)
            {
                flpCONTROLLER();
            }
            wristServo.setPosition(0.555); //0.2444
            clawServo.setPosition(0.6);
        }
        divider = 1;
        //endregion
        */
    }

    //CONTROLLERS
    public void extCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = armMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget) + (Math.cos(Math.toRadians(extTarget/ticksPerDegree)) * extF);
        armMotor.setPower(extPwr *(1/3.0));

        telemetry.addData("extPos ", extPose);
        telemetry.addData("extTarget ", extTarget);
    }

    public void flpCONTROLLER(int target, int state) //in with the target -> out with the velocity
    {
        int currError = target - state;
        double time = timer.seconds();
        timer.reset();
        flpPosISum += currError * time;
        double deriv = (currError - flpPosError)/time;
        flpPosError = currError;

        double veloTarget = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        telemetry.addData("targetvelo",veloTarget);
        double neededPower = flpVelocityCONTROLLER(veloTarget, flipMotor.getVelocity(), time);
        telemetry.addData("power",neededPower);
        flipMotor.setPower(neededPower);
    }

    public double flpVelocityCONTROLLER(double target, double state, double time) //in with the velocity -> out with the power
    {
        /*if(target<=-300)
        {
            target = -300;
        }*/
        double currError = (target - state);
        flpVeloISum += currError * time;
        double deriv = (currError - flpVeloError)/time;
        flpVeloError = currError;

        telemetry.addData("velocity",flipMotor.getVelocity());
        telemetry.addData("position",flipMotor.getCurrentPosition());

        if((flipMotor.getCurrentPosition()>-900 && target>0) || (flipMotor.getCurrentPosition()<-900 && target<0)) //IS FALLING
        {
            telemetry.addLine("FALLING");
            telemetry.addData("ERROR", currError);
            return (flpVP * currError) + (flpVI *flpVeloISum) + (flpVD * deriv);
        }
        else //IS RISING
        {
            telemetry.addLine("RISING");
            return (flpVP * currError) + (flpVI *flpVeloISum) + (flpVD * deriv);
        }
    }

    //(flpVG*Math.sin(Math.toRadians((flipMotor.getCurrentPosition()/-1800.0)*180))

    //TELEMETRY
    public void stateCheck()
    {
        if(clawIH)
        {
            telemetry.addLine("CLAW CLOSE");
        }
        else{
            telemetry.addLine("CLAW OPEN");
        }

        telemetry.addData("flpVelocity ", flipMotor.getVelocity());
        telemetry.addData("flpTargetVelocity ", flpVeloTarget);
        telemetry.addData("flpPosition ", flipMotor.getCurrentPosition());
        telemetry.addData("flpTargetPosition ", flpPosTarget);

        telemetry.addData("ext TARGET - ", extTarget);
        telemetry.addData("spinner - ", spinnerServo.getPosition());
        telemetry.addData("wrist - ", wristServo.getPosition());
        telemetry.addData("claw - ", clawServo.getPosition());
    }
}
