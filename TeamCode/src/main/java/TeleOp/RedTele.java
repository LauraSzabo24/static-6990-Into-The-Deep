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

    public static double flpPP = 10, flpPI = 0, flpPD = 0;
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

    int spinnerPos = 0;
    //endregion

    //region GAMEPADS
    Gamepad currG1;
    Gamepad oldG1;
    Gamepad currG2;
    Gamepad oldG2;

    //endregion

    //region CONTROL STATE
    private enum poseControlState
    {
        FREE,
        PICKUP,
        LOW,
        HIGH,
        HOME
    }
    private enum speedControlState
    {
        NORMAL,
        PRECISION,
        SUPERSPEED
    }
    speedControlState gameModeA;
    speedControlState gameModeB;
    poseControlState currentState;

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
        wristServo.setPosition(0.39);
        clawServo.setPosition(0.4);
        spinnerServo.setPosition(0.26);
        flpPosTarget = -200;
        extTarget = 0;
    }

    //BASICS
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();

        movementInit();
        clawIH = true;
        currentState = poseControlState.FREE;
        gameModeA = speedControlState.NORMAL;
        gameModeB = speedControlState.NORMAL;

        waitForStart();

        double cumutime = 0;
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainLoop();
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
        driverAControls();
        driverBControls();
        telemetry.addData("CURRENT POSITION STATE", currentState);
        stateCheck();

        //EXTENDER & FLIPPER
        extCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }

    //DRIVING CONTROLS
    public void driverAControls()
    {
        //SPEED CHANGES | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
        if (gamepad1.right_trigger > 0) {
            gameModeA = speedControlState.PRECISION;
        }
        if (gamepad1.left_trigger > 0) {
            gameModeA = speedControlState.SUPERSPEED;
        }
        switch(gameModeA){
            case NORMAL:
                multiply = 1;
                speed = 3;
                break;
            case PRECISION:
                multiply = 0.5;
                speed = 3;
                break;
            case SUPERSPEED:
                multiply =0.7;
                speed = 1;
                break;
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
        //region PRECISION AND SUPERSPEED MODE
        gameModeB = speedControlState.NORMAL;
        if (gamepad2.right_trigger > 0) {
            gameModeB = speedControlState.PRECISION;
        }
        if (gamepad2.left_trigger > 0) {
            gameModeB = speedControlState.SUPERSPEED;
        }
        double flpAddition = 0;
        double extAddition = 0;
        switch(gameModeA){
            case NORMAL:
                flpAddition = 70;
                extAddition = 40;
                break;
            case PRECISION:
                flpAddition = 10;
                extAddition = 10;
                break;
            case SUPERSPEED:
                flpAddition = 80;
                extAddition = 60;
                break;
        }
        //endregion

        setStates();
        //region CLAW
        if (currG2.b && !oldG2.b)
        {
            if(clawIH)
            {
                clawServo.setPosition(0.4);
            }
            else {
                clawServo.setPosition(0);
            }
            clawIH = !clawIH;
        }
        //endregion

        //region WRIST
        if(gamepad2.left_stick_x>0 && wristServo.getPosition()<(1-0.005))
        {
            telemetry.addLine("WRIST MOVEMENT");
            wristServo.setPosition(wristServo.getPosition() + 0.005);
            currentState = poseControlState.FREE;
        }
        else if(gamepad2.left_stick_x<0 && wristServo.getPosition()>=0.39)
        {
            telemetry.addLine("WRIST MOVEMENT");
            wristServo.setPosition(wristServo.getPosition() - 0.005);
            currentState = poseControlState.FREE;
        }
        if(gamepad2.left_stick_button)
        {
            telemetry.addLine("WRIST MOVEMENT");
            wristServo.setPosition(0.39);
            currentState = poseControlState.FREE;
        }
        //endregion

        //region SPINNER
        if(gamepad2.right_stick_x>0 || gamepad2.right_stick_x<0)
        {
            telemetry.addLine("SPINNER MOVEMENT");
            spinnerServo.setPosition(spinnerServo.getPosition() + (gamepad2.right_stick_x * 0.05));
            currentState = poseControlState.FREE;
        }
        if(currG2.right_stick_button && !oldG2.right_stick_button)
        {
            spinnerPos ++;
            if(spinnerPos>3)
            {
                spinnerPos = 0;
            }
            switch(spinnerPos)
            {
                case 0:
                    spinnerServo.setPosition(0);
                    break;
                case 1:
                    spinnerServo.setPosition(0.26);
                    break;
                case 2:
                    spinnerServo.setPosition(0.54);
                    break;
                case 3:
                    spinnerServo.setPosition(0.81);
                    break;
            }
        }
        //endregion

        //region EXTENDER
        if(gamepad2.dpad_up && extTarget<=1800) //1600
        {
            telemetry.addLine("ext UP");
            currentState = poseControlState.FREE;
            if(extTarget+extAddition>=1800-extAddition)
            {
                extTarget+=Math.abs(Math.abs(extTarget)-1800);
            }
            else {
                extTarget+=extAddition;
            }
        }
        else if(gamepad2.dpad_down && extTarget>=0)
        {
            telemetry.addLine("ext DOWN");
            currentState = poseControlState.FREE;
            if(extTarget-extAddition<=0)
            {
                extTarget-=Math.abs(extTarget);
            }
            else {
                extTarget-=extAddition;
            }
        }
        //endregion

        //region FLIPPER
        if(gamepad2.dpad_right && flpPosTarget<=-150)
        {
            telemetry.addLine("flp DOWN");
            currentState = poseControlState.FREE;

            if(flpPosTarget+flpAddition>=-150)
            {
                flpPosTarget+=Math.abs(flpPosTarget+150);
            }
            else {
                flpPosTarget+=flpAddition;
            }
        }
        else if(gamepad2.dpad_left && flpPosTarget>=-4300)
        {
            telemetry.addLine("flp UP");
            currentState = poseControlState.FREE;

            if(flpPosTarget-flpAddition<=-4300)
            {
                flpPosTarget-=Math.abs(flpPosTarget+4300);
            }
            else {
                flpPosTarget-=flpAddition;
            }
        }
        //endregion
    }
    public void setStates()
    {
        //region HIGH POSITION
        if(currentState!= poseControlState.HIGH && currG2.y && !oldG2.y)
        {
            telemetry.addLine("TO HIGH POSITION");
            currentState = poseControlState.HIGH;
            if(flpPosTarget>-2000) {
                extTarget = 1600;
                spinnerServo.setPosition(0.77);
                wristServo.setPosition(0.8389);
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    flpPosTarget = -1650;
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            else {
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    extTarget = 0;
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                armMotor.setPower(0);

                spinnerServo.setPosition(0.77);
                wristServo.setPosition(0.8389);
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    flpPosTarget = -1650;
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    extTarget = 1600;
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());

                }
            }
        }
        //endregion

        //region LOW POSITION
        if(currentState!= poseControlState.LOW && currG2.left_bumper && !oldG2.left_bumper)
        {
            if(currentState == poseControlState.HIGH)
            {
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    flpPosTarget = -1800;
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }

                extTarget = 440;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }

                spinnerServo.setPosition(0.77);
                wristServo.setPosition(0.7989);
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    flpPosTarget = -1160;
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            else if(flpPosTarget>-2000) {
                extTarget = 440;
                spinnerServo.setPosition(0.77);
                wristServo.setPosition(0.7989);
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    flpPosTarget = -1160;
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            else {
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    extTarget = 0;
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                armMotor.setPower(0);

                spinnerServo.setPosition(0.77);
                wristServo.setPosition(0.7989);
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    flpPosTarget = -1160;
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    extTarget = 440;
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());

                }
            }
            telemetry.addLine("TO LOW POSITION");
            currentState = poseControlState.LOW;
        }
        //endregion

        //region HOME POSITION
        if(currentState!= poseControlState.HOME && currG2.x && !oldG2.x)
        {
            telemetry.addLine("TO HOME POSITION");
            currentState = poseControlState.HOME;
            jerkTimer.reset();
            while(armMotor.getCurrentPosition()>60) {
                extTarget = 0;
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            armMotor.setPower(0);

            wristServo.setPosition(0.39);
            spinnerServo.setPosition(0.26);
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                flpPosTarget = -200;
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            armMotor.setPower(0);
        }
        //endregion

        //region PICKUP POSITION
        if(currentState!= poseControlState.PICKUP && currG2.a && !oldG2.a) {
            telemetry.addLine("TO PICKUP POSITION");
            currentState = poseControlState.PICKUP;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                extTarget = 0;
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            armMotor.setPower(0);
            spinnerServo.setPosition(0.215);
            wristServo.setPosition(0.595);
            jerkTimer.reset();
            while (jerkTimer.time() < 0.5) {
                flpPosTarget = -3671;
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
        //endregion

        //region JERK
        if( flipMotor.getCurrentPosition()>-2000 && currG2.right_bumper && !oldG2.right_bumper) {
            spinnerServo.setPosition(0.77);
            wristServo.setPosition(0.95);
            jerkTimer.reset();
            while (jerkTimer.time() < 0.6) {
                extTarget = 1140;
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            clawServo.setPosition(0);
        }
        //endregion
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

        double velocityVal = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        flipMotor.setVelocity(velocityVal);

        /*double veloTarget = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        telemetry.addData("targetvelo",veloTarget);
        double neededPower = flpVelocityCONTROLLER(veloTarget, flipMotor.getVelocity(), time);
        telemetry.addData("power",neededPower);
        flipMotor.setPower(neededPower);*/
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
        telemetry.addData("flpPosition ", flipMotor.getCurrentPosition());
        telemetry.addData("flpTarget", flpPosTarget);

        telemetry.addData("ext TARGET - ", extTarget);
        telemetry.addData("spinner - ", spinnerServo.getPosition());
        telemetry.addData("wrist - ", wristServo.getPosition());
        telemetry.addData("claw - ", clawServo.getPosition());
    }
}
