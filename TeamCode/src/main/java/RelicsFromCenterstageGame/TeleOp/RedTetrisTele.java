package RelicsFromCenterstageGame.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import RelicsFromCenterstageGame.Auto.Mailbox;

@TeleOp
@Disabled
public class RedTetrisTele extends LinearOpMode {
    //drivetrain
    Pose2d poseEstimate;

    //PID material
    DcMotorEx slideMotorRight;
    DcMotorEx slideMotorLeft;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    //DRIVER A material
    //toggles
    private double speed;
    private double multiply;
    private boolean armInHome;
    private boolean clawInHome;
    private boolean pushPopInHome;
    private boolean intakeLiftInHome;

    //other
    private boolean confirmA;
    private Servo clawServo, armLeftServo, armRightServo, airplaneServo, intakeLiftServo;
    DcMotorEx intakeMotor;

    //mecanum drive stuff
    private SampleMecanumDriveCancelable drive;
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;


    //button controls for DRIVER A
    private boolean a1Pressed;
    private boolean a1Released;
    private boolean b1Pressed;
    private boolean b1Released;
    private boolean x1Pressed;
    private boolean x1Released;
    private boolean y1Pressed;
    private boolean y1Released;
    private boolean leftBumperReleased;
    private boolean leftBumperPressed;
    private boolean rightBumperReleased;
    private boolean rightBumperPressed;


    //DRIVER B material

    private boolean hanging;
    private static String[][] outputArray;
    private int cursorX;
    private int cursorY;
    private static int cursorFlash;
    private int[] firstPos;
    private int[] secPos;
    private boolean confirmB;
    private String[] colors;
    private String previousOutput;

    private double[] position1;
    private double[] position2;

    //button controls for DRIVER B
    private boolean a2Pressed;
    private boolean a2Released;
    private boolean b2Pressed;
    private boolean b2Released;
    private boolean x2Pressed;
    private boolean x2Released;
    private boolean y2Pressed;
    private boolean y2Released;


    //cursor movement DRIVER B
    private boolean leftPressed;
    private boolean leftReleased;
    private boolean rightPressed;
    private boolean rightReleased;
    private boolean downPressed;
    private boolean downReleased;
    private boolean upPressed;
    private boolean upReleased;
    private boolean boxRow;

    //Finite State Machine
    public enum Mode
    {
        MANUAL, AUTO, EMERGENCY;
    }
    public Mode state;

    public void driverAInitialize() {
        //modes
        state = Mode.MANUAL;
        confirmA = false;

        //drivetrain
        speed = 2;
        multiply = 1;

        //emergency mode/ button controls
        a1Pressed = false;
        a1Released = false;
        b1Pressed = false;
        b1Released = false;
        x1Pressed = false;
        x1Released = false;
        y1Pressed = false;
        y1Released = false;
        rightBumperPressed = false;
        rightBumperReleased = false;
        leftBumperPressed = false;
        leftBumperReleased = false;

        //dashboard
        dashboard.setTelemetryTransmissionInterval(25);

        //slide motors
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight = hardwareMap.get(DcMotorEx.class, "RightSlide");
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //drive motors
        /*motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);*/

        //mailbox
        Mailbox mail = new Mailbox();

        //intake outake servos
        armInHome = true;
        pushPopInHome = true;
        clawInHome = true;
        intakeLiftInHome = true;
        clawServo = hardwareMap.get(Servo.class, "claw");
        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
        airplaneServo = hardwareMap.get(Servo.class, "airplaneServo");
        intakeLiftServo = hardwareMap.get(Servo.class, "intakeLiftServo");

        //intake motor
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");

        //arm into start position
        armLeftServo.setPosition(0.99);
        armRightServo.setPosition(0.01);
    }

    public void driverBInitialize() {
        //hanging
        hanging = false;

        //tetris material
        position1 = new double[]{-1,-1};
        position2 = new double[]{-1,-1};
        outputArray = new String[12][14]; //original: 12 13 // new: 12 14
        cursorX = 1;
        cursorY = 10;
        cursorFlash = 50;
        firstPos = new int[]{-1, -1};
        secPos = new int[]{-1, -1};
        confirmB = false;
        previousOutput = "";
        boxRow = true;
        colors = new String[]{"", ""};
        makeGrid();
        printAll();

        //color
        getColors();
        a2Pressed = false;
        a2Released = false;
        b2Pressed = false;
        b2Released = false;

        //other button controls
        x2Pressed = false;
        x2Released = false;
        y2Pressed = false;
        y2Released = false;

        //cursor
        leftPressed = false;
        leftReleased = false;
        rightPressed = false;
        rightReleased = false;
        upPressed = false;
        upReleased = false;
        downPressed = false;
        downReleased = false;
    }

    public void cameraInit() {
        //color camera stuff goes in here
    }

    public void initialize()
    {
        driverAInitialize();
        driverBInitialize();
        cameraInit();
        makeGrid();
    }

    public void runOpMode() throws InterruptedException {
       // initialize();

        //tetris - cancelable trajectories
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        waitForStart();
        if (isStopRequested()) return;

        //get the ending position of auto
        poseEstimate = Mailbox.currentPose;

        // Print pose to telemetry
        telemetry.addLine("pose estimate");
        telemetry.addData("mode", state);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        //while op mode is active get the current pose and run the loop
       while(opModeIsActive() && !isStopRequested())
        {
            poseEstimate = drive.getPoseEstimate();
            drive.update();
            altLoop();
        }
    }

    public void altLoop()
    {
        switch (state) {
            case MANUAL:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                if (gamepad1.a) {
                    Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                            .splineTo(new Vector2d(10,20), 20)
                            .build();

                    drive.followTrajectoryAsync(traj1);

                    state = Mode.AUTO;
                } else if (gamepad1.b) {
                    Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                            .lineTo(new Vector2d(10,20))
                            .build();

                    drive.followTrajectoryAsync(traj1);

                    state = Mode.AUTO;
                }
                break;
            case AUTO:
                if (gamepad1.x) {
                    drive.breakFollowing();
                    state = Mode.MANUAL;
                }
                if (!drive.isBusy()) {
                    state = Mode.MANUAL;
                }
                break;
        }
    }

    public void Loop()
    {
        //slow loop down
        sleep(40);

        //telemetry
        telemetry.addData("mode ", state);
        telemetry.addData(" auto end - ", Mailbox.autoEndHead);
        //telemetry.addData("servo position in ", armLeftServo.getPosition());

        //updates
        updateDriverAButtons();

        //transfer to emergency | both bumpers and any of the directions on the pad
        if (leftBumperReleased && rightBumperReleased && (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down)) {
            leftBumperReleased = false;
            leftBumperPressed = false;
            rightBumperReleased = false;
            rightBumperPressed = false;
            if (state.equals(Mode.EMERGENCY)) {
                state = Mode.MANUAL;
            } else {
                state = Mode.EMERGENCY;
            }
        }

        //change behaviour based on current state
        switch (state) {
            case MANUAL:
                //Color Camera | or in case of not using color camera it just refills the bar with pixels
                if (colors[0].equals("") && colors[1].equals("")) {
                    getColors();
                }

                //Telemetry
                if (armInHome) {
                    telemetry.addLine(String.format("arm in"));
                } else {
                    telemetry.addLine(String.format("arm out"));
                }
                if (clawInHome) {
                    telemetry.addLine(String.format("claws in"));
                } else {
                    telemetry.addLine(String.format("claws out"));
                }
                if (pushPopInHome) {
                    telemetry.addLine(String.format("push pop in"));
                } else {
                    telemetry.addLine(String.format("push pop out"));
                }

                //Driver A | updates controls and checks for confirmation
                updateDriverAControls();
                if (b1Released) {
                    b1Released = false;
                    b1Pressed = false;
                    confirmA = true;
                }

                //Driver B | updates tetris and prints
                printAll();
                updateTetrisThing();

                //Tetris Pixel Placing | checks for confirmation of both drivers | runs first position only
                if (confirmA && confirmB) {
                    confirmA = false;
                    confirmB = false;
                    Trajectory path = drive.trajectoryBuilder(poseEstimate) //position[0]+5
                            .lineToLinearHeading(new Pose2d(10,20))
                            .build();

                    drive.followTrajectoryAsync(path);
                    state = Mode.AUTO;
                    /*int[] place1 = firstPos;
                    int[] place2 = secPos;
                    runPixelPlacing1(place1);
                    confirmB = false;
                    confirmA = false;
                    firstPos = new int[]{-1,-1};
                    secPos = new int[]{-1,-1};*/
                }
                break;

            case AUTO:
                // Telemetry
                telemetry.addLine("RUNNING AUTO MODE");
                telemetry.addLine(String.format("pos1 x coor " + position1[0]));
                telemetry.addLine(String.format("pos1 y coor " + position1[1]));
                telemetry.addLine(String.format("pos2 x coor " + position2[0]));
                telemetry.addLine(String.format("pos2 y coor " + position2[1]));

                //Return to manual once trajectory is done
                if (!drive.isBusy()) {
                    state = Mode.MANUAL;
                }
                //Emergency Exit | driver A press x button
                if (gamepad1.x) {
                    drive.breakFollowing();
                    state = Mode.MANUAL;
                }
                break;

            case EMERGENCY:
                telemetry.addLine(String.format("EMERGENCYYYYYYYY MODEEEEEEEEEE"));
                updateDriverBButtons();
                emergencyModeControls();
                break;
        }
        telemetry.update();
    }

    //Old Loop | without switch case, more unorganized
    public void mainLoop() {
        telemetry.addData("mode ", state);
        //test - field centric
        telemetry.addData(" auto end - ", Mailbox.autoEndHead);

        //button update
        updateDriverAButtons();

        //EMERGENCY MODE CONTROLS
        if (leftBumperReleased && rightBumperReleased && (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down)) {
            leftBumperReleased = false;
            leftBumperPressed = false;
            rightBumperReleased = false;
            rightBumperPressed = false;
            if (state.equals(Mode.EMERGENCY)) {
                state = Mode.MANUAL;
            } else {
                state = Mode.EMERGENCY;
            }
            a2Released = false;
            a2Pressed = false;
        }
        if (state.equals(Mode.EMERGENCY)) {
            telemetry.addLine(String.format("EMERGENCYYYYYYYY MODEEEEEEEEEE"));
            updateDriverBButtons();
            emergencyModeControls();
        }

        //Normal Driver A Controls
        if (state.equals(Mode.MANUAL)) {
            //everything else
            updateDriverAControls();
            //confirmation
            if (b1Released) {
                b1Released = false;
                b1Pressed = false;
                confirmA = true;
            }
        }

        //Tetris Driver B Updating
        if (!state.equals(Mode.EMERGENCY)) {
            printAll();
            updateTetrisThing();
        }

        //Tetris color checker
        if (colors[0].equals("") && colors[1].equals("") && confirmB && confirmA) {
            getColors();
        }

        //Tetris Pixel Placing Thing
       if (confirmA && confirmB && state.equals(Mode.MANUAL)) {
           double targetAHeading = Math.toRadians(90);
           Vector2d targetAVector = new Vector2d((5),15); //y is the y to the board | x is the distance on board + to the board
           Trajectory path = drive.trajectoryBuilder(poseEstimate) //position[0]+5
                   .splineTo(targetAVector, targetAHeading)
                   .build();

           telemetry.addData("ROBOT MOVING AUTO", targetAVector);
           drive.followTrajectoryAsync(path);

            state = Mode.AUTO;
            int[] place1 = firstPos;
            int[] place2 = secPos;
           //moveToPose(position1);
            //runPixelPlacing1(place1);
        }

        /*if((state.equals(Mode.AUTO) || !drive.isBusy())) //bit suspicious
        {
            // If x is pressed, we break out of the automatic following
            if (gamepad1.x) {
                drive.breakFollowing();
                state = Mode.MANUAL;
                firstPos = new int[]{-1,-1};
                secPos = new int[]{-1,-1};
                confirmB = false;
                confirmA = false;
                /*if(secPos[0]!= -1)
                {
                    runPixelPlacing2(secPos);
                }
                else
                {
                    state = Mode.MANUAL;
                    firstPos = new int[]{-1,-1};
                    secPos = new int[]{-1,-1};
                    confirmB = false;
                    confirmA = false;
                }*/
            /*}

            if (!drive.isBusy()) {
                if(secPos[0]!= -1)
                {
                    runPixelPlacing2(secPos);
                }
                else
                {
                    state = Mode.MANUAL;
                    firstPos = new int[]{-1,-1};
                    secPos = new int[]{-1,-1};
                    confirmB = false;
                    confirmA = false;
                }
            }

        }*/

        if(state.equals(Mode.AUTO) || state.equals((Mode.MANUAL))) //take manual option out
        {
            telemetry.addLine(String.format("p1 x coor " + position1[0]));
            telemetry.addLine(String.format("p1 y coor " + position1[1]));
            telemetry.addLine(String.format("p2 x coor " + position2[0]));
            telemetry.addLine(String.format("p2 y coor " + position2[1]));
        }

        //telemetry CAN DELETE LATERRRRR
        if (armInHome) {
            telemetry.addLine(String.format("arm in"));
        } else {
            telemetry.addLine(String.format("arm out"));
        }
        if (clawInHome) {
            telemetry.addLine(String.format("claws in"));
        } else {
            telemetry.addLine(String.format("claws out"));
        }
        if (pushPopInHome) {
            telemetry.addLine(String.format("push pop in"));
        } else {
            telemetry.addLine(String.format("push pop out"));
        }

        //telemetry.addData("servo position in ", armLeftServo.getPosition());
        telemetry.update();
    }

    //DRIVER A NORMAL CONTROLS
    public void updateDriverAControls() {
        //drivetrain changes in speed | right trigger fast | left trigger slow
        if (gamepad1.right_trigger > 0) {
            speed = 4;
        } else {
            speed = 2;
        }
        if (gamepad1.left_trigger > 0) {
            speed = 2;
            multiply = 3;
        } else {
            speed = 2;
            multiply = 1;
        }
        drive();

        //pivots | left and right bumper
        if (gamepad1.right_bumper) {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(1.5);//0.9
            motorFrontRight.setPower(-1.5);
            telemetry.addLine(String.format("pivot right"));

        } else if (gamepad1.left_bumper) {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(-1.5);
            motorFrontRight.setPower(1.5);
            telemetry.addLine(String.format("pivot left"));
        }

        //intake spinner | A button in | X button out
        if (gamepad1.a) {
            intakeMotor.setPower(0.6);
            telemetry.addLine(String.format("powering vacuum cleaner"));
        } else if (gamepad1.x) {
            intakeMotor.setPower(-0.6);
            telemetry.addLine(String.format("powering vacuum cleaner BACK"));
        } else {
            intakeMotor.setPower(0);
            telemetry.addLine(String.format("vacuum cleaner on standby"));
        }

        //More Telemetry
        if (intakeLiftInHome) {
            telemetry.addLine(String.format("intake lifted"));
        } else {
            telemetry.addLine(String.format("intake lowered"));
        }
    }

    public void drive() {
        //Field Centric Driving

        poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -((gamepad1.left_stick_y)* multiply)/(speed+2),
                -((gamepad1.left_stick_x)* multiply)/(speed+2)
        ).rotated(-poseEstimate.getHeading() + Math.toRadians(90));

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        drive.update();
    }

    //EMERGENCY MODE CONTROLS
    public void emergencyModeControls() {
        updateDriverAControls();

        //ALL DRIVER B CONTROLS
        if (gamepad2.dpad_up )//&& slideMotorLeft.getCurrentPosition() < 3500)
        {
            slideMotorLeft.setPower(0.7);
            slideMotorRight.setPower(0.7);
            telemetry.addLine(String.format("slide goes up"));
        }
        if (gamepad2.dpad_down )//&& slideMotorLeft.getCurrentPosition() > 100)
        {
            slideMotorLeft.setPower(-0.4);
            slideMotorRight.setPower(-0.4);
            telemetry.addLine(String.format("slide goes down"));
        }
        if (!gamepad2.dpad_up ) //&& !gamepad2.dpad_down) //&& (Math.abs(targetPosition - slidePos)<15))
        {
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
            telemetry.addLine(String.format("slide on standby"));
        }
        if (gamepad2.dpad_left) {
            telemetry.addLine(String.format("slide goes full down"));
        }

        telemetry.addData("right motor position: ", slideMotorRight.getCurrentPosition());
        telemetry.addData("left motor position: ", slideMotorLeft.getCurrentPosition());


        //hang | Driver B stick buttons
        if (gamepad2.right_stick_button || gamepad2.left_stick_button) {
            hanging = true;
        }
        if (hanging) {
            slideMotorLeft.setPower(-0.4);
            slideMotorRight.setPower(-0.4);
        }

        //flipping thing | Y button
        if (y2Released && armInHome) {
            y2Released = false;
            y2Pressed = false;
            armInHome = false;
            armLeftServo.setPosition(0.95);
            armRightServo.setPosition(0.05);
        } else if (y2Released) {
            y2Released = false;
            y2Pressed = false;
            armInHome = true;
            armLeftServo.setPosition(0);
            armRightServo.setPosition(1);
        }

        //Arm low position | X button
        if (x2Released && pushPopInHome) {
            x2Released = false;
            x2Pressed = false;
            pushPopInHome = false;

            armLeftServo.setPosition(0.7);
            armRightServo.setPosition(0.3);
        } else if (x2Released) {
            x2Released = false;
            x2Pressed = false;
            pushPopInHome = true;

            armLeftServo.setPosition(0.7);
            armRightServo.setPosition(0.3);
        }

        //claw servo | A button
        if (a2Released && clawInHome) {
            a2Released = false;
            a2Pressed = false;
            clawInHome = false;
            clawServo.setPosition(0);
        } else if (a2Released) {
            a2Released = false;
            a2Pressed = false;
            clawInHome = true;
            clawServo.setPosition(0.3);
        }

        //airplane | Bumpers
        if (gamepad2.left_bumper && gamepad2.right_bumper) {
           airplaneServo.setPosition(0.5);
        }

        //Telemetry
        telemetry.addData("servo is at", clawServo.getPosition());
        telemetry.update();
        if (armInHome) {
            telemetry.addLine(String.format("arm in"));
        } else {
            telemetry.addLine(String.format("arm out"));
        }
        if (clawInHome) {
            telemetry.addLine(String.format("claws in"));
        } else {
            telemetry.addLine(String.format("claws out"));
        }
        if (pushPopInHome) {
            telemetry.addLine(String.format("push pop in"));
        } else {
            telemetry.addLine(String.format("push pop out"));
        }
    }

    //BUTTON UPDATES
    public void updateDriverAButtons() {
        //a
        if (gamepad1.a) {
            a1Pressed = true;
        } else if (a1Pressed) {
            a1Released = true;
        }
        //b
        if (gamepad1.b) {
            b1Pressed = true;
        } else if (b1Pressed) {
            b1Released = true;
        }
        //x
        if (gamepad1.x) {
            x1Pressed = true;
        } else if (x1Pressed) {
            x1Released = true;
        }
        //y
        if (gamepad1.y) {
            y1Pressed = true;
        } else if (y1Pressed) {
            y1Released = true;
        }
        //right bumper
        if (gamepad1.right_trigger > 0.8) {
            rightBumperPressed = true;
        } else if (rightBumperPressed) {
            rightBumperReleased = true;
        }
        //left bumper
        if (gamepad1.left_trigger > 0.8) {
            leftBumperPressed = true;
        } else if (leftBumperPressed) {
            leftBumperReleased = true;
        }
    }
    public void updateDriverBButtons() {
        //x
        if (gamepad2.x) {
            x2Pressed = true;
        } else if (x2Pressed) {
            x2Released = true;
        }
        //y
        if (gamepad2.y) {
            y2Pressed = true;
        } else if (y2Pressed) {
            y2Released = true;
        }
        //a
        if (gamepad2.a) {
            a2Pressed = true;
        } else if (a2Pressed) {
            a2Released = true;
        }
    }


    //TEEEEEEEEEETTTTTTTTTTTTTTTTTTTTTTTTTTRRRRRRRRRRRRRIIIIIIIIIIIIIISSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS

    //Convert Coordinate to Position
    public double convertX(int xCoor, int yCoor) //works???
    {
        double inches = 20; //distance to last x coordinate unindented
        if(xCoor%2==1) //indented
        {
            inches -= 1.5; //distance between indented and unindented
            for(int i=13; i>xCoor; i--)
            {
                if(i%2==1)
                {
                    inches -= 3; //one pixel
                }
            }
        }
        else{
            inches -=3; //idk it fixed it
            for(int i=13; i>xCoor; i--)
            {
                if(i%2==0)
                {
                    inches -= 3; //one pixel
                }
            }
        }
        double meepMeepUnit = 1; //inches in meep meepian
        return inches * meepMeepUnit;
    }
    public double convertY(int yCoor) //seems to work
    {
        double inches = 10.5; //height for first unindented pixel
        if(yCoor%2==1) //indented
        {
            inches += 2; //height between indented and unindented
        }
        for(int i=10; i>yCoor; i--)
        {
            if((yCoor%2==1) && (i%2==1)) //indented
            {
                inches += 4; //height between two pixels indented
            }
            else if((yCoor%2==0) && (i%2==0)) //indented
            {
                inches += 4; //height between two pixels unindented
            }
        }
        double slideValue = 1; // 1 inch = ? slide value
        double initialSlideValue = 0; //0 position for slides
        return (inches * slideValue) + initialSlideValue;
    }

    //Places First Pixel
    public void runPixelPlacing1(int[] target1) {
        position1 = new double[2];

        if (target1[0] != -1) {
            position1[0] = convertX(target1[1], target1[0]);
            position1[1] = convertY(target1[0]);

            //Move To Coordinate
            moveToPose(position1);
            //Move Slides
            //Place Pixel
        }
    }

    //Places Second Pixel
    public void runPixelPlacing2(int[] target2) {
        position2 = new double[2];

        if (target2[0] != -1) {
            position2[0] = convertX(target2[1], target2[0]);
            position2[1] = convertY(target2[0]);

            //Move To Coordinate
            moveToPose(position2);
            //Move Slides
            //Place Pixel
        }
    }

    //Moves to Coordinate
    public void moveToPose(double[] position)
    {
        //Test move to position by moving to a random position
        Trajectory path = drive.trajectoryBuilder(poseEstimate) //position[0]+5
                .lineToLinearHeading(new Pose2d(10,20))
                .build();

        drive.followTrajectoryAsync(path);
        state = Mode.AUTO;
    }

    //Refills Pixels in Bar
    public void getColors() {
        colors = new String[]{"@", "@"}; //⬜ 🟪 🟩 🟨
    }

    //PRINTS OUT TETRIS TO TELEMETRY
    public void updateTetrisThing() //WORKS
    {
        //cursor flashing
        if (outputArray[cursorY][cursorX] != "◼") {
            previousOutput = outputArray[cursorY][cursorX];
        }
        cursorFlash--;
        if (cursorFlash > 3) {
            outputArray[cursorY][cursorX] = "◼"; //⬛ █◼
        } else {
            outputArray[cursorY][cursorX] = previousOutput;
        }
        if (cursorFlash < 1) {
            cursorFlash = 50;
            previousOutput = outputArray[cursorY][cursorX];
            outputArray[cursorY][cursorX] = previousOutput;
        }
        cursorUpdate();


        //selection
        if (a2Released && state.equals(Mode.MANUAL) && !(colors[0].equals(""))) {
            a2Pressed = false;
            a2Released = false;
            outputArray[cursorY][cursorX] = colors[0];
            if (colors[1] == "") {
                secPos = new int[]{cursorY, cursorX};
            } else {
                firstPos = new int[]{cursorY, cursorX};
            }
            colors[0] = colors[1];
            colors[1] = "";
        }

        //retrieval???? necessary???

        //confirmation
        if (b2Released && (firstPos[1] != -1 || secPos[1] != -1) && state.equals(Mode.MANUAL)) {
            b2Pressed = false;
            b2Released = false;
            confirmB = true;
        }
    }
    public void cursorUpdate() //WORKS
    {
        //left
        if (gamepad2.dpad_left) {
            leftPressed = true;
        } else if (leftPressed) {
            leftReleased = true;
        }
        //right
        if (gamepad2.dpad_right && cursorX < 13) { //12
            rightPressed = true;
        } else if (rightPressed) {
            rightReleased = true;
        }
        //down
        if (gamepad2.dpad_down && cursorY < 10) {
            downPressed = true;
        } else if (downPressed) {
            downReleased = true;
        }
        //up
        if (gamepad2.dpad_up && cursorY >= 1) {
            upPressed = true;
        } else if (upPressed) {
            upReleased = true;
        }

        //a and b
        if (gamepad2.a) {
            a2Pressed = true;
        } else if (a2Pressed) {
            a2Released = true;
        }
        //up
        if (gamepad2.b) {
            b2Pressed = true;
        } else if (b2Pressed) {
            b2Released = true;
        }

        //cursor movement
        isBoxRow();
        if (leftReleased && cursorX > 1) {
            leftPressed = false;
            leftReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            if (cursorX - 1 > 1) {
                cursorX -= 2;
            }
        } else if (rightReleased && cursorX < 13) { //12
            rightPressed = false;
            rightReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            if (cursorX + 1 < 13) {
                cursorX += 2;
            }
        } else if (downReleased && cursorY < 10) {
            downPressed = false;
            downReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            cursorY++;
            if(cursorX<12) {
                if (boxRow) {
                    cursorX++;
                } else {
                    cursorX--;
                }
            }
            else {
                cursorX--;
            }
        } else if (upReleased && cursorY >= 1) {
            upPressed = false;
            upReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            cursorY--;
            if(cursorX<12) {
                if (boxRow) {
                    cursorX++;
                } else {
                    cursorX--;
                }
            }
            else {
                cursorX--;
            }
        }
    }
    public void isBoxRow() //WORKS
    {
        if (cursorY % 2 == 0) {
            boxRow = true;
        } else {
            boxRow = false;
        }
    }
    public void printAll() //WORKS
    {
        //colors avaliable
        telemetry.addLine(String.format("                    1      2"));
        telemetry.addLine(String.format("PIXELS  - " + colors[0] + "      " + colors[1]));

        //2d array output
        String rowOut = "";
        for (int r = 0; r < outputArray.length; r++) {
            rowOut = "";
            for (int c = 0; c < outputArray[1].length; c++) {
                rowOut += outputArray[r][c];
            }
            telemetry.addData("", rowOut);
        }

        //selections queue
        if (firstPos[0] != -1 && secPos[0] != -1) {
            telemetry.addLine(String.format("QUEUE |   " + firstPos[1] + "," + firstPos[0] + " then " + secPos[1] + "," + secPos[0], null));
        } else if (firstPos[0] != -1) {
            telemetry.addLine(String.format("QUEUE |   " + firstPos[1] + "," + firstPos[0]));
        } else {
            telemetry.addLine(String.format("QUEUE |   "));

        }

        //confirmation queue
        if (confirmB && confirmA) {
            telemetry.addLine(String.format("ROBOT RUNNING"));
        } else if (confirmB) {
            telemetry.addLine(String.format("CONFIRMED PLEASE WAIT"));
        } else if (colors[1].equals("") && firstPos[0] != -1) {
            telemetry.addLine(String.format("UNCONFIRMED CHANGES"));
        } else if (colors[0].equals("") && colors[1].equals("")) {
            telemetry.addLine(String.format("NO PIXELS LOADED"));
        } else {
            telemetry.addLine(String.format("PIXELS READY TO GO"));
        }

    }
    public void makeGrid() //WORKS
    {
        for (int r = 0; r < outputArray.length; r++) {
            for (int c = 0; c < outputArray[1].length; c++) {
                if (c != 0 && c != 14) {
                    if (r % 2 == 0 && c % 2 == 0) {
                        outputArray[r][c] = "   ";
                    } else if (r % 2 == 1 && c % 2 == 1) {
                        outputArray[r][c] = "   ";
                    } else {
                        outputArray[r][c] = "◻"; //O
                    }
                } else {
                    outputArray[r][c] = "   ";
                }
                if ((r == 2 || r == 5 || r == 8) && outputArray[r][c] == "   ") {
                    outputArray[r][c] = "_.";
                }
                if (r == 11) {
                    if (c == 3 || c == 7 || c == 11) {
                        outputArray[r][c] = "X";
                    } else {
                        outputArray[r][c] = "_";
                    }
                }
            }
        }
    }
}