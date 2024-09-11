///*package Ancient;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.util.Angle;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.teamcode.drive.TeleOpAugmentedDriving;
//
//import Auto.Mailbox;
//
//@TeleOp
//@Disabled
//public class redtetrisoldold extends LinearOpMode {
//    //drivetrain
//    IMU imu;
//    Pose2d poseEstimate;
//
//    //PID material
//    DcMotorEx slideMotorRight;
//    DcMotorEx slideMotorLeft;
//    private final FtcDashboard dashboard = FtcDashboard.getInstance();
//
//
//    //DRIVER A material
//    //toggles
//    private double speed;
//    private double multiply;
//    private boolean armInHome;
//    private boolean clawInHome;
//    private boolean pushPopInHome;
//    private boolean intakeLiftInHome;
//
//    //other
//    private boolean confirmA;
//    private Servo clawServo, armLeftServo, armRightServo, airplaneServo, intakeLiftServo;
//    DcMotorEx intakeMotor;
//
//    //mecanum drive stuff
//    private SampleMecanumDriveCancelable drive;
//    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
//
//
//    //button controls for DRIVER A
//    private boolean a1Pressed;
//    private boolean a1Released;
//    private boolean b1Pressed;
//    private boolean b1Released;
//    private boolean x1Pressed;
//    private boolean x1Released;
//    private boolean y1Pressed;
//    private boolean y1Released;
//    private boolean leftBumperReleased;
//    private boolean leftBumperPressed;
//    private boolean rightBumperReleased;
//    private boolean rightBumperPressed;
//
//
//    //DRIVER B material
//
//    private boolean hanging;
//    private static String[][] outputArray;
//    private int cursorX;
//    private int cursorY;
//    private static int cursorFlash;
//    private int[] firstPos;
//    private int[] secPos;
//    private boolean confirmB;
//    private String[] colors;
//    private String previousOutput;
//
//    private double[] position1;
//    private double[] position2;
//
//    //button controls for DRIVER B
//    private boolean a2Pressed;
//    private boolean a2Released;
//    private boolean b2Pressed;
//    private boolean b2Released;
//    private boolean x2Pressed;
//    private boolean x2Released;
//    private boolean y2Pressed;
//    private boolean y2Released;
//
//
//    //cursor movement DRIVER B
//    private boolean leftPressed;
//    private boolean leftReleased;
//    private boolean rightPressed;
//    private boolean rightReleased;
//    private boolean downPressed;
//    private boolean downReleased;
//    private boolean upPressed;
//    private boolean upReleased;
//    private boolean boxRow;
//
//    //mail
//    IMU.Parameters parameters;
//    String dirTestIMU;
//
//    //Finite State Machine
//    public enum Mode {
//        MANUAL, AUTO, EMERGENCY;
//    }
//
//    public TeleOp.RedTetrisTele.Mode state;
//
//    public void driverAInitialize() {
//        //modes
//        state = TeleOp.RedTetrisTele.Mode.MANUAL;
//        confirmA = false;
//
//        //drivetrain
//        speed = 2;
//        multiply = 1;
//
//        //emergency mode/ button controls
//        a1Pressed = false;
//        a1Released = false;
//        b1Pressed = false;
//        b1Released = false;
//        x1Pressed = false;
//        x1Released = false;
//        y1Pressed = false;
//        y1Released = false;
//        rightBumperPressed = false;
//        rightBumperReleased = false;
//        leftBumperPressed = false;
//        leftBumperReleased = false;
//
//        //dashboard
//        dashboard.setTelemetryTransmissionInterval(25);
//
//        //slide motors
//        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "LeftSlide");
//        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        slideMotorRight = hardwareMap.get(DcMotorEx.class, "RightSlide");
//        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //drive motors
//        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
//        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL");
//        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
//        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR");
//
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //imu - field centric
//        Mailbox mail = new Mailbox();
//        imu = hardwareMap.get(IMU.class, "imu");
//        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);
//
//        //intake outake servos
//        armInHome = true;
//        pushPopInHome = true;
//        clawInHome = true;
//        intakeLiftInHome = true;
//        clawServo = hardwareMap.get(Servo.class, "claw");
//        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
//        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
//        airplaneServo = hardwareMap.get(Servo.class, "airplaneServo");
//        intakeLiftServo = hardwareMap.get(Servo.class, "intakeLiftServo");
//
//        //intake motor
//        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
//
//        //arm into start position
//        armLeftServo.setPosition(0.99);
//        armRightServo.setPosition(0.01);
//    }
//
//    public void driverBInitialize() {
//        //hanging
//        hanging = false;
//
//        //tetris material
//        position1 = new double[]{-1, -1};
//        position2 = new double[]{-1, -1};
//        outputArray = new String[12][14]; //original: 12 13 // new: 12 14
//        cursorX = 1;
//        cursorY = 10;
//        cursorFlash = 50;
//        firstPos = new int[]{-1, -1};
//        secPos = new int[]{-1, -1};
//        confirmB = false;
//        previousOutput = "";
//        boxRow = true;
//        colors = new String[]{"", ""};
//        makeGrid();
//        printAll();
//
//        //color
//        getColors();
//        a2Pressed = false;
//        a2Released = false;
//        b2Pressed = false;
//        b2Released = false;
//
//        //other button controls
//        x2Pressed = false;
//        x2Released = false;
//        y2Pressed = false;
//        y2Released = false;
//
//        //cursor
//        leftPressed = false;
//        leftReleased = false;
//        rightPressed = false;
//        rightReleased = false;
//        upPressed = false;
//        upReleased = false;
//        downPressed = false;
//        downReleased = false;
//    }
//
//    public void cameraInit() {
//        //color camera stuff goes in here
//    }
//
//    public void runOpMode() throws InterruptedException {
//        initialize();
//
//        //tetris - cancelable trajectories
//        drive = new SampleMecanumDriveCancelable(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setPoseEstimate(Mailbox.currentPose);
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//            //tetris
//            poseEstimate = drive.getPoseEstimate();
//            // Print pose to telemetry
//            telemetry.addData("mode", state);
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//
//
//            mainLoop();
//        }
//    }
//
//    public void initialize() {
//        driverAInitialize();
//        driverBInitialize();
//        cameraInit();
//        makeGrid();
//    }
//
//    public void altLoop() {
//        sleep(40);
//
//        //telemetry
//        telemetry.addData("mode ", state);
//        telemetry.addData("dirTestIMU - " + dirTestIMU + " auto end - ", Mailbox.autoEndHead);
//        //telemetry.addData("servo position in ", armLeftServo.getPosition());
//
//        //updates
//        updateDriverAButtons();
//
//        //transfer to emergency
//        if (leftBumperReleased && rightBumperReleased && (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down)) {
//            leftBumperReleased = false;
//            leftBumperPressed = false;
//            rightBumperReleased = false;
//            rightBumperPressed = false;
//            if (state.equals(TeleOp.RedTetrisTele.Mode.EMERGENCY)) {
//                state = TeleOp.RedTetrisTele.Mode.MANUAL;
//            } else {
//                state = TeleOp.RedTetrisTele.Mode.EMERGENCY;
//            }
//        }
//
//        switch (state) {
//            case MANUAL:
//                //Color Camera
//                if (colors[0].equals("") && colors[1].equals("") && confirmB && confirmA) {
//                    getColors();
//                }
//                //Telemetry
//                if (armInHome) {
//                    telemetry.addLine(String.format("arm in"));
//                } else {
//                    telemetry.addLine(String.format("arm out"));
//                }
//                if (clawInHome) {
//                    telemetry.addLine(String.format("claws in"));
//                } else {
//                    telemetry.addLine(String.format("claws out"));
//                }
//                if (pushPopInHome) {
//                    telemetry.addLine(String.format("push pop in"));
//                } else {
//                    telemetry.addLine(String.format("push pop out"));
//                }
//                //Driver A
//                updateDriverAControls();
//                if (b1Released) {
//                    b1Released = false;
//                    b1Pressed = false;
//                    confirmA = true;
//                }
//                //Driver B
//                updateTetrisThing();
//                //Tetris Pixel Placing
//                if (confirmA && confirmB) {
//                    state = TeleOp.RedTetrisTele.Mode.AUTO;
//                    int[] place1 = firstPos;
//                    int[] place2 = secPos;
//                    runPixelPlacing1(place1);
//                    state = TeleOp.RedTetrisTele.Mode.MANUAL;
//                    firstPos = new int[]{-1, -1};
//                    secPos = new int[]{-1, -1};
//                    confirmB = false;
//                    confirmA = false;
//                }
//            case AUTO:
//                // Telemetry
//                telemetry.addLine(String.format("p1 x coor " + position1[0]));
//                telemetry.addLine(String.format("p1 y coor " + position1[1]));
//                telemetry.addLine(String.format("p2 x coor " + position2[0]));
//                telemetry.addLine(String.format("p2 y coor " + position2[1]));
//            case EMERGENCY:
//                telemetry.addLine(String.format("EMERGENCYYYYYYYY MODEEEEEEEEEE"));
//                updateDriverBButtons();
//                emergencyModeControls();
//            default:
//                //nothing
//        }
//        telemetry.update();
//    }
//
//    public void mainLoop() {
//        telemetry.addData("mode ", state);
//        //test - field centric
//        telemetry.addData("dirTestIMU - " + dirTestIMU + " auto end - ", Mailbox.autoEndHead);
//
//        //button update
//        updateDriverAButtons();
//
//        //EMERGENCY MODE CONTROLS
//        if (leftBumperReleased && rightBumperReleased && (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down)) {
//            leftBumperReleased = false;
//            leftBumperPressed = false;
//            rightBumperReleased = false;
//            rightBumperPressed = false;
//            if (state.equals(TeleOp.RedTetrisTele.Mode.EMERGENCY)) {
//                state = TeleOp.RedTetrisTele.Mode.MANUAL;
//            } else {
//                state = TeleOp.RedTetrisTele.Mode.EMERGENCY;
//            }
//            a2Released = false;
//            a2Pressed = false;
//        }
//        if (state.equals(TeleOp.RedTetrisTele.Mode.EMERGENCY)) {
//            telemetry.addLine(String.format("EMERGENCYYYYYYYY MODEEEEEEEEEE"));
//            updateDriverBButtons();
//            emergencyModeControls();
//        }
//
//        //Normal Driver A Controls
//        if (state.equals(TeleOp.RedTetrisTele.Mode.MANUAL)) {
//            //everything else
//            updateDriverAControls();
//            //confirmation
//            if (b1Released) {
//                b1Released = false;
//                b1Pressed = false;
//                confirmA = true;
//            }
//        }
//
//        //Tetris Driver B Updating
//        if (!state.equals(TeleOp.RedTetrisTele.Mode.EMERGENCY)) {
//            printAll();
//            updateTetrisThing();
//        }
//
//        //Tetris color checker
//        if (colors[0].equals("") && colors[1].equals("") && confirmB && confirmA) {
//            getColors();
//        }
//
//        //Tetris Pixel Placing Thing
//        if (confirmA && confirmB && state.equals(TeleOp.RedTetrisTele.Mode.MANUAL)) {
//            state = TeleOp.RedTetrisTele.Mode.AUTO;
//            int[] place1 = firstPos;
//            int[] place2 = secPos;
//            runPixelPlacing1(place1);
//        }
//
//        /*if((state.equals(Mode.AUTO) || !drive.isBusy())) //bit suspicious
//        {
//            // If x is pressed, we break out of the automatic following
//            if (gamepad1.x) {
//                drive.breakFollowing();
//                state = Mode.MANUAL;
//                firstPos = new int[]{-1,-1};
//                secPos = new int[]{-1,-1};
//                confirmB = false;
//                confirmA = false;
//                /*if(secPos[0]!= -1)
//                {
//                    runPixelPlacing2(secPos);
//                }
//                else
//                {
//                    state = Mode.MANUAL;
//                    firstPos = new int[]{-1,-1};
//                    secPos = new int[]{-1,-1};
//                    confirmB = false;
//                    confirmA = false;
//                }*/
//            /*}
//
//            if (!drive.isBusy()) {
//                if(secPos[0]!= -1)
//                {
//                    runPixelPlacing2(secPos);
//                }
//                else
//                {
//                    state = Mode.MANUAL;
//                    firstPos = new int[]{-1,-1};
//                    secPos = new int[]{-1,-1};
//                    confirmB = false;
//                    confirmA = false;
//                }
//            }
//
//        }*/
//
//        if (state.equals(TeleOp.RedTetrisTele.Mode.AUTO) || state.equals((TeleOp.RedTetrisTele.Mode.MANUAL))) //take manual option out
//        {
//            telemetry.addLine(String.format("p1 x coor " + position1[0]));
//            telemetry.addLine(String.format("p1 y coor " + position1[1]));
//            telemetry.addLine(String.format("p2 x coor " + position2[0]));
//            telemetry.addLine(String.format("p2 y coor " + position2[1]));
//        }
//
//        //telemetry CAN DELETE LATERRRRR
//        if (armInHome) {
//            telemetry.addLine(String.format("arm in"));
//        } else {
//            telemetry.addLine(String.format("arm out"));
//        }
//        if (clawInHome) {
//            telemetry.addLine(String.format("claws in"));
//        } else {
//            telemetry.addLine(String.format("claws out"));
//        }
//        if (pushPopInHome) {
//            telemetry.addLine(String.format("push pop in"));
//        } else {
//            telemetry.addLine(String.format("push pop out"));
//        }
//
//        //telemetry.addData("servo position in ", armLeftServo.getPosition());
//        telemetry.update();
//    }
//
//    //DRIVER A NORMAL CONTROLS FROM HEREEEE
//    public void updateDriverAControls() {
//        //mecanum
//        if (gamepad1.right_trigger > 0) {
//            speed = 4;
//        } else {
//            speed = 2;
//        }
//        if (gamepad1.left_trigger > 0) {
//            speed = 2;
//            multiply = 3;
//        } else {
//            speed = 2;
//            multiply = 1;
//        }
//        drive();
//
//        //pivots
//        if (gamepad1.right_bumper) {
//            motorBackLeft.setPower(0);
//            motorBackRight.setPower(0);
//            motorFrontLeft.setPower(1.5);//0.9
//            motorFrontRight.setPower(-1.5);
//            telemetry.addLine(String.format("pivot right"));
//
//        } else if (gamepad1.left_bumper) {
//            motorBackLeft.setPower(0);
//            motorBackRight.setPower(0);
//            motorFrontLeft.setPower(-1.5);
//            motorFrontRight.setPower(1.5);
//            telemetry.addLine(String.format("pivot left"));
//        }
//
//        //intake spinner sucking vacuum cleaner thing
//        if (gamepad1.a) {
//            // intakeMotor.setPower(0.6);
//            telemetry.addLine(String.format("powering vacuum"));
//        } else if (gamepad1.x) {
//            //intakeMotor.setPower(-0.6);
//            telemetry.addLine(String.format("powering vacuum BACK"));
//        } else {
//            //intakeMotor.setPower(0);
//            telemetry.addLine(String.format("vacuum on standby"));
//        }
//
//        //telemetry CAN DELETE LATERRRR
//        if (intakeLiftInHome) {
//            telemetry.addLine(String.format("intake lifted"));
//        } else {
//            telemetry.addLine(String.format("intake lowered"));
//        }
//    }
//
//    public void drive() {
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        // This button choice was made so that it is hard to hit on accident,
//        // it can be freely changed based on preference.
//        // The equivalent button is start on Xbox-style controllers.
//        if (gamepad1.options) {
//            imu.resetYaw();
//        }
//
//        //strange math that somehow works
//        double autoEnd = Mailbox.autoEndHead;
//        if (Mailbox.autoEndHead > 300) {
//            autoEnd -= 360;
//        }
//        double botHeading = Math.toRadians(autoEnd) - ((2 * Math.PI) - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.toRadians(90));
//
//        telemetry.addData("imu value", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
//        telemetry.addData("bot value", Math.toDegrees(botHeading));
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//        motorFrontLeft.setPower(multiply * frontLeftPower / speed);
//        motorBackLeft.setPower(multiply * backLeftPower / speed);
//        motorFrontRight.setPower(multiply * frontRightPower / speed);
//        motorBackRight.setPower(multiply * backRightPower / speed);
//        telemetry.addLine(String.format("setting motor powers to:"));
//        telemetry.addData("frontLeft ", frontLeftPower);
//        telemetry.addData("backLeft ", backLeftPower);
//        telemetry.addData("frontRight ", frontRightPower);
//        telemetry.addData("backRight ", backRightPower);
//    }
//
//
//    //EMERGENCY MODE THINGS HEREEE
//    public void emergencyModeControls() {
//        updateDriverAControls();
//
//        //ALL DRIVER B CONTROLS
//        if (gamepad2.dpad_up)//&& slideMotorLeft.getCurrentPosition() < 3500)
//        {
//            slideMotorLeft.setPower(0.7);
//            slideMotorRight.setPower(0.7);
//            telemetry.addLine(String.format("slide goes up"));
//        }
//        if (gamepad2.dpad_down)//&& slideMotorLeft.getCurrentPosition() > 100)
//        {
//            slideMotorLeft.setPower(-0.4);
//            slideMotorRight.setPower(-0.4);
//            telemetry.addLine(String.format("slide goes down"));
//        }
//        if (!gamepad2.dpad_up) //&& !gamepad2.dpad_down) //&& (Math.abs(targetPosition - slidePos)<15))
//        {
//            slideMotorLeft.setPower(0);
//            slideMotorRight.setPower(0);
//            telemetry.addLine(String.format("slide on standby"));
//        }
//        if (gamepad2.dpad_left) {
//            telemetry.addLine(String.format("slide goes full down"));
//        }
//
//        telemetry.addData("right motor position: ", slideMotorRight.getCurrentPosition());
//        telemetry.addData("left motor position: ", slideMotorLeft.getCurrentPosition());
//
//
//        //hang
//        if (gamepad2.right_stick_button || gamepad2.left_stick_button) {
//            hanging = true;
//        }
//        if (hanging) {
//            slideMotorLeft.setPower(-0.4);
//            slideMotorRight.setPower(-0.4);
//        }
//
//        //flipping thing
//        if (y2Released && armInHome) {
//            y2Released = false;
//            y2Pressed = false;
//            armInHome = false;
//            armLeftServo.setPosition(0.95);
//            armRightServo.setPosition(0.05);
//        } else if (y2Released) {
//            y2Released = false;
//            y2Pressed = false;
//            armInHome = true;
//            armLeftServo.setPosition(0);
//            armRightServo.setPosition(1);
//        }
//
//        //Arm low position
//        if (x2Released && pushPopInHome) {
//            x2Released = false;
//            x2Pressed = false;
//            pushPopInHome = false;
//
//            armLeftServo.setPosition(0.7);
//            armRightServo.setPosition(0.3);
//        } else if (x2Released) {
//            x2Released = false;
//            x2Pressed = false;
//            pushPopInHome = true;
//
//            armLeftServo.setPosition(0.7);
//            armRightServo.setPosition(0.3);
//        }
//
//        //claw servo
//        if (a2Released && clawInHome) {
//            a2Released = false;
//            a2Pressed = false;
//            clawInHome = false;
//            clawServo.setPosition(0);
//        } else if (a2Released) {
//            a2Released = false;
//            a2Pressed = false;
//            clawInHome = true;
//            clawServo.setPosition(0.3);
//        }
//
//        //airplane
//        if (gamepad2.left_bumper && gamepad2.right_bumper) {
//            airplaneServo.setPosition(0.5);
//        }
//
//        //telemetry CAN DELETE LATERRRRR
//        telemetry.addData("servo is at", clawServo.getPosition());
//        telemetry.update();
//        if (armInHome) {
//            telemetry.addLine(String.format("arm in"));
//        } else {
//            telemetry.addLine(String.format("arm out"));
//        }
//        if (clawInHome) {
//            telemetry.addLine(String.format("claws in"));
//        } else {
//            telemetry.addLine(String.format("claws out"));
//        }
//        if (pushPopInHome) {
//            telemetry.addLine(String.format("push pop in"));
//        } else {
//            telemetry.addLine(String.format("push pop out"));
//        }
//    }
//
//    //BUTTON UPDATES
//    public void updateDriverAButtons() {
//        //a
//        if (gamepad1.a) {
//            a1Pressed = true;
//        } else if (a1Pressed) {
//            a1Released = true;
//        }
//        //b
//        if (gamepad1.b) {
//            b1Pressed = true;
//        } else if (b1Pressed) {
//            b1Released = true;
//        }
//        //x
//        if (gamepad1.x) {
//            x1Pressed = true;
//        } else if (x1Pressed) {
//            x1Released = true;
//        }
//        //y
//        if (gamepad1.y) {
//            y1Pressed = true;
//        } else if (y1Pressed) {
//            y1Released = true;
//        }
//        //right bumper
//        if (gamepad1.right_trigger > 0.8) {
//            rightBumperPressed = true;
//        } else if (rightBumperPressed) {
//            rightBumperReleased = true;
//        }
//        //left bumper
//        if (gamepad1.left_trigger > 0.8) {
//            leftBumperPressed = true;
//        } else if (leftBumperPressed) {
//            leftBumperReleased = true;
//        }
//    }
//
//    public void updateDriverBButtons() {
//        //x
//        if (gamepad2.x) {
//            x2Pressed = true;
//        } else if (x2Pressed) {
//            x2Released = true;
//        }
//        //y
//        if (gamepad2.y) {
//            y2Pressed = true;
//        } else if (y2Pressed) {
//            y2Released = true;
//        }
//        //a
//        if (gamepad2.a) {
//            a2Pressed = true;
//        } else if (a2Pressed) {
//            a2Released = true;
//        }
//    }
//
//
//    //TEEEEEEEEEETTTTTTTTTTTTTTTTTTTTTTTTTTRRRRRRRRRRRRRIIIIIIIIIIIIIISSSSSSSSSSSSSSSSSSSSSSSSSSSS
//
//    public double convertX(int xCoor, int yCoor) //doesn't work
//    {
//        double inches = 20; //distance to last x coordinate unindented
//        if (xCoor % 2 == 1) //indented
//        {
//            inches -= 1.5; //distance between indented and unindented
//            for (int i = 13; i > xCoor; i--) {
//                if (i % 2 == 1) {
//                    inches -= 3; //one pixel
//                }
//            }
//        } else {
//            inches -= 3; //idk it fixed it
//            for (int i = 13; i > xCoor; i--) {
//                if (i % 2 == 0) {
//                    inches -= 3; //one pixel
//                }
//            }
//        }
//        double meepMeepUnit = 1; //inches in meep meepian
//        return inches * meepMeepUnit;
//    }
//
//    public double convertY(int yCoor) //seems to work
//    {
//        double inches = 10.5; //height for first unindented pixel
//        if (yCoor % 2 == 1) //indented
//        {
//            inches += 2; //height between indented and unindented
//        }
//        for (int i = 10; i > yCoor; i--) {
//            if ((yCoor % 2 == 1) && (i % 2 == 1)) //indented
//            {
//                inches += 4; //height between two pixels indented
//            } else if ((yCoor % 2 == 0) && (i % 2 == 0)) //indented
//            {
//                inches += 4; //height between two pixels unindented
//            }
//        }
//        double slideValue = 1; // 1 inch = ? slide value
//        double initialSlideValue = 0; //0 position for slides
//        return (inches * slideValue) + initialSlideValue;
//    }
//
//    public void runPixelPlacing1(int[] target1) {
//        position1 = new double[2];
//
//        if (target1[0] != -1) {
//            position1[0] = convertX(target1[1], target1[0]);
//            position1[1] = convertY(target1[0]);
//
//            moveToPose(position1);
//            //moves slides while going
//            //push pixel out
//        }
//
//    }
//
//    public void runPixelPlacing2(int[] target1) {
//        position2 = new double[2];
//
//        if (target1[0] != -1) {
//            position2[0] = convertX(target1[1], target1[0]);
//            position2[1] = convertY(target1[0]);
//
//            moveToPose(position2);
//            //moves slides while going
//            //push pixel out
//        }
//
//    }
//
//    public void moveToPose(double[] position) {
//        double targetAHeading = Math.toRadians(90);
//        Vector2d targetAVector = new Vector2d((5), 15); //y is the y to the board | x is the distance on board + to the board
//        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate) //position[0]+5
//                .splineTo(targetAVector, targetAHeading)
//                .build();
//
//        telemetry.addData("ROBOT ON THE MODE", targetAVector);
//        drive.followTrajectoryAsync(traj1);
//    }
//
//    public void getColors() {
//        colors = new String[]{"@", "@"}; //⬜ 🟪 🟩 🟨
//    }
//
//    public void updateTetrisThing() {
//        //cursor flashing
//        if (outputArray[cursorY][cursorX] != "◼") {
//            previousOutput = outputArray[cursorY][cursorX];
//        }
//        cursorFlash--;
//        if (cursorFlash > 3) {
//            outputArray[cursorY][cursorX] = "◼"; //⬛ █◼
//        } else {
//            outputArray[cursorY][cursorX] = previousOutput;
//        }
//        if (cursorFlash < 1) {
//            cursorFlash = 50;
//            previousOutput = outputArray[cursorY][cursorX];
//            outputArray[cursorY][cursorX] = previousOutput;
//        }
//        cursorUpdate();
//
//
//        //selection
//        if (a2Released && state.equals(TeleOp.RedTetrisTele.Mode.MANUAL) && !(colors[0].equals(""))) {
//            a2Pressed = false;
//            a2Released = false;
//            outputArray[cursorY][cursorX] = colors[0];
//            if (colors[1] == "") {
//                secPos = new int[]{cursorY, cursorX};
//            } else {
//                firstPos = new int[]{cursorY, cursorX};
//            }
//            colors[0] = colors[1];
//            colors[1] = "";
//        }
//
//        //retrieval???? necessary???
//
//        //confirmation
//        if (b2Released && (firstPos[1] != -1 || secPos[1] != -1) && state.equals(TeleOp.RedTetrisTele.Mode.MANUAL)) {
//            b2Pressed = false;
//            b2Released = false;
//            confirmB = true;
//        }
//    }
//
//    public void cursorUpdate() //WORKS
//    {
//        //left
//        if (gamepad2.dpad_left) {
//            leftPressed = true;
//        } else if (leftPressed) {
//            leftReleased = true;
//        }
//        //right
//        if (gamepad2.dpad_right && cursorX < 13) { //12
//            rightPressed = true;
//        } else if (rightPressed) {
//            rightReleased = true;
//        }
//        //down
//        if (gamepad2.dpad_down && cursorY < 10) {
//            downPressed = true;
//        } else if (downPressed) {
//            downReleased = true;
//        }
//        //up
//        if (gamepad2.dpad_up && cursorY >= 1) {
//            upPressed = true;
//        } else if (upPressed) {
//            upReleased = true;
//        }
//
//        //a and b
//        if (gamepad2.a) {
//            a2Pressed = true;
//        } else if (a2Pressed) {
//            a2Released = true;
//        }
//        //up
//        if (gamepad2.b) {
//            b2Pressed = true;
//        } else if (b2Pressed) {
//            b2Released = true;
//        }
//
//        //cursor movement
//        isBoxRow();
//        if (leftReleased && cursorX > 1) {
//            leftPressed = false;
//            leftReleased = false;
//            outputArray[cursorY][cursorX] = previousOutput;
//            if (cursorX - 1 > 1) {
//                cursorX -= 2;
//            }
//        } else if (rightReleased && cursorX < 13) { //12
//            rightPressed = false;
//            rightReleased = false;
//            outputArray[cursorY][cursorX] = previousOutput;
//            if (cursorX + 1 < 13) {
//                cursorX += 2;
//            }
//        } else if (downReleased && cursorY < 10) {
//            downPressed = false;
//            downReleased = false;
//            outputArray[cursorY][cursorX] = previousOutput;
//            cursorY++;
//            if (cursorX < 12) {
//                if (boxRow) {
//                    cursorX++;
//                } else {
//                    cursorX--;
//                }
//            } else {
//                cursorX--;
//            }
//        } else if (upReleased && cursorY >= 1) {
//            upPressed = false;
//            upReleased = false;
//            outputArray[cursorY][cursorX] = previousOutput;
//            cursorY--;
//            if (cursorX < 12) {
//                if (boxRow) {
//                    cursorX++;
//                } else {
//                    cursorX--;
//                }
//            } else {
//                cursorX--;
//            }
//        }
//    }
//
//    public void isBoxRow() //WORKS
//    {
//        if (cursorY % 2 == 0) {
//            boxRow = true;
//        } else {
//            boxRow = false;
//        }
//    }
//
//    public void printAll() {
//        //colors avaliable
//        telemetry.addLine(String.format("                    1      2"));
//        telemetry.addLine(String.format("PIXELS  - " + colors[0] + "      " + colors[1]));
//
//        //2d array output
//        String rowOut = "";
//        for (int r = 0; r < outputArray.length; r++) {
//            rowOut = "";
//            for (int c = 0; c < outputArray[1].length; c++) {
//                rowOut += outputArray[r][c];
//            }
//            telemetry.addData("", rowOut);
//        }
//
//        //selections queue
//        if (firstPos[0] != -1 && secPos[0] != -1) {
//            telemetry.addLine(String.format("QUEUE |   " + firstPos[1] + "," + firstPos[0] + " then " + secPos[1] + "," + secPos[0], null));
//        } else if (firstPos[0] != -1) {
//            telemetry.addLine(String.format("QUEUE |   " + firstPos[1] + "," + firstPos[0]));
//        } else {
//            telemetry.addLine(String.format("QUEUE |   "));
//
//        }
//
//        //confirmation queue
//        if (confirmB && confirmA) {
//            telemetry.addLine(String.format("ROBOT RUNNING"));
//        } else if (confirmB) {
//            telemetry.addLine(String.format("CONFIRMED PLEASE WAIT"));
//        } else if (colors[1].equals("") && firstPos[0] != -1) {
//            telemetry.addLine(String.format("UNCONFIRMED CHANGES"));
//        } else if (colors[0].equals("") && colors[1].equals("")) {
//            telemetry.addLine(String.format("NO PIXELS LOADED"));
//        } else {
//            telemetry.addLine(String.format("PIXELS READY TO GO"));
//        }
//
//    }
//
//    public void makeGrid() { //WORKS
//        for (int r = 0; r < outputArray.length; r++) {
//            for (int c = 0; c < outputArray[1].length; c++) {
//                if (c != 0 && c != 14) {
//                    if (r % 2 == 0 && c % 2 == 0) {
//                        outputArray[r][c] = "   ";
//                    } else if (r % 2 == 1 && c % 2 == 1) {
//                        outputArray[r][c] = "   ";
//                    } else {
//                        outputArray[r][c] = "◻"; //O
//                    }
//                } else {
//                    outputArray[r][c] = "   ";
//                }
//                if ((r == 2 || r == 5 || r == 8) && outputArray[r][c] == "   ") {
//                    outputArray[r][c] = "_.";
//                }
//                if (r == 11) {
//                    if (c == 3 || c == 7 || c == 11) {
//                        outputArray[r][c] = "X";
//                    } else {
//                        outputArray[r][c] = "_";
//                    }
//                }
//            }
//        }
//    }
//}
//*/