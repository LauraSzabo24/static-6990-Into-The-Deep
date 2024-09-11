/*package Ancient;

public void mainLoop() {
        telemetry.addData("mode ", state);
        //test - field centric
        telemetry.addData("dirTestIMU - " + dirTestIMU + " auto end - ", Mailbox.autoEndHead);

        //button update
        updateDriverAButtons();

        //EMERGENCY MODE CONTROLS
        if (leftBumperReleased && rightBumperReleased && (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down)) {
        leftBumperReleased = false;
        leftBumperPressed = false;
        rightBumperReleased = false;
        rightBumperPressed = false;
            /*b1Released = false;
            b1Pressed = false;
            a1Released = false;
            a1Pressed = false;
            x1Released = false;
            x1Pressed = false;
            y1Released = false;
            y1Pressed = false; // star and / goes hereeeeeeeee
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
        state = Mode.AUTO;
        int[] place1 = firstPos;
        int[] place2 = secPos;
        runPixelPlacing(place1, place2);
        state = Mode.MANUAL;
        firstPos = new int[]{-1,-1};
        secPos = new int[]{-1,-1};
        confirmB = false;
        confirmA = false;
        }

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

        */