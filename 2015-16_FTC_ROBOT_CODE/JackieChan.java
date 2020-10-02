// Import packages
package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class JackieChan extends OpMode {

    //NRG 6762 Res-Q Teleop Code
    //Programmer: Aiden

    // Set Constant Values =======================================================================

    //Last Update: 2/19/2016
    //Used in telemetry to show current code version
    double CNVersion = 160219.05;

    // Initialize DC motors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor armCenter;
    DcMotor tapeRetract;


    // Initialize Servos
    Servo armLeft;
    Servo armRight;
    Servo pinLeft;
    Servo pinRight;
    Servo armLift;
    Servo hookArm;
    Servo tapeAim;
    Servo allClearLeft;
    Servo allClearRight;

    // Set values for pins
    double LEFT_PIN_UP = 0.45;
    double LEFT_PIN_DOWN = 0.0;
    double RIGHT_PIN_UP = 0.47;
    double RIGHT_PIN_DOWN = 0.98;

    // Set values for wings
    double LEFT_WING_UP = 0.43;
    double LEFT_WING_DOWN1 = 0.8;
    double LEFT_WING_DOWN2 = 0.93;
    double RIGHT_WING_UP = 0.6;
    double RIGHT_WING_DOWN1 = 0.23;
    double RIGHT_WING_DOWN2 = 0.10;

    //Set values for AllClear
    double ALLCLEAR_LEFT_UP = 0;
    double ALLCLEAR_LEFT_DOWN = .8;
    double ALLCLEAR_RIGHT_UP = 1;
    double ALLCLEAR_RIGHT_DOWN = .07;

    // Set values for hook
    double HOOK_LOCK = 0.54;
    double HOOK_UP = 0.0;

    // Set starting power for drive DC motors to scale back ...
    double OrigPowerScale = 0.2;
    double PowerScaler = 0.2;
    double PowerScale = OrigPowerScale;

    //Needed to add tis because new motor controller motor is faster (Corrected)
    double LEFT_DRIVE_SCALE = 1.0;

    // Set arm lifting-servo speed
    double TapeLiftSpeed = 0.8;

    // Start protocols
    public void start ()    {

        // Start robot
        super.start ();

        // Reset encoders for DC motors
        reset_dc_encoders();

        //Set Run Modes for DC motors
        set_motor_run_modes();

    }

    // Map Hardware ===============================================================================
    @Override
    public void init() {

        // Retrieve drive DC motors and set directions
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Retrieve arm DC motor
        armCenter = hardwareMap.dcMotor.get("motor_arm");

        // Retrieve tape DC motor
        tapeRetract = hardwareMap.dcMotor.get("motor_tape");
        tapeRetract.setDirection(DcMotor.Direction.REVERSE);


        // Retrieve pin servos
        pinLeft = hardwareMap.servo.get("pin_left");
        pinRight = hardwareMap.servo.get("pin_right");

        // Set pin servo start positions
        pinLeft.setPosition(LEFT_PIN_UP);
        pinRight.setPosition(RIGHT_PIN_UP);

        // Retrieve wing servos
        armLeft = hardwareMap.servo.get("wing_left");
        armRight = hardwareMap.servo.get("wing_right");

        // Set wing servo starting positions
        armLeft.setPosition(LEFT_WING_UP);
        armRight.setPosition(RIGHT_WING_UP);

        // Retrieve lift servo
        armLift = hardwareMap.servo.get("arm_lift");
        armLift.setPosition(0.48);

        // Retrieve hook servo and set start position
        hookArm = hardwareMap.servo.get("arm_hook");
        hookArm.setPosition(HOOK_UP);

        // Retrieve tape aim servo
        tapeAim = hardwareMap.servo.get("tape_aim");
        tapeAim.setPosition(0.5);

        //Retrieve ALLCLEAR Left
        allClearLeft = hardwareMap.servo.get("allclear_left");
        allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);

        //Retrieve ALLCLEAR Left
        allClearRight = hardwareMap.servo.get("allclear_right");
        allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);

    }

    // Control Loop ===============================================================================
    @Override
    public void loop() {

        // Link analog stick positions to float
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        float arm = -gamepad2.right_stick_y;
        float tape = -gamepad2.left_stick_y;

        // Clip analog stick positions to stop from going over 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        arm = Range.clip(arm, -1, 1);
        tape = Range.clip(tape, -1, 1);


        // scale motor power/TURBO MODE!!!
        if (gamepad1.right_stick_button) {
            PowerScale = 1.0;
        }
        else {
            PowerScale =  OrigPowerScale;
        }


        // Scale clipped input and associate power speed with right stick button
        right = (float) scaleInput(right) * (float) PowerScale;
        left = (float) scaleInput(left) * (float) PowerScale * (float) LEFT_DRIVE_SCALE;
        arm = (float) scaleInput(arm);
        tape = (float) scaleInput(tape);


        double AIM_POSITION = (tape * 0.5) + 0.5;

        if (tape > 0){
        tapeAim.setPosition(AIM_POSITION);
        } else if (tape < 0){
            tapeAim.setPosition(AIM_POSITION);
        } else {
            tapeAim.setPosition(0.0);
        }

        // lock and unlock the hook at the end of the arm
        if (gamepad2.right_stick_button) {
            hookArm.setPosition(HOOK_LOCK);
        }
        else {
            hookArm.setPosition(HOOK_UP);
        }

        // Set right wing positions
        //Dual if else structure to originally reset when button released, removed for toggle

        if (gamepad1.x || gamepad1.b) {
            if (gamepad1.x){
                armRight.setPosition(RIGHT_WING_DOWN1);
            }
            else {
                armRight.setPosition(RIGHT_WING_DOWN2);
            }
        }
        if (gamepad1.y)  {
            armRight.setPosition(RIGHT_WING_UP);
        }

        // Set left wing positions
        //Dual if else structure to originally reset when button released, removed for toggle
        if (gamepad1.dpad_right || gamepad1.dpad_left ) {
            if (gamepad1.dpad_right){
                armLeft.setPosition(LEFT_WING_DOWN1);
            }
            else {
                armLeft.setPosition(LEFT_WING_DOWN2);
            }
               }

        if (gamepad1.dpad_up){
            armLeft.setPosition((LEFT_WING_UP));
        }


        double ARM_POSITION;

        // Move the arm lifting servo
        if (gamepad2.dpad_up || gamepad2.dpad_down ) {
            if (gamepad2.dpad_up) {
                armLift.setPosition(0.28);
                ARM_POSITION = 0.28;
            } else {
                armLift.setPosition(0.6);
                ARM_POSITION = 0.6;
            }
        }
        else {
            armLift.setPosition(0.514);
            ARM_POSITION = 0.512;
        }

        // Move the tape aiming servo
        if (gamepad2.y || gamepad2.a) {
            if (gamepad2.a){
                tapeAim.setPosition(.65);
            }
            else {
                tapeAim.setPosition(.45);
            }

        }
        else {
            tapeAim.setPosition(0.5);
        }

        // set right pin position
        if (gamepad1.right_bumper || (gamepad1.right_trigger > 0.25)) {
            if (gamepad1.right_bumper){
                pinRight.setPosition(RIGHT_PIN_UP);
            }
            else if (gamepad1.right_trigger > 0.25) {
                pinRight.setPosition(RIGHT_PIN_DOWN);
            }
         }



        // set left pin position
        if (gamepad1.left_bumper || (gamepad1.left_trigger > 0.25  )) {
           if (gamepad1.left_bumper){
               pinLeft.setPosition(LEFT_PIN_UP);
           }
           else if (gamepad1.left_trigger > 0.25) {
                pinLeft.setPosition(LEFT_PIN_DOWN);
           }
        }



        // set left all clear position
        if (gamepad2.left_bumper || (gamepad2.left_trigger > 0.25  )) {
            if (gamepad2.left_bumper){
                allClearLeft.setPosition(ALLCLEAR_LEFT_UP);
                allClearRight.setPosition(ALLCLEAR_RIGHT_UP);
            }
            else if (gamepad2.left_trigger > 0.25) {
                allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);
                allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);
            }
        }

        //Override Tape Aiming If Feeding Out
        if (tape > 0){
            tapeAim.setPosition(.5+(tape/10));
        }


        // Link dc power to clipped analog stick positions
        motorRight.setPower(right);
        motorLeft.setPower(left);
        armCenter.setPower(arm);
        tapeRetract.setPower(tape);


        // add telemetry data
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right  pwr: " + String.format("%.2f", right));
        telemetry.addData("arm", "arm  pwr: " + String.format("%.2f", arm));
        telemetry.addData("armLift", "armLift  pwr: " + String.format("%.2f", ARM_POSITION));
        telemetry.addData("CNVersion", "Version:" + String.format("%.2f", CNVersion));
    }


    //Sub-Routines ================================================================================
    //
    // input scalar for dc drives
    double scaleInput(double dVal) {

        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18,
                0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        int index = (int) (dVal * 16.0);

        if (index < 0) {
            index = -index;
        }

        if (index > 16) {
            index = 16;
        }

        double dScale;

        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    // reset left encoder
    public void reset_left_drive_encoder () {
        if (motorLeft != null) {
            motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    // reset right encoder
    public void reset_right_drive_encoder () {
        if (motorRight != null) {
            motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    // reset arm encoder
    public void reset_arm_ext_encoder () {
        if (armCenter != null) {
            armCenter.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    // reset tape encoder
    public void reset_tape_ext_encoder () {
        if (tapeRetract != null) {
            tapeRetract.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    // Combine encoder reset methods
    public void reset_dc_encoders () {
        reset_left_drive_encoder();
        reset_right_drive_encoder();
        reset_arm_ext_encoder();
        reset_tape_ext_encoder();
    }

    public void set_motor_run_modes () {
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        armCenter.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        tapeRetract.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

}