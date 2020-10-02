package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by user on 2/19/2016.
 */
public class BabyJerbees extends LinearOpMode {


    //Variables
    int TicksPerInch = 120;
    //Last Update: 2/5/2016
    //Used in telemetry to show current code version
    double CNVersion = 160219.01;

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

    // Set values for pins
    double LEFT_PIN_UP = 0.45;
    double LEFT_PIN_DOWN = 0.0;
    double RIGHT_PIN_UP = 0.47;
    double RIGHT_PIN_DOWN = 0.98;

    // Set values for wings
    double LEFT_WING_UP = 0.43;
    double LEFT_WING_DOWN1 = 0.8;
    double LEFT_WING_DOWN2 = 0.88;
    double RIGHT_WING_UP = 0.6;
    double RIGHT_WING_DOWN1 = 0.23;
    double RIGHT_WING_DOWN2 = 0.10;

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
    double ArmLiftSpeed = 0.0;


    // Call motors and autonomous actions
    @Override
    public void runOpMode() throws InterruptedException {

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

        waitForStart();

        //Wait 9 Seconds
        sleep(9000);

        // Move across field 73 inches
        Move(-100, 0.20, 10000, "Drive Across Field");

        //Idle
        driveMotorFloat();

    }


    // Move forward method
    // NOTE: there are about 89 pulses per inch and 1069 pulses per foot
    // Power: Is literal power that goes to both motors.
    // timeout is in milliseconds

    public void Move(int inches, double power, long timeout, String description) throws InterruptedException {

        int pulses = inches * TicksPerInch;
        long expiredtime = System.currentTimeMillis() + timeout;

        driveMotorsEncoderReset();

        motorLeft.setPower(power);
        motorRight.setPower(power);
        motorLeft.setTargetPosition(pulses);
        motorRight.setTargetPosition(pulses);

        driveMotorsRunToMode();


        while ((((motorLeft.isBusy())|| (motorRight.isBusy()))) && (System.currentTimeMillis()<= expiredtime))
        {
            waitOneFullHardwareCycle();
            telemetry.addData("1:", "Command: Move " + String.format("%d", pulses)+ " " + description);
            telemetry.addData("2:", "left targ/pos: " + String.format("%d", pulses)+ " " + String.format("%d", motorLeft.getCurrentPosition()));
            telemetry.addData("3:", "right targ/pos: " + String.format("%d", pulses)+ " " + String.format("%d", motorRight.getCurrentPosition()));
            telemetry.addData("4:", "time: " +String.format("%d", expiredtime) + " " + String.format("%d", System.currentTimeMillis()));
        }

    }

    // Stop motor method
    public void driveMotorFloat()  throws InterruptedException {
        telemetry.addData("CurrCommand", "Command: Stop/Float ");
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);
        if (motorLeft != null) {
            motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
        if (motorRight != null) {
            motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
        waitOneFullHardwareCycle();
    }

    public void driveMotorsEncoderReset() throws InterruptedException {

        // Reset motor controllers
        if (motorLeft != null) {motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);}
        if (motorRight != null) {motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);}

        while (motorLeft.getCurrentPosition()!=0 && motorRight.getCurrentPosition()!=0) {
            waitOneFullHardwareCycle();
        }
    }

    public void driveMotorsRunToMode() throws InterruptedException {

        // Run motors with encoders
        if (motorLeft != null) {motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);}
        if (motorRight != null) {motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);}

        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();

    }
}
