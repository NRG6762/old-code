package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import android.app.Activity;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by user on 4/16/2016.
 */
public class Dance extends LinearOpMode {

    //DDDDAAAANNNNCCCCEEEE!!!!

    //Variables
    //Drive wheel encoder pulses per inch
    int TicksPerInch = 120;


    //Last Update: 2/19/2016
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
    Servo allClearLeft;
    Servo allClearRight;

    // Set values for pins
    double LEFT_PIN_UP = 0.45;
    double LEFT_PIN_DOWN = 0.0;
    double RIGHT_PIN_UP = 0.47;
    double RIGHT_PIN_DOWN = 0.98;

    // Set values for wings
    double LEFT_WING_UP = 0.46;
    double LEFT_WING_DOWN1 = 0.8;
    double LEFT_WING_DOWN2 = 0.88;
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
    double ArmLiftSpeed = 0.0;

    public MediaPlayer mediaplayer;


    public void runOpMode() throws InterruptedException{
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


        waitForStart();
        //mediaplayer = MediaPlayer.create(,R.raw.song);

        Step1();
        Step2();
        Step3();
        Step4();
        Step5();
        Step3();
        Step7();
        Step8();
        Step3();
        Step7();
        Step3();
        Step8();
    }
    public void Move(int inches, double power, long timeout, String description) throws InterruptedException {

        //Calculate # encoder pulses based on inches (pos or neg values)
        int pulses = inches * TicksPerInch;

        //Calc expiration for timeout
        long expiredtime = System.currentTimeMillis() + timeout;

        //Reset encoders
        driveMotorsEncoderReset();

        //Set power and distance
        motorLeft.setPower(power);
        motorRight.setPower(power);
        motorLeft.setTargetPosition(pulses);
        motorRight.setTargetPosition(pulses);

        //Execute
        driveMotorsRunToMode();

        //Wait while getting there or hit timeout
        while ((((motorLeft.isBusy())|| (motorRight.isBusy()))) && (System.currentTimeMillis()<= expiredtime))
        {
            waitOneFullHardwareCycle();
            telemetry.addData("1:", "Command: Move " + String.format("%d", pulses)+ " " + description);
            telemetry.addData("2:", "left targ/pos: " + String.format("%d", pulses)+ " " + String.format("%d", motorLeft.getCurrentPosition()));
            telemetry.addData("3:", "right targ/pos: " + String.format("%d", pulses)+ " " + String.format("%d", motorRight.getCurrentPosition()));
            telemetry.addData("4:", "time: " +String.format("%d", expiredtime) + " " + String.format("%d", System.currentTimeMillis()));
        }

    }

    public void Turn(int degrees, double power, long timeout, String description)  throws InterruptedException  {
        //Need to define left and right distances
        int leftpulses;
        int rightpulses;

        //Reset Encoders
        driveMotorsEncoderReset();

        //44 inches of rotation per tire = 360 degrees

        //Just move one tire while locking other ... less slippage
        leftpulses = (int)((double) 44/360 * (double) degrees * (double) TicksPerInch) ;
        rightpulses = leftpulses * -1;

        //Calc max time
        long expiredtime = System.currentTimeMillis() + timeout;

        //Set power and distance
        motorLeft.setPower(power);
        motorRight.setPower(power);
        motorLeft.setTargetPosition(leftpulses);
        motorRight.setTargetPosition(rightpulses);

        driveMotorsRunToMode();

        telemetry.addData("1:", "Command: Turn " + String.format("%d", degrees) + " " + description);
        telemetry.addData("2:", "left targ/pos: " + String.format("%d", leftpulses) + " " + String.format("%d", motorLeft.getCurrentPosition()));
        telemetry.addData("3:", "right targ/pos: " + String.format("%d", rightpulses) + " " + String.format("%d", motorRight.getCurrentPosition()));
        telemetry.addData("4:", "time: " + String.format("%d", expiredtime) + " " + String.format("%d", System.currentTimeMillis()));

        //Wait while getting there or hit timeout
        while ((((motorLeft.isBusy())|| (motorRight.isBusy()))) && (System.currentTimeMillis()<= expiredtime))
        {
            waitOneFullHardwareCycle();
            telemetry.addData("1:", "Command: Turn " + String.format("%d", degrees)+ " " + description);
            telemetry.addData("2:", "left targ/pos: " + String.format("%d", leftpulses) + " " + String.format("%d", motorLeft.getCurrentPosition()));
            telemetry.addData("3:", "right targ/pos: " + String.format("%d", rightpulses)+ " " + String.format("%d", motorRight.getCurrentPosition()));
            telemetry.addData("4:", "time: " +String.format("%d", expiredtime) + " " + String.format("%d", System.currentTimeMillis()));
        }

    }

    // Stop motor method
    public void driveMotorFloat()  throws InterruptedException {
        telemetry.addData("CurrCommand", "Command: Stop/Float " );
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);
        if (motorLeft != null) {motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);}
        if (motorRight != null) {motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);}
        waitOneFullHardwareCycle();
    }

    // Spin motor method
    // We use this to keep tires spinning while trying to climb mountain
    public void driveMotorSpin(double power)  throws InterruptedException {
        telemetry.addData("CurrCommand", "Command: Spin Wheels Forward" );

        driveMotorsEncoderReset();

        motorLeft.setPower(power);
        motorRight.setPower(power);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        waitOneFullHardwareCycle();

        if (motorLeft != null) {motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);}
        if (motorRight != null) {motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);}

        waitOneFullHardwareCycle();
    }


    public void driveMotorsEncoderReset() throws InterruptedException {

        // Reset motor controllers
        if (motorLeft != null) {motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);}
        if (motorRight != null) {motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);}

        //Wait until actually reset
        while (motorLeft.getCurrentPosition()!=0 && motorRight.getCurrentPosition()!=0) {
            waitOneFullHardwareCycle();
        }
    }

    public void driveMotorsRunToMode() throws InterruptedException {

        // Run motors with encoders
        if (motorLeft != null) {motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);}
        if (motorRight != null) {motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);}
        waitOneFullHardwareCycle();

    }

    public void NervousTick(int TickNum) throws InterruptedException{
        for (int repeatnum=0; repeatnum<=TickNum; repeatnum++) {
            pinLeft.setPosition(LEFT_PIN_UP);
            pinRight.setPosition(RIGHT_PIN_DOWN);
            sleep(500);
            pinLeft.setPosition(LEFT_PIN_DOWN);
            pinRight.setPosition(RIGHT_PIN_UP);
            sleep(500);
        }
    }

    public void ArmThrow(int ThrowNum) throws InterruptedException{
        for (int repeatnum=0; repeatnum<=ThrowNum; repeatnum++) {
            armLeft.setPosition(LEFT_WING_UP);
            armRight.setPosition(RIGHT_WING_DOWN2);
            sleep(500);
            armLeft.setPosition(LEFT_WING_DOWN2);
            armRight.setPosition(RIGHT_WING_UP);
            sleep(500);
        }
    }

    public void Cheer(int Lemon) throws InterruptedException{
        for (int repeatnum=0; repeatnum<=Lemon; repeatnum++) {
            allClearLeft.setPosition(ALLCLEAR_LEFT_UP);
            allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);
            sleep(500);
            allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);
            allClearRight.setPosition(ALLCLEAR_RIGHT_UP);
            sleep(500);
        }
    }

    public void Step1() throws InterruptedException{
        NervousTick(5);
        pinLeft.setPosition(LEFT_PIN_UP);
        pinRight.setPosition(RIGHT_PIN_UP);
    }

    public void Step2() throws InterruptedException{
        NervousTick(5);
        pinLeft.setPosition(LEFT_PIN_UP);
        pinRight.setPosition(RIGHT_PIN_UP);
        ArmThrow(5);
        armRight.setPosition(RIGHT_WING_UP);
        armLeft.setPosition(LEFT_PIN_UP);
    }

    public void Step3() throws InterruptedException{
        NervousTick(5);
        pinLeft.setPosition(LEFT_PIN_UP);
        pinRight.setPosition(RIGHT_PIN_UP);
        ArmThrow(5);
        armRight.setPosition(RIGHT_WING_UP);
        armLeft.setPosition(LEFT_PIN_UP);
        Cheer(5);
        allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);
        allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);
    }

    public void Step4() throws InterruptedException{
        Move(36, 0.75, 25000, "DriveForwardFar");
        Cheer(5);
        allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);
        allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);
    }

    public void Step5() throws InterruptedException{
        Turn(900, 0.75, 50000, "Turn 2.5 times");
        Cheer(5);
        allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);
        allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);
    }

    public void Step7() throws InterruptedException{
        Move(18, 0.75, 25000, "DriveForwardShort");
        Cheer(5);
        allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);
        allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);
    }

    public void Step8() throws InterruptedException{
        Turn(740, 0.75, 50000, "Turn 2 times");
        Cheer(5);
        allClearLeft.setPosition(ALLCLEAR_LEFT_DOWN);
        allClearRight.setPosition(ALLCLEAR_RIGHT_DOWN);
    }

}
