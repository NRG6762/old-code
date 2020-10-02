package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
 * Team:   FTC NRG6762
 * School: Northwest Regional 7
 * Author: Aiden Maraia
 */
public class HardwareRobot {

    /* Public OpMode members. */
    public DcMotor     DrivePos1       = null;
    public DcMotor     DrivePos2       = null;
    public DcMotor     DrivePos3       = null;
    public DcMotor     DrivePos4       = null;
    public DcMotor     BallLaunch1     = null;
    public DcMotor     BallLaunch2     = null;
    public DcMotor     Sweeper         = null;
    public DcMotor     CapLift         = null;
    public CRServo     SweeperLift     = null;
    public Servo       BallLift        = null;
    public Servo       BallWall        = null;
    public Servo       BeaconPusher    = null;
    public BNO055IMU   IMU             = null;
    public ColorSensor RGBRAMP         = null;
    public ColorSensor RGBBOTTOMSIDE   = null;
    public ColorSensor RGBBOTTOMCENTER = null;
    public ColorSensor RGBBEACON       = null;
    public TouchSensor LIMIT           = null;
    public OpticalDistanceSensor ODS   = null;
    public DeviceInterfaceModule cdim  = null;


    //Set Digital Ports
    static final int   RGB_LED_RAMP = 0;
    static final int   RGB_LED_BOTTOMSIDE = 1;
    static final int   RGB_LED_BOTTOMCENTER = 2;
    static final int   RGB_LED_BEACON = 3;
    static final int   LIMITSWITCH_1 = 4;
    static final int   BEACONLIMITSWITCH = 5;
    static final int   ALLIANCE_SWITCH = 6;


    /* Local OpMode members. */
    HardwareMap hwMapRobot = null;

    /* Constructor */
    public HardwareRobot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMapRobot = ahwMap;

        // Define and Initialize Motors
        DrivePos1   = hwMapRobot.dcMotor.get("drive1");
        DrivePos2   = hwMapRobot.dcMotor.get("drive2");
        DrivePos3   = hwMapRobot.dcMotor.get("drive3");
        DrivePos4   = hwMapRobot.dcMotor.get("drive4");
        BallLaunch1 = hwMapRobot.dcMotor.get("ball1");
        BallLaunch2 = hwMapRobot.dcMotor.get("ball2");
        Sweeper     = hwMapRobot.dcMotor.get("sweeper");
        CapLift     = hwMapRobot.dcMotor.get("caplift");

        // Define and Initialize servos
        SweeperLift  = hwMapRobot.crservo.get("sweeperlift");
        BallLift     = hwMapRobot.servo.get("balllift");
        BallWall     = hwMapRobot.servo.get("ballwall");
        BeaconPusher = hwMapRobot.servo.get("beaconpusher");

        //Define Motor Directions
        DrivePos1.setDirection(DcMotor.Direction.REVERSE);
        DrivePos2.setDirection(DcMotor.Direction.FORWARD);
        DrivePos3.setDirection(DcMotor.Direction.REVERSE);
        DrivePos4.setDirection(DcMotor.Direction.FORWARD);
        BallLaunch1.setDirection(DcMotor.Direction.REVERSE);
        BallLaunch2.setDirection(DcMotor.Direction.FORWARD);
        Sweeper.setDirection(DcMotor.Direction.FORWARD);
        CapLift.setDirection(DcMotor.Direction.FORWARD);
        BallLift.setDirection(Servo.Direction.FORWARD);
        BallWall.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        DrivePos1.setPower(0);
        DrivePos2.setPower(0);
        DrivePos3.setPower(0);
        DrivePos4.setPower(0);
        BallLaunch1.setPower(0);
        BallLaunch2.setPower(0);
        Sweeper.setPower(0);
        CapLift.setPower(0);

        // Set servo positions and power
        SweeperLift.setPower(0.0);
        BallLift.setPosition(0.0);
        BallWall.setPosition(1.0);
        BeaconPusher.setPosition(0.0);

        // Set all motors to run with encoders.
        // May want to use RUN_WITHOUT_ENCODERS if encoders aren't installed.
        DrivePos1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DrivePos2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DrivePos3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DrivePos4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DrivePos1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DrivePos2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DrivePos3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DrivePos4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BallLaunch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BallLaunch2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CapLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set ball launcher to float (broke a gearbox before this addition)
        BallLaunch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BallLaunch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set parameters for initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";

        // Initialize the IMU
        try {
            IMU = hwMapRobot.get(BNO055IMU.class, "IMU");
            IMU.initialize(parameters);
        }
        catch (Exception e)
        {
            //Failed to initiate IMU
        }

        //Define and initialize RGB and touch sensors
        cdim = hwMapRobot.deviceInterfaceModule.get("cdim");
        // RGB1 = hwMapRobot.colorSensor.get("rgb1");
        RGBBOTTOMSIDE = hwMapRobot.colorSensor.get("rgb2");
        RGBBOTTOMCENTER = hwMapRobot.colorSensor.get("rgb3");
        RGBBEACON = hwMapRobot.colorSensor.get("rgb4");
        LIMIT = hwMapRobot.touchSensor.get("limit");
        ODS = hwMapRobot.opticalDistanceSensor.get("ods");

        // Set the digital channels
        // Remember, the AdaFruit Sensor is actually two devices.
        // It's an I2C Sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(RGB_LED_RAMP, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelMode(RGB_LED_BOTTOMSIDE, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelMode(RGB_LED_BOTTOMCENTER, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelMode(RGB_LED_BEACON, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelMode(LIMITSWITCH_1, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(BEACONLIMITSWITCH, DigitalChannelController.Mode.INPUT);
        cdim.setDigitalChannelMode(ALLIANCE_SWITCH, DigitalChannelController.Mode.INPUT);

        //TURN OFF LED TO CONSERVE BATTERY
        cdim.setDigitalChannelState(RGB_LED_RAMP, false);
        cdim.setDigitalChannelState(RGB_LED_BOTTOMSIDE, false);
        cdim.setDigitalChannelState(RGB_LED_BOTTOMCENTER, false);
        cdim.setDigitalChannelState(RGB_LED_BEACON, false);

    }

}

