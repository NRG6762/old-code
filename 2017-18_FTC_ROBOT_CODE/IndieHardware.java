package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * Team:   FTC NRG6762
 * School: Northwest Regional 7
 * Author: Aiden Maraia
 */
public class IndieHardware {

    /* Public OpMode members. */
    public DcMotor DrivePos1            = null;
    public DcMotor DrivePos2            = null;
    public DcMotor DrivePos3            = null;
    public DcMotor DrivePos4            = null;
    public DcMotor ScissorArmPwr        = null;
    public DcMotor ScissorArmLiftPwr    = null;
    public DcMotor BlockLift            = null;
    public Servo GripperLeft            = null;
    public Servo GripperRight           = null;
    public Servo ScissorRotate          = null;
    public Servo RelicGripper           = null;
    public Servo RelicPivot             = null;
    public Servo JewelLeft              = null;
    public Servo JewelRight             = null;
    public I2cDeviceSynch Pixy          = null;
    public DigitalChannel BallLiftLow   = null;
    public DigitalChannel LeftRGBLED    = null;
    public DigitalChannel RightRGBLED   = null;
    public DigitalChannel SCIWall       = null;
    public DigitalChannel SCIBackLeft   = null;
    public DigitalChannel SCIBackRight  = null;
    public DigitalChannel LIFTInAll     = null;
    public VuforiaLocalizer Vision      = null;
    public BNO055IMU Imu                = null;
    public ColorSensor LeftRGB          = null;
    public ColorSensor RightRGB         = null;

    //Motor/Sensor Activators
    public boolean driveBool        = true;
    public boolean scissorBool      = false;
    public boolean blockLiftBool    = true;
    public boolean pixyBool         = true;
    public boolean liftLimitBool    = true;
    public boolean vuforiaBool      = true;
    public boolean imuBool          = true;
    public boolean colorBool        = true;
    public boolean colorLEDBool     = true;
    public boolean scissorLimitsBool= false;
    public boolean jewelBool        = true;

    /* Local OpMode members. */
    HardwareMap hwMapRobot = null;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;


    /* Constructor */
    public IndieHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMapRobot = ahwMap;

        if (driveBool) {
            // Define and Initialize Drive Motors
            DrivePos1 = hwMapRobot.dcMotor.get("drive1");
            DrivePos2 = hwMapRobot.dcMotor.get("drive2");
            DrivePos3 = hwMapRobot.dcMotor.get("drive3");
            DrivePos4 = hwMapRobot.dcMotor.get("drive4");

            //Define Drive Motor Directions
            DrivePos1.setDirection(DcMotor.Direction.REVERSE);
            DrivePos2.setDirection(DcMotor.Direction.FORWARD);
            DrivePos3.setDirection(DcMotor.Direction.REVERSE);
            DrivePos4.setDirection(DcMotor.Direction.FORWARD);

            // Set all drive motors to zero power
            DrivePos1.setPower(0);
            DrivePos2.setPower(0);
            DrivePos3.setPower(0);
            DrivePos4.setPower(0);

            //Reset all drive motor encoders
            DrivePos1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DrivePos2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DrivePos3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DrivePos4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set all drive motors to run with encoders.
            DrivePos1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DrivePos2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DrivePos3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DrivePos4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (blockLiftBool) {
            //Define block lift servo/dc motors
            BlockLift = hwMapRobot.dcMotor.get("blocklift");
            GripperLeft = hwMapRobot.servo.get("gripperleft");
            GripperRight = hwMapRobot.servo.get("gripperright");

            //Define block lift servo/dc motors directions
            BlockLift.setDirection(DcMotor.Direction.REVERSE);
            GripperLeft.setDirection(Servo.Direction.REVERSE);
            GripperRight.setDirection(Servo.Direction.FORWARD);

            //Set their power/position
            BlockLift.setPower(0);
            GripperLeft.setPosition(0);
            GripperRight.setPosition(0);

            //Set their run-mode/range
            BlockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BlockLift.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
            GripperLeft.scaleRange(.3, 1);
            GripperRight.scaleRange(0, .7);
        }

        if (scissorBool) {
            ScissorArmPwr = hwMapRobot.dcMotor.get("scissor");
            ScissorArmPwr.setDirection(DcMotor.Direction.FORWARD);
            ScissorArmPwr.setPower(0);
            ScissorArmPwr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ScissorArmPwr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            ScissorArmLiftPwr = hwMapRobot.dcMotor.get("scissorLift");
            ScissorArmLiftPwr.setDirection(DcMotor.Direction.FORWARD);
            ScissorArmLiftPwr.setPower(0);
            ScissorArmLiftPwr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ScissorArmLiftPwr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*ScissorRotate = hwMapRobot.servo.get("scissorRotate");
            ScissorRotate.setDirection(Servo.Direction.FORWARD);
            ScissorRotate.setPosition(.5);*/

            RelicGripper = hwMapRobot.servo.get("relicGripper");
            RelicGripper.setDirection(Servo.Direction.FORWARD);
            RelicGripper.setPosition(0);

            RelicPivot = hwMapRobot.servo.get("relicpRotate");
            RelicPivot.setDirection(Servo.Direction.FORWARD);
            RelicPivot.setPosition(0.0);
        }

        if(pixyBool){
            //setting up Pixy to the hardware map
            Pixy = hwMapRobot.i2cDeviceSynch.get("pixy");
        }

        if(liftLimitBool) {
            BallLiftLow = hwMapRobot.digitalChannel.get("ballliftlow");
            //BallLiftHigh = hwMapRobot.digitalChannel.get("balllifthigh");

            BallLiftLow.setMode(DigitalChannel.Mode.INPUT);
            //BallLiftHigh.setMode(DigitalChannel.Mode.INPUT);
        }

        if(vuforiaBool) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = "AaDgae3/////AAAAGSOFWa7i/kIwn/7gz/E0MSc3YRNg96lWv4KGfbITtJRop7jfrwfeoXbTjypSxLr2E+Rg7NHObqNYr86Ctj8Isq6G41mbX7ZwbTGwxNkZ/ArBY24pWsFXaS34AFJ/Gp7kaFChNm5LfmEZAT5Ri2LE56BJv4QWfutqhg9AiM3jHHstiz01dWLer39IumAdCEnwsApzTc6VK01bqroxVYvNl+AEY0stH6d0lZImc+PdKS9yXBqYecKMZG+qgMSYdZ08GNj1zpb6vSKqmOfOC9om9xbqLt1sWIEsbdukRNlVbwmjAVngsfyqbvOMfyJFYXYrsHnGl2IamsjIUyx7RsQd6lvUFXt2G2oN9bubIZt28YNF";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            Vision = ClassFactory.createVuforiaLocalizer(parameters);
            relicTrackables = Vision.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
        }

        if(imuBool){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile  = "AdafruitIMUCalibration.json";
            parameters.loggingEnabled       = true;
            parameters.mode                 = BNO055IMU.SensorMode.IMU;
            parameters.loggingTag           = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            Imu = hwMapRobot.get(BNO055IMU.class, "IMU");
            Imu.initialize(parameters);
        }

        if(colorBool) {
            LeftRGB = hwMapRobot.colorSensor.get("leftrgb");
            RightRGB = hwMapRobot.colorSensor.get("rightrgb");
        }

        if(colorLEDBool){

            LeftRGBLED = hwMapRobot.digitalChannel.get("lrgbled");
            LeftRGBLED.setMode(DigitalChannel.Mode.OUTPUT);
            LeftRGBLED.setState(false);

            RightRGBLED = hwMapRobot.digitalChannel.get("rrgbled");
            RightRGBLED.setMode(DigitalChannel.Mode.OUTPUT);
            RightRGBLED.setState(false);

        }

        if(scissorLimitsBool){
            SCIBackLeft = hwMapRobot.digitalChannel.get("scissorbackleftlimit");
            SCIBackLeft.setMode(DigitalChannel.Mode.INPUT);

            SCIBackRight = hwMapRobot.digitalChannel.get("scissorbackrightlimit");
            SCIBackRight.setMode(DigitalChannel.Mode.INPUT);

            SCIWall = hwMapRobot.digitalChannel.get("scissorwalllimit");
            SCIWall.setMode(DigitalChannel.Mode.INPUT);

            LIFTInAll = hwMapRobot.digitalChannel.get("liftinlimit");
            LIFTInAll.setMode(DigitalChannel.Mode.INPUT);
        }

        if(jewelBool){
            JewelLeft = hwMapRobot.servo.get("jewelleft");
            JewelLeft.setDirection(Servo.Direction.FORWARD);
            JewelLeft.setPosition(0.0);

            JewelRight = hwMapRobot.servo.get("jewelright");
            JewelRight.setDirection(Servo.Direction.REVERSE);
            JewelRight.setPosition(0.0);
        }
    }
}