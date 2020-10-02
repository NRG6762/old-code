package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Drive Motor 1: "drive1"
 * Motor channel:  Drive Motor 2: "drive2"
 * Motor channel:  Drive Motor 3: "drive3"
 * Motor channel:  Drive Motor 4: "drive4"
 */
public class HardwareCheck
{
    //Declare OpMode members. Use the class created to define the robot's hardware
    private HardwareRobot robot = new HardwareRobot();

    //Set Digital Ports
    static final int   LED_CHANNEL_1 = 0;
    static final int   LED_CHANNEL_2 = 1;
    static final int   LED_CHANNEL_3 = 2;
    static final int   LED_CHANNEL_4 = 3;
    static final int   LIMITSWITCH_1 = 4;
    static final int   LIMITSWITCH_2 = 5;
    static final int ALLIANCE_SWITCH = 6;


    /* Local OpMode members. */
    HardwareMap hwMapRobot = null;

    /* Constructor */
    public HardwareCheck(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {



    }
}

