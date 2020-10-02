package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RoverHardware{

    //Drive Train Motors
    public DcMotor  drive1  = null;
    public DcMotor  drive2  = null;
    public DcMotor  drive3  = null;
    public DcMotor  drive4  = null;

    //Internal IMU
    public BNO055IMU imu    = null;

    //Local Op Mode Members
    HardwareMap rvMap = null;

    //Necessary Constructor
    public RoverHardware(){}

    //Initialize Mapped Hardware
    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        rvMap = ahwMap;

        /*DRIVE TRAIN INITIALIZATION*/

        //Retrieve and Initialize Drive Train Motors
        drive1 = rvMap.get(DcMotor.class, "drive1");
        drive2 = rvMap.get(DcMotor.class, "drive2");
        drive3 = rvMap.get(DcMotor.class, "drive3");
        drive4 = rvMap.get(DcMotor.class, "drive4");

        //Set Drive Train Directions
        drive1.setDirection(DcMotor.Direction.FORWARD);
        drive2.setDirection(DcMotor.Direction.FORWARD);
        drive3.setDirection(DcMotor.Direction.FORWARD);
        drive4.setDirection(DcMotor.Direction.FORWARD);

        //Set Drive Train Power to 0
        drive1.setPower(0);
        drive2.setPower(0);
        drive3.setPower(0);
        drive4.setPower(0);

        //Set Drive Train Mode to Run Without The Encoders
        drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*IMU INITIALIZATION*/

        //Set IMU Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Retrieve and Initialize the IMU
        imu = rvMap.get(BNO055IMU.class, "imu");
    }
}
