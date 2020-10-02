package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/*
 * NRG6762
 * Northwestern Regional 7 Gearheads
 * 2019-2020 Season - Skystone
 * Hardware Class
 * Written by Aiden Maraia
 * Version: 1/21/2020
 * Feel free to make any changes and use at your disposal.
 */
public class BuilderHardware {

    //Declare Robot Sizes
    public double                   robotDepth          = 17.0;
    public double                   backGripper         = 3.0;
    public double                   frontGripper        = 3.0;

    //Declare Distance Sensor Values
    public double                   gripperDown         = 3.8;
    public double                   gripperClearing     = 6;
    public double                   blockHeight         = 4.0;
    public double                   distanceRange       = 0.225;

    //Declare Drive Motors/Activator
    public Boolean                  driveMotorsTrue     = true;
    public DcMotor                  leftFront           = null;
    public DcMotor                  rightFront          = null;
    public DcMotor                  leftBack            = null;
    public DcMotor                  rightBack           = null;

    //Declare Drive Motor Values
    public double                   ticksPerInchFront   = 60;
    public double                   ticksPerInchSide    = 70;

    //Declare Scissor Motors/Activator
    public Boolean                  scissorDriveTrue    = true;
    public DcMotor                  scissor             = null;
    public DcMotor                  scissor2            = null;
    public DcMotor                  linear              = null;

    //Declare Scissor Motor Values
    public double                   scissorRevPerInch   = 6.5;
    public double                   ticksPerScissorInch = 1100;
    public double                   ticksPerLinearInch  = 475;

    //Declare Gripper Mechanism Servos/Activator
    public Boolean                  gripperMechTrue     = true;
    public Servo                    gripper             = null;
    public Servo                    rotator             = null;
    public Servo                    flipper             = null;

    //Declare Gripper Mechanism Positions
    public double                   gripperOpen         = .1;
    public double                   gripperClosed       = .6;
    public double                   rotatorMidPosition  = .5;
    public double                   flipperOpen         = 1.0;
    public double                   flipperClosed       = 0;

    //Declare Foundation Gripper Servos/Activator
    public Boolean                  foundationTrue      = false;
    public Servo                    foundationA         = null;
    public Servo                    foundationB         = null;

    //Declare Foundation Gripper Positions
    public double                   foundationOpen      = 0;
    public double                   foundationClosed    = .65;

    //Declare Color Sensors/Activator
    public Boolean                  colorSensorsTrue    = true;
    public ColorSensor              color               = null;

    //Declare DIM Object/Pins/Activator
    public Boolean                  lightTrue           = true;
    public LED                      colorLight          = null;

    //Delcare Limit Switches/Activator
    public Boolean                  limitTrue           = true;
    public DigitalChannel           limitUpper          = null;
    public DigitalChannel           limitLower          = null;
    public DigitalChannel           limitBack           = null;
    public DigitalChannel           limitFront          = null;

    //Declare IMU/Activator
    public Boolean                  imuTrue             = true;
    public BNO055IMU                imu                 = null;

    //Declare Distance Sensors/Activator
    public Boolean                  distanceTrue        = true;
    public DistanceSensor           distanceScissor     = null;
    public DistanceSensor           distanceFront       = null;

    //Declare Vision Sensors/Objects/Activator
    public Boolean                  visionTrue          = true;
    public boolean                  visionActive;
    public OpenGLMatrix             lastLocation        = null;
    public VuforiaLocalizer         vuforia             = null;
    public VuforiaLocalizer.Parameters     parameters   = null;
    public VuforiaTrackables        targetsSkyStone     = null;
    public VuforiaTrackable         stoneTarget         = null;
    public OpenGLMatrix             robotFromCamera     = null;
    public WebcamName               webcam              = null;

    //Declare Vision Variables
    public float                            phoneXRotate        = 0;
    public float                            phoneYRotate        = 0;
    public float                            phoneZRotate        = 0;
    public VuforiaLocalizer.CameraDirection CAMERA_CHOICE       = BACK;
    public boolean                          PHONE_IS_PORTRAIT   = false;
    public static final float               mmPerInch           = 25.4f;
    public static final float               mmTargetHeight      = (6) * mmPerInch;
    public static final float               stoneZ              = 2.00f * mmPerInch;
    public final float             CAMERA_FORWARD_DISPLACEMENT  = 8.0f * mmPerInch;   // eg: Camera is 8 Inches in front of robot-center
    public final float             CAMERA_VERTICAL_DISPLACEMENT = 3.35f * mmPerInch;   // eg: Camera is 3.35 Inches above ground
    public final float             CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
    private static final String VUFORIA_KEY = "Ae7R6fb/////AAABmVuwcWo6s0kbghzDBT2iWX5a9AzYV1xx4EfFgQsn4UY3YDqBjlyh27OdlzOKfBIEqk2DJBtkB/XZM8zKZqXO/fypt9AaTbfdMF+xn09VNiscSC6kufSp+cnojxzInsxa6fn2Xf435YYSI5i1k/u73PmTbGM9eiokW6Ka6MCLaqVoi14bm+c8hUnL5IFRh2oCeqi2lECV4vXFtl+ZrlvAttCOzRkOZI3lNu45uUfvfcwB0AVskV/G9S9IT4Gga6MBYShUu8ti7Ss0wjcVFbpvUkkjEKooS9i+bEAN696UmYFjOwEUF5iqh1VgAXfJqdwA5Nv0uMueO0plqtCtrWLT5J1Crh52hrYte61CjzleOA4Y";

    //Local OpMode Members
    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    //Constructor
    public BuilderHardware(boolean isAutonomous){

        visionActive = isAutonomous;

    }

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap){

        //Save reference to Hardware map
        hwMap = ahwMap;

        //Drive Motor Initialization
        if(driveMotorsTrue){

            //Get Drive Motors
            leftFront = ahwMap.dcMotor.get("left_front");
            rightFront = ahwMap.dcMotor.get("right_front");
            leftBack = ahwMap.dcMotor.get("left_back");
            rightBack = ahwMap.dcMotor.get("right_back");

            //Reset Drive Motor Encoders
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Set Drive Motor Modes
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Set Drive Motor Directions
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

            //Set Drive Motor Powers
            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);

        }

        //Scissor Drive Motor Initialization
        if(scissorDriveTrue){

            //Get Scissor Motors
            scissor = ahwMap.dcMotor.get("scissor");
            scissor2 = ahwMap.dcMotor.get("scissor2");
            linear = ahwMap.dcMotor.get("linear");

            //Reset Scissor Motor Encoders
            scissor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Set Scissor Motor Modes
            scissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Set Drive Motor Directions
            scissor.setDirection(DcMotorSimple.Direction.FORWARD);
            scissor2.setDirection(DcMotorSimple.Direction.FORWARD);
            linear.setDirection(DcMotorSimple.Direction.REVERSE);

            //Set Scissor Motor Powers
            scissor.setPower(0.0);
            scissor2.setPower(0.0);
            linear.setPower(0.0);

        }

        //Gripping Mechanism Initialization
        if(gripperMechTrue){

            //Get Mechanism Servos
            gripper = ahwMap.servo.get("gripper");
            rotator = ahwMap.servo.get("rotator");
            flipper = ahwMap.servo.get("flipper");

            //Set Mechanism Servo Directions
            gripper.setDirection(Servo.Direction.FORWARD);
            rotator.setDirection(Servo.Direction.FORWARD);
            flipper.setDirection(Servo.Direction.REVERSE);

            //Set Mechanism Servo Positions
            gripper.setPosition(gripperClosed);
            rotator.setPosition(rotatorMidPosition);
            flipper.setPosition(flipperClosed);

        }

        //Foundation Servo Initialization
        if(foundationTrue){

            //Get Foundation Servos
            foundationA = ahwMap.servo.get("foundation_a");
            foundationB = ahwMap.servo.get("foundation_b");

            //Set Foundation Servo Directions
            foundationA.setDirection(Servo.Direction.FORWARD);
            foundationB.setDirection(Servo.Direction.REVERSE);

            //Set Foundation Servo Positions
            foundationA.setPosition(foundationOpen);
            foundationB.setPosition(foundationOpen);

        }

        //Color Sensor Initialization
        if(colorSensorsTrue){

            //Get Color Sensors
            color = ahwMap.colorSensor.get("color");

        }

        //Device Interface Module Initialization
        if(lightTrue){

            //Get Device Interface Module
            colorLight = ahwMap.led.get("led");

            //Set Modes of Used Device Interface Module Pins
            colorLight.enable(false);

        }

        if(limitTrue){

            limitLower = ahwMap.digitalChannel.get("limit_lower");
            limitUpper = ahwMap.digitalChannel.get("limit_upper");
            limitBack = ahwMap.digitalChannel.get("limit_back");
            limitFront = ahwMap.digitalChannel.get("limit_front");

            limitLower.setMode(DigitalChannel.Mode.INPUT);
            limitUpper.setMode(DigitalChannel.Mode.INPUT);
            limitBack.setMode(DigitalChannel.Mode.INPUT);
            limitFront.setMode(DigitalChannel.Mode.INPUT);

        }

        //Internal Measurement Unit Initialization
        if(imuTrue){

            //Create and Get Internal Measurement Unit Parameters
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled       = true;
            parameters.mode                 = BNO055IMU.SensorMode.IMU;
            parameters.loggingTag           = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            //Get Internal Measurement Unit
            imu = ahwMap.get(BNO055IMU.class, "imu");

            //Initialize Internal Measurement Unit with Parameters
            imu.initialize(parameters);
        }

        //Distance Sensor Initialization
        if(distanceTrue){

            //Get Distance Sensor
            distanceScissor = ahwMap.get(DistanceSensor.class, "distance_scissor");
            distanceFront = ahwMap.get(DistanceSensor.class, "distance_front");

        }

        //Vision Initialization
        if(visionTrue && visionActive){

            webcam = ahwMap.get(WebcamName.class, "webcam");

            parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;

            parameters.cameraName = webcam;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

            stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");

            stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }

            // Rotate the phone vertical about the X axis if it's in portrait mode
            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90 ;
            }

            robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */

            ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        }

    }

}

