package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public class RoverHardware{

    //Drive Train Motors
    public DcMotor  drive1  = null;
    public DcMotor  drive2  = null;
    public DcMotor  drive3  = null;
    public DcMotor  drive4  = null;

    //Lift Motor
    public DcMotor  lift    = null;
    public DcMotor  pivot   = null;
    public DcMotor  light   = null;

    //Servo Motors
    public Servo    totem   = null;
    public Servo    rake    = null;

    //Internal IMU
    public BNO055IMU imu    = null;

    //Vuforia and TFlow
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public static final String VUFORIA_KEY = "AaDgae3/////AAAAGSOFWa7i/kIwn/7gz/E0MSc3YRNg96lWv4KGfbITtJRop7jfrwfeoXbTjypSxLr2E+Rg7NHObqNYr86Ctj8Isq6G41mbX7ZwbTGwxNkZ/ArBY24pWsFXaS34AFJ/Gp7kaFChNm5LfmEZAT5Ri2LE56BJv4QWfutqhg9AiM3jHHstiz01dWLer39IumAdCEnwsApzTc6VK01bqroxVYvNl+AEY0stH6d0lZImc+PdKS9yXBqYecKMZG+qgMSYdZ08GNj1zpb6vSKqmOfOC9om9xbqLt1sWIEsbdukRNlVbwmjAVngsfyqbvOMfyJFYXYrsHnGl2IamsjIUyx7RsQd6lvUFXt2G2oN9bubIZt28YNF";

    //System Activators
    public boolean drive    = true;
    public boolean liftB    = true;
    public boolean BNO      = true;
    public boolean totemB   = true;
    public boolean vuforiaB = true;
    public boolean rakeB    = true;
    public boolean lightB   = true;

    /*private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;*/

    //Local Op Mode Members
    HardwareMap rvMap       = null;

    //Necessary Constructor
    public RoverHardware(){}

    //Initialize Mapped Hardware
    public void init(HardwareMap ahwMap, boolean autonomous) {
        //Save reference to Hardware map
        rvMap = ahwMap;

        /*DRIVE TRAIN INITIALIZATION*/
        if(drive){

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

            if(autonomous){
                drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
        }

        /*LIFT INITIALIZATION*/
        if(liftB){
            lift = rvMap.get(DcMotor.class, "lift");
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pivot = rvMap.get(DcMotor.class, "pivot");
            pivot.setDirection(DcMotor.Direction.FORWARD);
            pivot.setPower(0);
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Totem Initialization
        if(totemB){

            totem = rvMap.get(Servo.class, "totem");

            totem.setDirection(Servo.Direction.FORWARD);

            totem.setPosition(0.2);
            totem.scaleRange(0,.8);
        }

        if(rakeB){

            rake = rvMap.get(Servo.class, "rake");

            rake.setDirection(Servo.Direction.FORWARD);

            rake.setPosition(0.2);
        }

        if(lightB){
            light = rvMap.get(DcMotor.class, "light");
            light.setDirection(DcMotor.Direction.FORWARD);
            light.setPower(0);
            light.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*IMU INITIALIZATION*/
        if(BNO){
            //Set IMU Parameters
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile  = "AdafruitIMUCalibration.json";
            parameters.loggingEnabled       = true;
            parameters.mode                 = BNO055IMU.SensorMode.IMU;
            parameters.loggingTag           = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            //Retrieve and Initialize the IMU
            imu = rvMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);
        }

        if(vuforiaB && autonomous){
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            int tfodMonitorViewId = rvMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", rvMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }

    }

}
