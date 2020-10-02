package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.File;
import java.io.FileReader;
import java.io.LineNumberReader;
import java.util.Locale;


/*
 * Team:   FTC NRG6762
 * School: Northwest Regional 7
 * Author: Aiden Maraia
 */
@TeleOp(name="Benji: TeleOp", group="@TeleOp")

//@Disabled
public class BenjiTeleOP extends LinearOpMode{

     //Declare OpMode members. Use the class created to define the robot's hardware
    private HardwareRobot robot = new HardwareRobot();

    //Sensor boolean (used to quickly activate or deactivate sensors)
    boolean aIMU  = false;
    boolean aRGB3 = false;
    boolean aLED3 = false;
    boolean aALLI = true;
    boolean aLIM1 = true;
    boolean aLIMT = true;

    //Telemetry boolean (used to turn on any valid telemetry at the right time)
    boolean joyPosTele        = false;
    boolean joyPolarCoordTele = false;
    boolean wheelScalersTele  = false;
    boolean ballLauncherTele  = false;

    //Button boolean
    boolean gCurrState        = false;
    boolean gPrevState        = false;
    boolean tCurrState        = false;
    boolean tPrevState        = false;

    //Alliance color boolean
    boolean allianceColor     = false;

    //Find the angles of the IMU
    private Orientation  angles;
    private double       startHeading;
    private double       lastHeading;

    //Digital CDIM Devices
    private static final int LED_CHANNEL_3   = 2;
    private static final int LIMITSWITCH_1   = 4;
    private static final int ALLIANCE_SWITCH = 6;

    //Ints. to treat buttons as switches
    private int     dChangeSwitch = 0;
    private int     shooterSwitch = 0;

    //RGB values and color values
    private double  red = 325.0;
    private double  blue = 275.0;
    private float   hsv3Values[];

    //Ball Lift Positions
    private double ballLiftUp   = 0.0;
    private double ballLiftDown = 0.5;
    private double ballWallUp   = 0.4;
    private double ballWallDown = 0.8;

    //BallCycleTiming
    private double ballStartTime = 0;
    private double ballLaunchTriggerCycles = 0;

    //Trigger Dance
    private boolean doIDance = false;


    @Override public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()){
            processLoop();
        }
    }


    //Code to run ONCE when opmode is started
      public void initialize() {

        //Initialize the hardware variables. The init() method of the hardware class does all the work here
        telemetry.addData("Init", "Initializing Hardware Map");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addData("Init", "Initialized Hardware Map");
        telemetry.update();

        //Find and read a text file for the initial heading at the beginning of autonomous
        telemetry.addData("Init", "Searching For Existing File");
        telemetry.update();
        File headingText = new File("/sdcard/FIRST/heading.txt");
        if (headingText.exists() && headingText.isFile()){
            try{
                telemetry.addData("Init", "Reading Heading");
                telemetry.update();
                FileReader fr = new FileReader(headingText);
                LineNumberReader ln = new LineNumberReader(fr);
                while (ln.getLineNumber() == 0){
                    String number = ln.readLine();
                    startHeading = Double.parseDouble(number);
                    telemetry.addData("Init", "Read Heading");
                    telemetry.update();
                }
                headingText.delete();
            }catch (Exception e){
                telemetry.addData("Init", "Bada Boom - Error Reading File");
                telemetry.update();
            }
        }else{
            telemetry.addData("Init", "No Heading File Found");
            telemetry.update();
            if(aIMU){
                    telemetry.addData("Init", "Reading Heading From Compass");
                    telemetry.update();
                    try
                    {
                        angles = robot.IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY).toAngleUnit(angles.angleUnit);
                        startHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                    }
                    catch (Exception e)
                    {
                        aIMU = false;
                        telemetry.addData("Init", "IMU blew up, disabled");
                        telemetry.update();
                    }
                    angles = robot.IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY).toAngleUnit(angles.angleUnit);
                    startHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            }else{
                telemetry.addData("Init", "IMU Disabled, Using 0.0 as heading");
                telemetry.update();
                startHeading = 0.0;
            }
        }

        //Find the alliance color based on the alliance color switch
       // if(aALLI){allianceColor = robot.cdim.getDigitalChannelState(ALLIANCE_SWITCH);}

       // if(aRGB3){float hsv3Values[] = {0f,0f,0f};}

        // Send telemetry message to signify robot waiting
        telemetry.clearAll();
        telemetry.addData("Init Complete", "Hello Driver");
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    public void processLoop() {

        if(gamepad1.guide){
            if(doIDance){
                doIDance = false;
            }else if(!doIDance){
                doIDance = true;
            }
        }

        if(doIDance) {
            dance();
        }else {
            //Create and set values to used variables
            double speed = 0, angle = 0, dchange = 0;
            double retValues[];

            //Instance of cartesianToPolar method used for changing the cartesian values into polar values.
            retValues = cartesianToPolar(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            //Set retValues array to shown values. retValues is used to return the variables used in multiple methods.
            speed = retValues[0];
            angle = retValues[1];
            dchange = retValues[2];

            //Instance of polarToWheelSpeed method used for powering wheels
            polarToWheelSpeed(speed, angle, dchange);

            //Instance of ballShooter method used for angling and using the ballshooter
            ballShooter();

            //Update the Telemetry
            telemetry.update();
        }
    }


    public void dance(){

        mecanum(1.0,0.0,0.0);

        sleep(1000);

        mecanum(0.0,0.0,1.0);

        sleep(1000);

        mecanum(-1.0,0.0,0.0);

        sleep(1000);

        mecanum(0.0,0.0,0.0);

        eat(1.0);

        sleep(10000);

        eat(0.0);

        fire();

        mecanum(1.0,45.0,0.0);

        sleep(750);

        mecanum(1.0,225.0,0.0);

        sleep(750);

        mecanum(1.0,315.0,0.0);

        sleep(750);

        mecanum(1.0,135.0,0.0);

        sleep(750);

        mecanum(1.0,225.0,0.0);

        sleep(750);

        mecanum(1.0,45.0,0.0);

        sleep(750);

        mecanum(1.0,135.0,0.0);

        sleep(750);

        mecanum(1.0,315.0,0.0);

        sleep(750);

        mecanum(0.0,0.0,1.0);

        fire();

        mecanum(0.0,0.0,-1.0);

        fire();

        doIDance = false;
    }

    private double[] cartesianToPolar(double y1, double x1, double x2) {

        //Reset retValues to 0 for later use
        double[] retValues = new double []{0.0,0.0,0.0};
        double speed = 0.0, angle=0.0, dchange=0.0;

        //Create a deadzone for the main drive sticks
        gamepad1.setJoystickDeadzone((float) 0.05);

        //Change joypad values into useful polar values
        speed = Math.sqrt((y1 * y1) + (x1 * x1));
        angle = Math.atan2(x1, -y1);
        dchange = -x2 / 3.33;

        //Rewrite the last heading after the last hardware cycle
        if (aIMU) {
            angles = robot.IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY).toAngleUnit(angles.angleUnit);
            lastHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            // check the status of the guide button on gamepad.
            gCurrState = gamepad1.guide;

            // check for button-press state transitions.
            if ((gCurrState) && (gCurrState != gPrevState))  {
                // button is transitioning to a pressed state.
                dChangeSwitch = 1;
                gPrevState = true;
            } else if ((!gCurrState) && (gCurrState != gPrevState)){
                dChangeSwitch = 0;
                gPrevState = false;
            }

            //Spin the robot back to forward if the switch is off
            if ((dChangeSwitch == 0) && (dchange == 0.0)) {
                if (lastHeading > startHeading) {
                    dchange = -1.0 / 3.33;
                } else if (lastHeading < startHeading) {
                    dchange = 1.0 / 3.33;
                } else if (lastHeading == startHeading) {
                    dchange = 0.0;
                }
            }
        }

        //Joypad input Telemetry
        if (joyPosTele) {
            telemetry.addData("X1: ",x1);
            telemetry.addData("Y1: ",y1);
            telemetry.addData("X2: ",x2);
        }

        //Polar values Telemetry
        if (joyPolarCoordTele) {
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Angle: ", angle);
            telemetry.addData("Rotational Change: ", dchange);
        }

        //Add polar values to retValues array
        retValues[0] = speed;
        retValues[1] = angle;
        retValues[2] = dchange;
        return retValues;
    }

    private void polarToWheelSpeed(double speed, double angle, double dchange){

        //Create Variables used only in this method
        double pos1, pos2, pos3, pos4, maxValue;

        //Define unscaled voltage multipliers
        pos1 = speed*Math.sin(angle+(Math.PI/4))+dchange;
        pos2 = speed*Math.cos(angle+(Math.PI/4))-dchange;
        pos3 = speed*Math.cos(angle+(Math.PI/4))+dchange;
        pos4 = speed*Math.sin(angle+(Math.PI/4))-dchange;

        //VOLTAGE MULTIPLIER SCALER

        //Set maxValue to pos1 absolute
        maxValue = Math.abs(pos1);

        //If pos2 absolute is greater than maxValue, then make maxValue equal to pos2 absolute
        if(Math.abs(pos2) > maxValue){maxValue = Math.abs(pos2);}

        //If pos3 absolute is greater than maxValue, then make maxValue equal to pos3 absolute
        if(Math.abs(pos3) > maxValue){maxValue = Math.abs(pos3);}

        //If pos4 absolute is greater than maxValue, then make maxValue equal to pos4 absolute
        if(Math.abs(pos4) > maxValue){maxValue = Math.abs(pos4);}

        //Check if need to scale -- if not set to 1 to nullify scale
        if (maxValue <= 1){ maxValue = 1;}

        //Power motors with scaled voltage multipliers
        robot.DrivePos1.setPower(pos1/maxValue);
        robot.DrivePos2.setPower(pos2/maxValue);
        robot.DrivePos3.setPower(pos3/maxValue);
        robot.DrivePos4.setPower(pos4/maxValue);

        //Scaled Voltage Multiplier Telemetry
        if (wheelScalersTele) {
            telemetry.addData("Wheel 1 W/ Scale: ",pos1/maxValue);
            telemetry.addData("Wheel 2 W/ Scale: ",pos2/maxValue);
            telemetry.addData("Wheel 3 W/ Scale: ",pos3/maxValue);
            telemetry.addData("Wheel 4 W/ Scale: ",pos4/maxValue);
        }
    }

    private void ballShooter(){

        //Get the current object color on the sweeper ramp
        if(aRGB3){Color.RGBToHSV((robot.RGBRAMP.red() * 255) / 800, (robot.RGBRAMP.green() * 255) / 800, (robot.RGBRAMP.blue() * 255) / 800, hsv3Values);}

        //Control the ball lift using your right index finger
        if((aLIM1)&&(aLIMT)) {
            if (!robot.cdim.getDigitalChannelState(LIMITSWITCH_1) || !robot.LIMIT.isPressed()) {
                if (gamepad2.right_trigger == 1) {
                    robot.SweeperLift.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.SweeperLift.setPower(1.0);
                } else if (gamepad2.right_bumper) {
                    robot.SweeperLift.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.SweeperLift.setPower(1.0);
                } else {
                    robot.SweeperLift.setPower(0.0);
                }
            } else if (robot.cdim.getDigitalChannelState(LIMITSWITCH_1)) {
                if (gamepad2.right_bumper) {
                    robot.SweeperLift.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.SweeperLift.setPower(1.0);
                } else {
                    robot.SweeperLift.setPower(0.0);
                }
            } else if (robot.LIMIT.isPressed()) {
                if (gamepad2.right_trigger == 1) {
                    robot.SweeperLift.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.SweeperLift.setPower(1.0);
                } else {
                    robot.SweeperLift.setPower(0.0);
                }
            } else {
                robot.SweeperLift.setPower(0.0);
            }
        }else{
            if (gamepad2.right_trigger == 1) {
                robot.SweeperLift.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.SweeperLift.setPower(1.0);
            } else if (gamepad2.right_bumper) {
                robot.SweeperLift.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.SweeperLift.setPower(1.0);
            } else {
                robot.SweeperLift.setPower(0.0);
            }
        }

        // check the status of the guide button on gamepad.
        tCurrState = gamepad1.right_bumper;

        // check for button-press state transitions.
        if ((tCurrState) && (tCurrState != tPrevState))  {
            // button is transitioning to a pressed state.
            tPrevState = true;
        } else if ((!tCurrState) && (tCurrState != tPrevState)){
            tPrevState = false;
        }

        //Control the ball launcher using your right thumb
        if(gamepad2.a){

            if (ballLaunchTriggerCycles == 0) {
                ballStartTime = getRuntime();
                robot.BallLaunch1.setPower(1.0);
                robot.BallLaunch2.setPower(1.0);
            }

            ballLaunchTriggerCycles = getRuntime() - ballStartTime;

            //Control the ball wall after the ball lift is moved and button is held down
            if (ballLaunchTriggerCycles >= 0.0 && ballLaunchTriggerCycles < 0.5) {
                robot.BallWall.setPosition(ballWallDown);
            }else if (ballLaunchTriggerCycles >= 0.50 && ballLaunchTriggerCycles < 1.0) {
                robot.BallLift.setPosition(ballLiftDown);
            }else if (ballLaunchTriggerCycles >= 1.00 && ballLaunchTriggerCycles < 1.5) {
                robot.BallWall.setPosition(ballWallUp);
                robot.BallLift.setPosition(ballLiftDown);
            }else if (ballLaunchTriggerCycles >= 1.5 && ballLaunchTriggerCycles < 2.0) {
                robot.BallLift.setPosition(ballLiftUp);
            }else if (ballLaunchTriggerCycles >= 2.0){
                ballLaunchTriggerCycles = 0;
            }

            tPrevState = false;
        }else{
            ballLaunchTriggerCycles = 0;
            robot.BallLift.setPosition(ballLiftDown);
            robot.BallWall.setPosition(ballWallUp);
            robot.BallLaunch1.setPower(0.0);
            robot.BallLaunch2.setPower(0.0);
        }

        //Control the sweeper using your right thumb and RGB3
        if(aLED3){robot.cdim.setDigitalChannelState(LED_CHANNEL_3,true);}
        if(aRGB3) {
            if (allianceColor) {
                if (hsv3Values[0] >= red) {
                    robot.Sweeper.setPower(0.0);
                } else {
                    if (gamepad2.b) {
                        robot.Sweeper.setPower(1.0);
                    } else {
                        robot.Sweeper.setPower(0.0);
                    }
                }
            } else if (!allianceColor) {
                if ((hsv3Values[0] <= blue) && (hsv3Values[0] >= 200)) {
                    robot.Sweeper.setPower(0.0);
                } else {
                    if (gamepad2.b) {
                        robot.Sweeper.setPower(1.0);
                    } else {
                        robot.Sweeper.setPower(0.0);
                    }
                }
            }
        }else{
            if (gamepad2.b) {
                robot.Sweeper.setPower(1.0);
            } else {
                robot.Sweeper.setPower(0.0);
            }
        }
        if(aLED3){robot.cdim.setDigitalChannelState(LED_CHANNEL_3,false);}

        //Ball Launching Telemetry
        if(ballLauncherTele){
            if (robot.SweeperLift.getPower() != 0.0) {telemetry.addData("Sweeper Lift Direction: ", robot.SweeperLift.getDirection());}
            if (robot.BallLift.getPosition() != 0.0) {telemetry.addData("Ball Lift Position: ", robot.BallLift.getPosition());}
            if (robot.BallLift.getPosition() != 0.0) {telemetry.addData("Ball Lift Position: ", robot.BallLift.getPosition());}
            if (robot.BallWall.getPosition() != 0.0) {telemetry.addData("Ball Lift Position: ", robot.BallWall.getPosition());}
            if (robot.BallLaunch1.getPower() != 0.0) {telemetry.addLine("Ball Launcher Is Running");}
            if (robot.Sweeper.getPower() != 0.0) {telemetry.addLine("Sweeper Is Running");}
        }
    }


    private void mecanum(double speed, double degrees, double spin){

        //Change joypad values into useful polar values
        double angle = (degrees * Math.PI)/ 180;
        double dchange = -spin / 3.33;

        double pos1, pos2, pos3, pos4, maxValue;

        //Define unscaled voltage multipliers
        pos1 = speed*Math.sin(angle+(Math.PI/4))+dchange;
        pos2 = speed*Math.cos(angle+(Math.PI/4))-dchange;
        pos3 = speed*Math.cos(angle+(Math.PI/4))+dchange;
        pos4 = speed*Math.sin(angle+(Math.PI/4))-dchange;

        //VOLTAGE MULTIPLIER SCALER

        //Set maxValue to pos1 absolute
        maxValue = Math.abs(pos1);

        //If pos2 absolute is greater than maxValue, then make maxValue equal to pos2 absolute
        if(Math.abs(pos2) > maxValue){maxValue = Math.abs(pos2);}

        //If pos3 absolute is greater than maxValue, then make maxValue equal to pos3 absolute
        if(Math.abs(pos3) > maxValue){maxValue = Math.abs(pos3);}

        //If pos4 absolute is greater than maxValue, then make maxValue equal to pos4 absolute
        if(Math.abs(pos4) > maxValue){maxValue = Math.abs(pos4);}

        //Check if need to scale -- if not set to 1 to nullify scale
        if (maxValue <= 1){ maxValue = 1;}

        //Power motors with scaled voltage multipliers
        robot.DrivePos1.setPower(pos1/maxValue);
        robot.DrivePos2.setPower(pos2/maxValue);
        robot.DrivePos3.setPower(pos3/maxValue);
        robot.DrivePos4.setPower(pos4/maxValue);
    }


    public void eat(double speed){
        robot.Sweeper.setPower(speed);
    }

    public void fire(){
        robot.BallLaunch1.setPower(1.0);
        robot.BallLaunch2.setPower(1.0);
        robot.BallWall.setPosition(ballWallDown);
        sleep(500);
        robot.BallLift.setPosition(ballLiftDown);
        sleep(500);
        robot.BallWall.setPosition(ballWallUp);
        robot.BallLift.setPosition(ballLiftDown);
        sleep(500);
        robot.BallLift.setPosition(ballLiftUp);
        sleep(500);
        robot.BallLaunch1.setPower(0.0);
        robot.BallLaunch1.setPower(0.0);
    }

    //Grab a specific angle
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatRadians(AngleUnit.RADIANS.fromUnit(angleUnit, angle));
    }

    //Grab the angle needed in radians
    private String formatRadians(double radians){
        return String.format(Locale.getDefault(), "%.3f", AngleUnit.RADIANS.normalize(radians));
    }
}
