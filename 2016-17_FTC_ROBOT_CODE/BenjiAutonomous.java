package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name="Benji: Autonomous", group="Autonomous")
public class BenjiAutonomous extends LinearOpMode
{

    //Declare OpMode members. Use the class created to define the robot's hardware
    private HardwareRobot robot = new HardwareRobot();

    private double startTime;
    private double newStartTime;
    private double  loopTime;
    double ballLaunchTriggerCycles = 0;
    int ballLaunchedNumber = 0;

    //Set Run-Time Variables
    private int shootPosition = 1300;                       //In Encoder Ticks
    private double accelTimeMinPowerWheels = 0.500;         //In Seconds
    private double decelMinTicks = 1200;                    //In Encoder Ticks
    private double decelMinCrawlPwr = 0.05;                 //In Power (Using Encoders)
    private double wallLightODS = 0.03;                     //In Detected and Scaled Light
    private double linActuOut = 1.0;                        //In Sensor Position
    private double linActuIn = 0.0;                         //In Sensor Position
    private double RGBBeaconREDThreshold  = 0.43;           //Red Threshold Value
    private double nextBeaconButtonTicks = 100;             //Ticks to next beacon button
    public boolean IMUactive = true;                        //Changes to not Use IMU
    private boolean blueAliance = false;                    //Defaults to Red/Blue Alliance
    private boolean useAllianceSwitch = true;               //Override Alliance Switch with above
    String currMainTask = "moveToShoot";                    //Set Initial Task (Testing)
    public double whiteLineAlphaThreshold = 125;             //Set threshold value for alpha
    public double maxWhiteLineDistance = 800;               //Max encoder ticks looking for wh line

    //Activate different Autonomous first time in booleans
    private boolean firstLoopMoveToShoot = true;
    private boolean firstBallShooterMode = true;
    private boolean firstRotationMode    = true;
    private boolean firstTimeInBeaconPress = true;
    private boolean firstBeaconPress = true;
    private boolean firstAlignToBeacon = true;
    private boolean foundBeacon = false;
    private boolean beaconRed = false;

    //Ball Lift Positions
    private double ballLiftUp   = 0.0;
    private double ballLiftDown = 0.5;
    private double ballWallUp   = 0.4;
    private double ballWallDown = 0.8;


    //Find the angles of the IMU
    private Orientation angles;
    private double       startHeading;
    private double       lastHeading;

    //Various Global Variables Needed By Routines
    private double rotForwardCrabDirection;
    private boolean blnForwardRotationSpinAround = false;
    private double initialPOS = 0.0;
    private double origPos = 0.0;

    @Override public void runOpMode() {
        initialize();

        waitForStart();
        startTime = getRuntime();
        newStartTime = getRuntime();
        while (opModeIsActive()) {
            newLoop();
        }

    }

    public void initialize(){

        //Initialize the hardware variables. The init() method of the hardware class does all the work here
        telemetry.addData("Init", "Initializing Hardware Map");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addData("Init", "Initialized Hardware Map");
        telemetry.update();

        telemetry.addData("Init", "Searching For Existing File");
        telemetry.update();

        telemetry.addData("Init", "Reading Heading From Compass");
        telemetry.update();

        try
        {
            //robot.IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles = robot.IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY);
            startHeading = angles.firstAngle;
        }
        catch (Exception e)
        {
            telemetry.addData("Init", "IMU blew up, disabled");
            telemetry.update();
            startHeading = 0.0;
        }

        File headingText = new File("/sdcard/FIRST/heading.txt");

        //Create the file
        try {
            if (headingText.createNewFile()) {
                telemetry.addLine("File was created!");
                telemetry.update();
            } else {
                telemetry.addLine("File already exists.");
                telemetry.update();
            }
            FileWriter writer = new FileWriter(headingText);
            writer.write(Double.toString(startHeading));
            writer.close();
            telemetry.addLine("File was written.");
            telemetry.update();

        }catch (IOException e){
            telemetry.addLine("File could not be created/written.");
            telemetry.update();
        }

        if (useAllianceSwitch) {
            if (robot.cdim.getDigitalChannelState(robot.ALLIANCE_SWITCH)) {
                blueAliance = true;
                telemetry.addData("Alliance: ", "Blue");
            }
            else {
                blueAliance = false;
                telemetry.addData("Alliance: ", "Red");
            }
        }

        telemetry.addData("Heading: ", startHeading);
        telemetry.update();
    }


    public void newLoop(){

        loopTime = getRuntime() - newStartTime;

        switch (currMainTask) {

            case "moveToShoot":
                /* Move Robot to Shooting Position
                    Task:
                    Drives forward to the desired position using the accel and decel methods.
                 */
                if (firstLoopMoveToShoot) {
                    newStartTime = getRuntime();
                    firstLoopMoveToShoot = false;
                } else {
                    telemetry.addData("POS", robot.DrivePos1.getCurrentPosition());
                    telemetry.update();
                    //Move to next task if shoot position achieved
                    if (robot.DrivePos1.getCurrentPosition() >= shootPosition) {
                        drive(0.0);
                        currMainTask = "shootParticles";
                    }
                    else
                        //Set Motor Drive with Accel and Deccel Limiters in place to prevent jerking of bot
                        drive(decelerationLimiter(robot.DrivePos1.getCurrentPosition(),shootPosition,accelerationLimiter(loopTime,1)));
                }
                break;

            case "shootParticles":
                /* SHOOT PARTICLES:
                    Assumption:
                    Single Particle Loaded in Tube
                    Second Particle Waiting in Hopper
                    Task:
                    Shoot both balls as quickly and accurately as possible.
                 */

                    if(firstBallShooterMode){
                        //Reset loop timer for timing of sequence
                        newStartTime = getRuntime();

                        // Turn on shooter
                        launch(1.0);

                        //Process rest of loop going forward
                        firstBallShooterMode = false;
                    }
                    else {
                        //Timed Sequence To Launch Particles

                        //Do not start until 500 msec to give shooter time to get to speed.
                        if (loopTime >= .65 && loopTime < 1.25) {
                            robot.BallLift.setPosition(ballLiftDown);
                        }
                        if (loopTime >= 1.25 && loopTime < 1.5) {
                            robot.BallWall.setPosition(ballWallUp);
                        }
                        if (loopTime >= 1.5 && loopTime < 2.00) {
                            robot.BallLift.setPosition(ballLiftUp);
                        }
                        if (loopTime >= 2.0 && loopTime < 2.5) {
                            robot.BallWall.setPosition(ballWallDown);
                        }
                        if (loopTime >= 2.50 && loopTime < 3.0) {
                            robot.BallLift.setPosition(ballLiftDown);
                        }
                        if (loopTime >= 3.00) {
                            robot.BallWall.setPosition(ballWallUp);
                            launch(0.0);
                            //currMainTask = "rotateForward";
                        }
                    }
                break;

            case "rotateForward":

                /* Rotate Forward
                    Assumption:
                    This will be called once robot is in launch position and that alliance switch
                    or alliance override is set. IMU and Optical Distance Sensor required although
                    emergency mode for both is in place in case of error or sensor failure. The
                    emergency mode will use encoder and timing and a limit switch to determine pos.
                    Task:
                    Crabs left/right and uses IMU to maintain heading and
                    optical distance sensor to judge when wall is near.
                 */

                if(firstRotationMode){
                    //Reset Loop Timer
                    newStartTime = getRuntime();

                    // Set Values Based on Alliance Switch or Override
                    if (blueAliance){
                        rotForwardCrabDirection = 62;
                    }
                    else
                    {
                        rotForwardCrabDirection = 295;
                        blnForwardRotationSpinAround = true;
                    }

                    //Process remainder of code for additional loops
                    firstRotationMode = false;
                }
                else {

                    if (IMUactive == true) {
                        try {

                            if (robot.IMU.getSystemStatus() == BNO055IMU.SystemStatus.RUNNING_FUSION) {
                                angles = robot.IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY);
                                lastHeading = angles.firstAngle;
                            }
                        } catch (Exception e) {
                            telemetry.addData("IMU Status", robot.IMU.getSystemStatus());
                            telemetry.addData("IMU Error", robot.IMU.getSystemError());
                            telemetry.addData("Error", e.toString());

                        }
                        telemetry.addData("Headings", startHeading + " " + lastHeading);
                        telemetry.addData("IMU Status", robot.IMU.getSystemStatus());

                    }

                    if (blnForwardRotationSpinAround) {
                        mecanum( 0, 0, 0.3);
                        if ((loopTime >= 6) || (IMUactive == true && lastHeading >= 180 ))  {
                            //Reset Loop Timer
                            newStartTime = getRuntime();
                            blnForwardRotationSpinAround = false;
                        }
                    } else {
                        if ((loopTime >= 4.25)  || (robot.ODS.getLightDetected() >= wallLightODS)) {
                            drive(0.0);
                            currMainTask = "alignTo1stBeacon";
                            telemetry.addData("ODS: ", robot.ODS.getLightDetected());
                        } else {
                            //Set Drive Speed Based on Alliance Switch/Override and IMU heading
                            mecanum( 0.75, rotForwardCrabDirection, (lastHeading)*.05);
                            telemetry.addData("ODS: ", robot.ODS.getLightDetected());

                        }
                    }
                }
                telemetry.update();

                break;

            case "alignTo1stBeacon":
                //Move Forward Until White Line Located
                if (firstAlignToBeacon) {
                    //Reset Loop Timer
                    newStartTime = getRuntime();

                    //Turn On RGB LED
                    //robot.RGBBOTTOMCENTER.enableLed(true);
                    robot.cdim.setDigitalChannelState(robot.RGB_LED_BOTTOMSIDE, true);

                    firstAlignToBeacon = false;
                }
                else
                {
                    //Determine encoder ticks traveled
                    double deltaPOSDiff = initialPOS - robot.DrivePos1.getCurrentPosition();

                    if (IMUactive == true) {
                        try {

                            if (robot.IMU.getSystemStatus() == BNO055IMU.SystemStatus.RUNNING_FUSION) {
                                angles = robot.IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY);
                                lastHeading = angles.firstAngle;
                            }
                        } catch (Exception e) {
                            telemetry.addData("IMU Status", robot.IMU.getSystemStatus());
                            telemetry.addData("IMU Error", robot.IMU.getSystemError());
                            telemetry.addData("Error", e.toString());

                        }
                        telemetry.addData("Headings", startHeading + " " + lastHeading);
                        telemetry.addData("IMU Status", robot.IMU.getSystemStatus());

                    }

                    //Check for encoder ticks (as well as time limit)
                    if (loopTime <= 4 && deltaPOSDiff <= maxWhiteLineDistance){
                        telemetry.addData("POS: ", robot.DrivePos1.getCurrentPosition());
                        telemetry.addData("AlphaValue: ", robot.RGBBOTTOMSIDE.alpha());

                        mecanum( 0.15, 0, correction(startHeading,lastHeading));

                        if (robot.RGBBOTTOMSIDE.alpha() >= whiteLineAlphaThreshold) {
                            //Stop robot!
                            drive(0);

                            //Turn Off LED to conserve battery
                            robot.cdim.setDigitalChannelState(robot.RGB_LED_BOTTOMSIDE, true);

                            //We are (hopefully) aligned -- go to beacon press task
                            firstBeaconPress = true;
                            firstAlignToBeacon = true; //RESET FOR SECOND BEACON
                            currMainTask = "beaconpress";
                        }

                    }  else {

                        //We overshot OR RGB not picking up
                        telemetry.addData("Error: ", "Could not find line.");
                        drive(0);
                    }

                }
                telemetry.update();

                break;

            case "alignTo2ndBeacon":

                //Move Forward Until White Line Located
                if (firstAlignToBeacon) {
                    //Reset Loop Timer
                    newStartTime = getRuntime();

                    //Turn On RGB LED
                    robot.RGBBOTTOMCENTER.enableLed(true);
                    robot.cdim.setDigitalChannelState(robot.RGB_LED_BOTTOMCENTER, true);

                    firstBeaconPress = false;
                }
                else
                {
                    //Determine encoder ticks traveled
                    double deltaPOSDiff = initialPOS - robot.DrivePos1.getCurrentPosition();

                    //Check for encoder ticks (as well as time limit)
                    if (loopTime <= 4 && deltaPOSDiff <= maxWhiteLineDistance){
                        telemetry.addData("POS: ", robot.DrivePos1.getCurrentPosition());
                        telemetry.addData("AlphaValue: ", robot.RGBBOTTOMCENTER.alpha());

                        if (robot.RGBBOTTOMCENTER.alpha() >= whiteLineAlphaThreshold) {

                            //Turn Off LED to conserve battery
                            robot.cdim.setDigitalChannelState(robot.RGB_LED_BOTTOMCENTER, true);

                            //We are (hopefully) aligned -- got to beacon press task
                            firstBeaconPress = false;
                            currMainTask = "beaconpress";
                        }

                    }  else {


                    }

                }
                telemetry.update();
                break;

            case "beaconPress":

                if (firstTimeInBeaconPress) {
                    //Reset Loop Timer
                    newStartTime = getRuntime();

                    origPos = robot.DrivePos1.getCurrentPosition();

                    firstTimeInBeaconPress = false;
                    foundBeacon = false;
                } else {
                    if (IMUactive == true) {
                        try {

                            if (robot.IMU.getSystemStatus() == BNO055IMU.SystemStatus.RUNNING_FUSION) {
                                angles = robot.IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY);
                                lastHeading = angles.firstAngle;
                            }
                        } catch (Exception e) {
                            telemetry.addData("IMU Status", robot.IMU.getSystemStatus());
                            telemetry.addData("IMU Error", robot.IMU.getSystemError());
                            telemetry.addData("Error", e.toString());

                        }
                        telemetry.addData("Headings", startHeading + " " + lastHeading);
                        telemetry.addData("IMU Status", robot.IMU.getSystemStatus());

                    }
                    telemetry.addData("RGB Beacon Red: ", robot.RGBBEACON.red() );

                    if (loopTime <= 1.5 && !robot.cdim.getDigitalChannelState(robot.BEACONLIMITSWITCH) && !foundBeacon){
                        //Crab RIGHT with Heading adjust
                        mecanum( 0.15, 90, (lastHeading)*.1);
                    }
                    else {
                        if (!foundBeacon) {
                            if (robot.RGBBEACON.red() >= RGBBeaconREDThreshold) {
                                robot.BeaconPusher.setPosition(linActuOut);
                                beaconRed = true;
                                foundBeacon = true;
                            } else {
                                beaconRed = false;
                                foundBeacon = true;
                            }
                        } else
                        {
                            if (beaconRed && (loopTime >= .5) && (loopTime <= 1)) {
                                robot.BeaconPusher.setPosition(linActuIn);
                            }
                            if (beaconRed && (loopTime > 1) && (loopTime <= 1.5)){
                                robot.BeaconPusher.setPosition(linActuOut);
                            }
                            if (beaconRed && (loopTime > 1.5)){
                                robot.BeaconPusher.setPosition(linActuIn);
                                if (firstBeaconPress) {
                                    firstBeaconPress = false;
                                    currMainTask = "alignTo2ndBeacon";
                                } else {
                                    currMainTask = "driveToBall";
                                }
                            }
                            if (!beaconRed) {
                                if (origPos >= origPos + nextBeaconButtonTicks) {
                                    drive(0.0);

                                    if ((loopTime >= .5) && (loopTime <= 1)) {
                                        robot.BeaconPusher.setPosition(linActuIn);
                                    }
                                    if ((loopTime > 1) && (loopTime <= 1.5)){
                                        robot.BeaconPusher.setPosition(linActuOut);
                                    }
                                    if ((loopTime > 1.5)){
                                        robot.BeaconPusher.setPosition(linActuIn);
                                        if (firstBeaconPress) {
                                            firstBeaconPress = false;
                                            currMainTask = "alignTo2ndBeacon";
                                        } else {
                                            currMainTask = "driveToBall";
                                        }
                                    }

                                } else {
                                    drive(0.2);
                                    //Reset Loop Timer
                                    newStartTime = getRuntime();
                                }
                            }
                        }
                    }


                }
                break;

            case "drivetoball":

                break;

        }

    }

    private void drive(double power){
        robot.DrivePos1.setPower(power);
        robot.DrivePos2.setPower(power);
        robot.DrivePos3.setPower(power);
        robot.DrivePos4.setPower(power);
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

    private void launch(double speed){
        robot.BallLaunch1.setPower(speed);
        robot.BallLaunch2.setPower(speed);
    }

    private double accelerationLimiter(double deltaTime, double power){
        return Range.clip(deltaTime * (1 / (accelTimeMinPowerWheels - ((1 - power) * accelTimeMinPowerWheels))),0,power);
    }

    private double decelerationLimiter(double currEnc, double endPos, double power){
        double deltaEncoder = Math.abs(endPos - currEnc);
        double newPower;
        if (deltaEncoder <= decelMinTicks - (power * decelMinTicks)){
            newPower = Range.clip(((decelMinTicks - deltaEncoder)/ decelMinTicks), decelMinCrawlPwr, power);
        }else{
            newPower = power;
        }
        return newPower;
    }

    private double correction(double oldHeading, double newHeading){
        double corrSpin = 0.0;
        if(oldHeading - newHeading > 180) {
            if (oldHeading > newHeading) {
                corrSpin = (newHeading + 360) - oldHeading;
            } else if (oldHeading < newHeading) {
                corrSpin = (oldHeading + 360) - newHeading;
            } else if (oldHeading == newHeading) {
                corrSpin = 0.0;
            }
        }
        return corrSpin;
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
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    //Grab the angle needed in radians
    private String formatRadians(double radians){
        return String.format(Locale.getDefault(), "%.3f", AngleUnit.RADIANS.normalize(radians));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.3f", AngleUnit.DEGREES.normalize(degrees));
    }

}
