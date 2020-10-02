package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Objects;

/*
        _    __   _   __   _   _           ___                        _        ___   _
 |\ |  |_)  /__  |_    /  |_    )    __     |   ._    _|  o   _      /_\  |  |  |   / \
 | \|  | \  \_|  |_)  /   |_)  /_          _|_  | |  (_|  |  (/_     | |  \_/   |   \_/
 ---------------------------------------------------------------------------------------------
    Programmer: Aiden
                Nole Sniekus
                Chris Ferrotti
                
    School:     Northwest Regional 7

 */
@Autonomous(name="If Visible, See Aiden/F.R.P.", group="@IndieAutonomous")
@Disabled
public class IndieAutonomousBase extends LinearOpMode{

    //Declare OpMode members. Use the class created to define the robot's hardware
    public IndieHardware indie = new IndieHardware();
    private String liftMode = "Initial";
    private double lastlooptime = 0.00;
    private double MaxRotBlah = 500.0;
    private double MinDegreeRampDown = 30.0;
    public boolean liftInitComplete = false;
    public double scissorSpeed = .5;
    public double scissorRotationSpeed = 0.05;
    public double currentScissorGoal = .5;


    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

    }

    public void mecanum(double speed, double degrees, double spin){

        //Change joypad values into useful polar values
        double angle = (degrees * Math.PI)/ 180;
        double dchange = -spin / 2;

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
        indie.DrivePos1.setPower(pos1/maxValue);
        indie.DrivePos2.setPower(pos2/maxValue);
        indie.DrivePos3.setPower(pos3/maxValue);
        indie.DrivePos4.setPower(pos4/maxValue);
    }

    public void stopMotors(){
        mecanum(0,0,0);
    }

    public void resetEncoders(){
        indie.DrivePos1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indie.DrivePos2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indie.DrivePos3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indie.DrivePos4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indie.DrivePos1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indie.DrivePos2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indie.DrivePos3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indie.DrivePos4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public String VuMark(){

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(indie.relicTemplate);
        String VuMark = "Un-identified";

        if(vuMark==RelicRecoveryVuMark.LEFT){
            VuMark="LEFT";
        }else if(vuMark==RelicRecoveryVuMark.RIGHT){
            VuMark="RIGHT";
        }else if(vuMark==RelicRecoveryVuMark.CENTER){
            VuMark="CENTER";
        }else if(vuMark==RelicRecoveryVuMark.UNKNOWN) {
            VuMark = "Un-identified";
        }

        return VuMark;

    }

    public void resetLiftEncoder(){
        indie.BlockLift.setPower(0);
        indie.BlockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean liftInit(double loopTime) {

        boolean liftInitBool = false;

        switch (liftMode) {
            case "Initial":
                if ((loopTime <= 1.5) && (indie.BlockLift.getCurrentPosition() > -500)) {
                    indie.BlockLift.setPower(-.8);
                    indie.GripperLeft.setPosition(1);
                    indie.GripperRight.setPosition(1);
                } else {
                    liftInitBool = false;
                    liftMode = "ZeroLift";
                }
                break;

            case "ZeroLift":
                if ((loopTime <= 10) && indie.BallLiftLow.getState()) {
                    indie.BlockLift.setPower(.5);
                } else {
                    indie.BlockLift.setPower(0);
                    indie.BlockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    indie.BlockLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftInitBool = false;
                    liftMode = "ClampGlyph";
                }
                break;

            case "ClampGlyph":
                indie.GripperLeft.setPosition(0);
                indie.GripperRight.setPosition(0);

                if (loopTime >= 2.5) {
                    liftInitBool = false;
                    lastlooptime = loopTime;
                    liftMode = "RaiseBlock";
                }
                break;

            case "RaiseBlock":
                indie.BlockLift.setPower(-.8);

                if ((indie.BlockLift.getCurrentPosition() <= -550) || (loopTime - lastlooptime >= .5)) {
                    indie.BlockLift.setPower(0);
                    liftInitComplete = true;
                    liftInitBool = true;
                }
                break;
        }

        return liftInitBool;
    }

    public double imuHeading() {
        double heading;
        try{
            //indie.Imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            Orientation angles = indie.Imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY);
            heading = indie.Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        }catch (Exception e){
            telemetry.addData("Init", "IMU blew up, disabled: " + e.toString());
            telemetry.update();
            heading = 0.0;
        }

        return heading;
    }

    public double correction(double oldHeading, double newHeading){
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
        return  corrSpin;
    }

    public double targetHeading(double startHeading, double headingDifference) {
        double difference = 0.0;
        difference = startHeading + headingDifference;
        if (difference > 180) {
            difference = difference - 360;
        } else if (difference < -180) {
            difference = difference + 360;
        }
        return difference;
    }

    public double runme(double CurrHeading, double DestHeading, double Power, double looptime) {
        double diff = CurrHeading - DestHeading;
        if(Math.abs(diff) > 180) {
            diff = DestHeading - CurrHeading;
        }
        double pow;
        double milliLoopTime = looptime * 1000;
        if(diff >= 0 && diff > MinDegreeRampDown) {
            pow = calc(milliLoopTime,-1);
            /*if(Math.abs(pow) > Power) {
                pow = -1 * Power;
            }*/
        }
        else if(diff <=0 && Math.abs(diff) > MinDegreeRampDown) {
            pow = calc(milliLoopTime,1);/*
            if( pow > Power ) {
                pow = Power;
            }*/
        }
        else {
            pow = RampDowncalc(diff);
            if( -.05 <= pow && pow <=0  ) {
                pow = -.09;
            }
            else if(0 <= pow && .05 >= pow) {
                pow = .09;
            }
        }
        return pow;

    }

    private double calc(double looptime,int Di) {
        double A = Di/Math.sqrt(MaxRotBlah);
        return A * Math.sqrt(looptime);
    }

    private double RampDowncalc(double Diff) {
        double H = MinDegreeRampDown;
        double A = -1 / Math.pow(H,3);
        return  Math.pow(Diff,3) * A;
    }

    public double diff (double value1, double value2) {
        double total = 0.0;
        if (value1 > value2) {
            total = ((value1 - value2));
        } else {
            total = ((value2 - value1));
        }
        return total;
    }

    public int[] rgbSensorValues(ColorSensor rgb){
        int[] values = new int[3];

        values[0] = rgb.red()*256;
        values[1] = rgb.green()*256;
        values[2] = rgb.blue()*256;

        return values;
    }

    public void rgbLedsChangeState(boolean stateChange){
        indie.LeftRGBLED.setState(stateChange);
        indie.RightRGBLED.setState(stateChange);
    }

    /*public int pixySeeRedBlob(){

        byte sign1[] = indie.Pixy.read(0x51,5);

        return 0xff & sign1[1];

    }*/

    public String jewelFinder(String side, double startTime, double loopTime){

        String currentState = "false";

        if(loopTime - startTime >= 4.5){
            currentState = "timeOut";
        }else{
            /*if (Objects.equals(side, "Left")) {
                indie.JewelLeft.setPosition(.8);

                //if (pixySeeRedBlob() == 0) {
                    rgbLedsChangeState(true);
                    if (rgbSensorValues(indie.LeftRGB)[2] > .4) {
                        currentState = "blue";
                    } else if (rgbSensorValues(indie.LeftRGB)[0] > .6) {
                        currentState = "red";
                    }
                /*} else {
                    currentState = "red";
                }*/

             if (Objects.equals(side, "Right")) {
                if(loopTime - startTime <= 3){
                    indie.JewelRight.setPosition(.7);
                    rgbLedsChangeState(true);
                    //Here Is The Error
                    if (rgbSensorValues(indie.LeftRGB)[2] > 102) {
                        currentState = "blue";
                    } else if (rgbSensorValues(indie.LeftRGB)[0] > 153) {
                        currentState = "red";
                    } else {
                        currentState = "false";
                    }
                }else{
                    currentState = "blue";
                }
             }
        }

        if(!Objects.equals(currentState, "false")){
            startTime = loopTime;
            rgbLedsChangeState(false);
        }

        telemetry.addData("RGB Value Red: ", rgbSensorValues(indie.RightRGB)[0]*256);
        telemetry.addData("RGB Value Green: ", rgbSensorValues(indie.RightRGB)[1]*256);
        telemetry.addData("RGB Value Blue: ", rgbSensorValues(indie.RightRGB)[2]*256);
        telemetry.addData("Jewel Color Seen: ", currentState);
        telemetry.addData("Jewel Time: ",loopTime-startTime);

        return currentState;

    }

    /*public String jewelPositionFinder(){

        String position = "false";

        if(pixySeeRedBlob() > 85){
            position = "right";
        }else{
            position = "left";
        }

        if(!Objects.equals(position, "false")){
            indie.Pixy.disengage();
        }

        return position;
    }*/

    /*public boolean scissorFinder(String autonomousMode){

        boolean currentState = false;

        double goalPosition = currentScissorGoal;

        if(indie.SCIWall.getState()){
            currentState = true;
        }else{
            if(!indie.SCIBackLeft.getState() && !indie.SCIBackRight.getState()){
                indie.ScissorArmPwr.setPower(scissorSpeed);
            }else{
                if(autonomousMode.equals("IndieAutonomousRedA")){
                    goalPosition = currentScissorGoal + scissorRotationSpeed;
                }else if(autonomousMode.equals("IndieAutonomousRedB")){
                    goalPosition = currentScissorGoal + scissorRotationSpeed;
                }else if(autonomousMode.equals("IndieAutonomousBlueA")){
                    goalPosition = currentScissorGoal - scissorRotationSpeed;
                }else {
                    goalPosition = currentScissorGoal - scissorRotationSpeed;
                }
            }
            indie.ScissorRotate.setPosition(goalPosition);
        }

        return currentState;

    }*/

    /*public boolean gripAndUp(boolean start, boolean grip, boolean up, boolean in, double currentTime, double gripStartTime){

        boolean done = false;

        if(start){
            gripStartTime = currentTime;
            start = false;
        }else if(grip){
            indie.RelicGripper.setPosition(1);
            if(currentTime-gripStartTime >= 1){
                grip = false;
                up = true;
            }
        }else if(up){
            indie.ScissorArmLiftPwr.setPower(.5);
            if(Math.abs(indie.ScissorArmLiftPwr.getCurrentPosition()) == 200){
                indie.ScissorArmLiftPwr.setPower(0);
                up = false;
                in = true;
            }
        }else if(in){
            indie.ScissorArmPwr.setPower(-scissorSpeed);
            if(indie.LIFTInAll.getState()){
                indie.ScissorArmPwr.setPower(0);
                in = false;
            }
        }else{
            done = true;
        }

        return done;

    }

    public double jewelArmRamping(double jewelGoalPosition, double startTime, double loopTime){

        double newGoalPosition = currentPosition;

        if(currentPosition - jewelGoalPosition < 0) {
            if (jewelGoalPosition - currentPosition > .3) {
                newGoalPosition = currentPosition - .5;
            } else if (jewelGoalPosition - currentPosition > .15) {
                newGoalPosition = currentPosition - .05;
            } else if (jewelGoalPosition - currentPosition > .05) {
                newGoalPosition = currentPosition - .01;
            }
        }else if(currentPosition - jewelGoalPosition > 0){
            if (currentPosition - jewelGoalPosition > .3) {
                newGoalPosition = currentPosition - .5;
            } else if (currentPosition - jewelGoalPosition > .15) {
                newGoalPosition = currentPosition - .05;
            } else if (currentPosition - jewelGoalPosition > .05) {
                newGoalPosition = currentPosition - .01;
            }
        }else{
            newGoalPosition = jewelGoalPosition;
        }

        return newGoalPosition;

    }*/

    public boolean turnAndBack(String jewelSide, double loopTime, double startTime){

        boolean done = false;

        if(jewelSide == "back") {
            if (loopTime - startTime <= 3) {
                indie.RelicPivot.setPosition(0);
            } else if ((loopTime - startTime <= 6) && (loopTime - startTime > 3)) {
                indie.RelicPivot.setPosition(1);
            } else {
                done = true;
            }
        }else{
            if (loopTime - startTime <= 3) {
                indie.RelicPivot.setPosition(1);
            } else if ((loopTime - startTime <= 6) && (loopTime - startTime > 3)) {
                indie.RelicPivot.setPosition(0);
            } else {
                done = true;
            }
        }

        return done;
    }
}