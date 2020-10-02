package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Objects;

@Autonomous(name="Indie: Autonomous Blue A", group="#No Jewel")
public class IndieAutonomousBlueA extends IndieAutonomousBase {

    //telemetry switches
    private boolean wheelPositionTele   = true;
    private boolean liftPositionTele    = true;
    private boolean vuforiaTele         = true;
    private boolean imuHeadingTele      = true;

    private String currMainTask = "Step 1";
    //Initialize as Un-Identified for fall through.
    private String VuMark = "Un-identified";

    private double startTime;
    private double loopTime;
    private double step4Start;
    private double prevStartPos;

    private double startHeading = 0.0;
    private double currentHeading = 0.0;

    private double scissorTime = 0.0;

    boolean vuforiaDone = false;
    boolean step2Reset = true;
    boolean step4Time = true;
    boolean pixyActivate = true;
    boolean jewelActivate = false;
    boolean scissorActivate = false;
    boolean turn = true;
    boolean back = false;
    boolean grip = false;
    boolean up = false;
    boolean in = false;
    boolean start = true;

    @Override
    public void runOpMode() {

        //Initialize the hardware variables. The init() method of the hardware class does all the work here
        telemetry.addData("Init", "Initializing Hardware Map");
        telemetry.update();
        indie.init(hardwareMap);
        telemetry.addData("Init", "Hardware Map Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        startTime = getRuntime();
        indie.Imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        indie.relicTrackables.activate();
        startHeading = imuHeading();
        telemetry.addData("Current Step: ", currMainTask);

        while (opModeIsActive()) {

            loopTime = getRuntime() - startTime;
            currentHeading = imuHeading();
            telemetry.addData("Current Step: ", currMainTask);

            switch (currMainTask){

                //Initialize(Expand) Lift Mechanism
                case "Step 1":

                    if (liftInit(loopTime)){
                        currMainTask = "Step 2";
                    }

                    if(!vuforiaDone){
                        VuMark = VuMark();
                        if (!VuMark.equals("Un-identified")) {
                            indie.relicTrackables.deactivate();
                            vuforiaDone = true;
                        }
                    }
                    break;
                case "Step 2":

                    /*if(pixyActivate){
                        indie.Pixy.engage();
                        jewelActivate = true;
                        pixyActivate = false;
                    }

                    if(jewelActivate) {
                        if (Objects.equals(jewelFinder("Right", ), "blue")){
                            if(jewelPositionFinder() == "right"){
                                //turn left and back
                                if(turnAndBack(-15,turn,back,currentHeading,startHeading)){
                                    jewelActivate = false;
                                    scissorActivate = true;
                                }
                            }else if(jewelPositionFinder() == "left"){
                                //turn right and back
                                if(turnAndBack(15,turn,back,currentHeading,startHeading)){
                                    jewelActivate = false;
                                    scissorActivate = true;
                                }
                            }
                        }else if (Objects.equals(jewelFinder("Right"), "red")) {
                            if(jewelPositionFinder() == "right"){
                                //turn right and back
                                if(turnAndBack(15,turn,back,currentHeading,startHeading)){
                                    jewelActivate = false;
                                    scissorActivate = true;
                                }
                            }else if(jewelPositionFinder() == "left"){
                                //turn left and back
                                if(turnAndBack(-15,turn,back,currentHeading,startHeading)){
                                    jewelActivate = false;
                                    scissorActivate = true;
                                }
                            }
                        }
                    }

                    if(scissorActivate){
                        if(scissorFinder("IndieAutonomousRedA")){
                            if(gripAndUp(start,grip,up,in,loopTime,scissorTime)){
                                scissorActivate = false;
                                */currMainTask = "Step 3";/*
                            }
                        }
                    }*/

                    break;

                case "Step 3":


                    if(step2Reset) {
                        telemetry.addData("Encoders Reset: ", step2Reset);

                        //If vuforia is not done reading the vumarks, set the vumark as unidentified
                        if(!vuforiaDone){
                            VuMark = VuMark();
                            if(VuMark.equals("Un-identified")) {
                                    VuMark = "LEFT";
                                    vuforiaDone = true;
                                    indie.relicTrackables.deactivate();

                            }
                            else {
                                vuforiaDone = true;
                                VuMark = "LEFT";
                                indie.relicTrackables.deactivate();
                            }
                        }

                        resetEncoders();
                        step2Reset = false;
                    }

                    if(VuMark.equals("LEFT") || VuMark.equals("Un-identified") ) {
                        if(indie.DrivePos1.getCurrentPosition() <= 2210) {
                            mecanum(.5, 0, -correction(startHeading, currentHeading));
                        }else{
                            stopMotors();
                            resetEncoders();
                            currMainTask = "Step 4";
                        }
                    }else if(VuMark.equals("CENTER")){
                        if(indie.DrivePos1.getCurrentPosition()<= 2830) {
                            mecanum(.5, 0 ,  -correction(startHeading,currentHeading));
                        }else{
                            stopMotors();
                            resetEncoders();
                            currMainTask = "Step 4";}
                    }else if(VuMark.equals("RIGHT")){
                        if(indie.DrivePos1.getCurrentPosition()<= 3450) {
                            mecanum(.5, 0 , -correction(startHeading,currentHeading));
                        }else{
                            stopMotors();
                            resetEncoders();
                            currMainTask = "Step 4";
                            telemetry.addData("Current Step: ", currMainTask);}

                    }
                    break;

                case "Step 4":

                    if(step4Time) {
                        step4Start = loopTime;
                        step4Time = false;
                    }

                    telemetry.addData("Time: ", loopTime - step4Start);

                    if(diff(currentHeading, targetHeading (startHeading, 90))>1) {
                        //mecanum(0, 0, -1);
                        double Power = runme(currentHeading,targetHeading(startHeading,90),1,loopTime - step4Start);
                        telemetry.addData("Power: ", Power);
                        mecanum(0,0,Power);
                    }else{
                        stopMotors();
                        prevStartPos = indie.DrivePos1.getCurrentPosition();
                        currMainTask = "Step 5";
                    }
                    break;

                case "Step 5":
                    if ((indie.DrivePos1.getCurrentPosition()-prevStartPos) < 1000) {
                        mecanum(.4, 0, 0);
                    } else {
                        mecanum(0, 0, 0);
                        prevStartPos = indie.DrivePos1.getCurrentPosition();
                        indie.GripperLeft.setPosition(1);
                        indie.GripperRight.setPosition(1);
                        currMainTask = "Step 6";
                    }
                break;

                case "Step 6":
                    if((prevStartPos - indie.DrivePos1.getCurrentPosition()) < 1000) {
                        mecanum(-.4, 0, 0);
                    } else {
                        mecanum(0, 0, 0);
                        indie.GripperLeft.setPosition(.4);
                        indie.GripperRight.setPosition(.4);
                        prevStartPos = indie.DrivePos1.getCurrentPosition();
                        currMainTask = "Step 7";
                    }
                    break;

                case "Step 7":
                    if ((indie.DrivePos1.getCurrentPosition()-prevStartPos) < 1000) {
                        mecanum(.15, 0, 0);
                    } else {
                        mecanum(0, 0, 0);
                        resetEncoders();
                        prevStartPos = indie.DrivePos1.getCurrentPosition();
                        currMainTask = "Step 8";
                    }
                    break;

                case "Step 8":
                    if ((indie.DrivePos1.getCurrentPosition()-prevStartPos) > -500) {
                        mecanum(-.15, 0, 0);
                    } else {
                        mecanum(0, 0, 0);
                        currMainTask = "Step 8";
                    }
                    break;
            }

            if (vuforiaTele) {
                telemetry.addData("VuforiaResult: ", VuMark);
            }

            //CheckForWheelTelemetry
            if (wheelPositionTele) {
                telemetry.addData("Wheel Position1: ", indie.DrivePos1.getCurrentPosition());
                telemetry.addData("Wheel Position2: ", indie.DrivePos2.getCurrentPosition());
                telemetry.addData("Wheel Position3: ", indie.DrivePos3.getCurrentPosition());
                telemetry.addData("Wheel Position4: ", indie.DrivePos4.getCurrentPosition());
            }

            //CheckForLiftTelemetry
            if (liftPositionTele)  {
                telemetry.addData("Lift Position: ", indie.BlockLift.getCurrentPosition());
                telemetry.addData("Lift Bottom Limit: ", indie.BallLiftLow.getState());
            }

            if (imuHeadingTele) {
                telemetry.addData("IMU Heading: ", currentHeading);
                telemetry.addData("IMU Start Heading: ", startHeading);
                telemetry.addData("Target Heading: ", targetHeading (startHeading, -90));
            }

            telemetry.addData("LoopTime: ", loopTime);

            //Update the Telemetry
            telemetry.update();
        }
    }
}
