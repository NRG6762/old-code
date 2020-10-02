package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="CRServo Test", group="Test")
//@Disabled
public class CRServoTest extends LinearOpMode{

    // Initialize servos
    public CRServo svtest;

    HardwareMap hardwareMap;

    // Call motors and autonomous actions
    @Override
    public void runOpMode() {

        // Retrieve drive DC motors and set directions
       svtest = hardwareMap.crservo.get("stest");

       waitForStart();

       //svtest.setPower(1.0);

    }

}
