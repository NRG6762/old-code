package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@Autonomous(name = "Pixy", group = "Sensor")
public class PixyTest extends LinearOpMode {

    HardwareMap hardwareRobot = null;

    I2cDeviceSynch pixyCam = null;

    byte[] sign1 = null;
    byte[] sign2 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        pixyCam = hardwareRobot.i2cDeviceSynch.get("pixyCam");

        pixyCam.engage();

        waitForStart();

        while(opModeIsActive()) {

            sign1 = pixyCam.read(0x51,5);
            sign2 = pixyCam.read(0x52,5);

            telemetry.addData("X value of sign1", 0xff&sign1[1]);
            telemetry.addData("X value of sign2", 0xff&sign2[1]);

            telemetry.update();
        }
    }
}