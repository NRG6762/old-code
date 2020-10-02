/* Copyright (c) 2014 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class PinTest extends OpMode {

    Servo armLift;

    public PinTest() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void start()

    {
        //
        // Call the PushBotHardware (super/base class) start method.
        //
        super.start();

        //
        // Reset the motor encoders on the drive wheels.
        //

        armLift = hardwareMap.servo.get("pin_left");
        armLift.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void init() {

		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        if (gamepad1.dpad_up || gamepad1.dpad_down ) {
            if (gamepad1.dpad_up) {
                armLift.setDirection(Servo.Direction.FORWARD);
                armLift.setPosition(0.2);
            } else if (gamepad1.dpad_down) {
                armLift.setDirection(Servo.Direction.REVERSE);
                armLift.setPosition(0.2);
            } else {
                armLift.setPosition(0.0);
            }
        }


        // clip the right/left values so that the values never exceed +/- 1
        //right = Range.clip(right, -1, 1);
        //left = Range.clip(left, -1, 1);


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        //right = (float) scaleInput(right) * (float) PowerScale;
        //left = (float) scaleInput(left) * (float) PowerScale;

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()    */


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */


    @Override
    public void stop() {

    }
}