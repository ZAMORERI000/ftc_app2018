/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Mode", group="2018")
public class program extends LinearOpMode {

    // def motors
    private DcMotor leftDrive, rightDrive, spinDrive, armDrive = null;
    private Servo kickServo = null;

    // keeps track of kickArm servo state
    private boolean kickerOut = false;
    private long lastServoMove = 0;

    @Override
    public void runOpMode() {
        // print debug
        telemetry.addData("Debug", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");       // left motor for driving
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");      // right motor for driving
        armDrive = hardwareMap.get(DcMotor.class, "motorArm");          // vertical motor for throwing balls
        spinDrive = hardwareMap.get(DcMotor.class, "motorSpin");        // motor for collecting balls
        kickServo = hardwareMap.get(Servo.class, "servoKick");          // servo for kicking trophy

        // set motor dirs
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.FORWARD);
        spinDrive.setDirection(DcMotor.Direction.REVERSE);

        // move servo back initiailly
        kickServo.setPosition(0.0);

        // wait for start
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // handle driving mechanics
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn * 2, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn * 2, -1.0, 1.0);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Debug", "Left: " + leftPower + " | Right: " + rightPower);

            // set vertical arm power
            armDrive.setPower(gamepad2.right_stick_y * 0.2);

            // make spinDrive spin with speed proportional to trigger displacement
            spinDrive.setPower(gamepad2.right_trigger);
            spinDrive.setPower(-gamepad2.left_trigger);

            // toggle servo for trophy kick
            if (gamepad2.left_bumper && System.currentTimeMillis() - lastServoMove > 500) {
                kickServo.setPosition(kickerOut? 0.0 : 1.0);
                kickerOut = !kickerOut;
                lastServoMove = System.currentTimeMillis();
            }
            telemetry.addData("Debug", "Kicker Arm Toggled to " + kickerOut);
            telemetry.update();
        }
    }
}
