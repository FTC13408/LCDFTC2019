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

import android.app.Activity;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class FTCBaseRobot {
    /* Public OpMode members. */
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor armMotor;
    public DcMotor latchMotor;

    public CRServo armServo = null;
    public Servo hookServo = null;

    private static final double CONTINUOUS_SERVO_STOP = 0.05;
    private static final double CONTINUOUS_SERVO_FORWARD = 1.0;
    private static final double CONTINUOUS_SERVO_REVERSE = 0.0;

    public enum CRServoPosition {
        STOP,
        FORWARD,
        REVERSE
    }

    private static final double HB_SERVO_OPEN = 1.0;
    private static final double HB_SERVO_CLOSE = 0.0;
    private static final double HB_SERVO_MID = 0.5;

    public enum HBServoPosition
    {
        OPEN,
        MID,
        CLOSE
    }

    /* local OpMode members. */
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public FTCBaseRobot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        latchMotor = hwMap.get(DcMotor.class, "latchMotor");

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
        latchMotor.setPower(0);

        // Define and initialize ALL installed servos.
        armServo = hwMap.get(CRServo.class, "armServo");
        armServo.setPower(CONTINUOUS_SERVO_STOP);

        hookServo = hwMap.get(Servo.class, "hookServo");
        //hookServo.setPosition(HB_SERVO_OPEN);

    }

//*************************************************************************************************
//          General
//*************************************************************************************************

    public void DriveRobot(double leftPower, double rightPower) {
        //Move the robot
        if (rightMotor != null && leftMotor != null ) {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            rightMotor.setPower(rightPower);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            leftMotor.setPower(leftPower);
        }
    }

    public void StopRobot() {
        // Set all motors to zero power if not reference is provided
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
        latchMotor.setPower(0);
    }

    public void StopRobot(DcMotor motor) {
        motor.setPower(0);
    }

    public void ManageHook(HBServoPosition servoPosition)
    {
        if(servoPosition == HBServoPosition.OPEN)
        {
            hookServo.setPosition(HB_SERVO_OPEN);
        }
        else if(servoPosition == HBServoPosition.CLOSE)
        {
            hookServo.setPosition(HB_SERVO_CLOSE);
        }
        else if(servoPosition == HBServoPosition.MID)
        {
            hookServo.setPosition(HB_SERVO_MID);
        }
    }

//*************************************************************************************************
//          Autonomous
//*************************************************************************************************

    public void RobotAscend() {
        latchMotor.setDirection(DcMotor.Direction.FORWARD);
        latchMotor.setPower(0.35);
    }

    public void RobotDescend() {
        latchMotor.setDirection(DcMotor.Direction.REVERSE);
        latchMotor.setPower(0.6);
    }

//*************************************************************************************************
//          TeleOp
//*************************************************************************************************

    public void RobotAscend(double motorPower) {
        latchMotor.setDirection(DcMotor.Direction.REVERSE);
        latchMotor.setPower(motorPower);
    }

    public void RobotDescend(double motorPower) {
        latchMotor.setDirection(DcMotor.Direction.FORWARD);
        latchMotor.setPower(motorPower);
    }

    public void MoveBasket(double servoPos) {
        armServo.setPower(servoPos);
    }


    public void MoveBasket(CRServoPosition servoPos) {
        if (servoPos == CRServoPosition.STOP) {
            armServo.setPower(CONTINUOUS_SERVO_STOP);
        } else if (servoPos == CRServoPosition.FORWARD) {
            armServo.setPower(CONTINUOUS_SERVO_FORWARD);
        } else if (servoPos == CRServoPosition.REVERSE) {
            armServo.setPower(CONTINUOUS_SERVO_REVERSE);
        }
    }
}

