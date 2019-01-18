package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

public class FTCBaseRobot {
    // Public OpMode member
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor plateTiltMotor;
    public DcMotor torqueLinearMotor;
    public DcMotor basketLiftMotor;
    public DcMotor leftTorqueLinearMotor;

    public CRServo sweeperSpinServo = null;

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

    // local OpMode members
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor
    public FTCBaseRobot() {

    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hwMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hwMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hwMap.get(DcMotor.class, "rightRearMotor");
        plateTiltMotor = hwMap.get(DcMotor.class, "plateTiltMotor");
        basketLiftMotor = hwMap.get(DcMotor.class, "basketLiftMotor");
        torqueLinearMotor = hwMap.get(DcMotor.class, "torqueLinearMotor");
        leftTorqueLinearMotor = hwMap.get(DcMotor.class, "leftTorqueLinearMotor");

        // Set all motors to run without encoders, may want to use RUN_USING_ENCODERS if encoders are installed.
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateTiltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        basketLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        torqueLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftTorqueLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set all motors to zero power
        leftRearMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        plateTiltMotor.setPower(0);
        basketLiftMotor.setPower(0);
        torqueLinearMotor.setPower(0);
        leftTorqueLinearMotor.setPower(0);

        // Define and initialize ALL installed servos
         sweeperSpinServo = hwMap.get(CRServo.class, "sweeperSpinServo");
         sweeperSpinServo.setPower(CONTINUOUS_SERVO_STOP);
    }

//*************************************************************************************************
//          General
//*************************************************************************************************


    public void DriveAWDRobot(double leftPower, double rightPower) {
        //Move the robot

        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightRearMotor.setPower(rightPower);

        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightFrontMotor.setPower(rightPower);

        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRearMotor.setPower(leftPower);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftFrontMotor.setPower(leftPower);
    }

    public void omniRobot(double mRP, double mLP) {
        //The following is a test of what direction the mecanum wheels will go. Name Accordingly by the direction the robot goes

        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRearMotor.setPower(mRP);

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightFrontMotor.setPower(-mRP);

        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRearMotor.setPower(-mRP);

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftFrontMotor.setPower(mRP);

        /***********************************************************************/
        //The following is the opposite of the code above

        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRearMotor.setPower(-mLP);

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightFrontMotor.setPower(mLP);

        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftRearMotor.setPower(mLP);

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftFrontMotor.setPower(-mLP);
    }

    public void StopRobot() {
        // Set all motors to zero power (reference not provided)

        basketLiftMotor.setPower(0);
        plateTiltMotor.setPower(0);
        torqueLinearMotor.setPower(0);
        leftTorqueLinearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
    }

    public void StopRobot(DcMotor motor) {

        motor.setPower(0);
    }

    public void liftArm(double armPowerUp)
    {
        basketLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        basketLiftMotor.setPower(armPowerUp);
    }

    public void dropArm(double armPowerDown)
    {
        basketLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        basketLiftMotor.setPower(armPowerDown);
    }

//*************************************************************************************************
//          Autonomous
//*************************************************************************************************

    public void spinServoAuto(CRServoPosition servoPos){
        if (servoPos == CRServoPosition.STOP) {
            sweeperSpinServo.setPower(CONTINUOUS_SERVO_STOP);
        } else if (servoPos == CRServoPosition.FORWARD) {
            sweeperSpinServo.setPower(CONTINUOUS_SERVO_FORWARD);
        } else if (servoPos == CRServoPosition.REVERSE) {
            sweeperSpinServo.setPower(CONTINUOUS_SERVO_REVERSE);
        }
    }
    public void tiltBasketAuto(double tiltPower)
    {
        plateTiltMotor.setDirection(DcMotor.Direction.FORWARD);
        plateTiltMotor.setPower(tiltPower);
    }
    public void RobotAscendAuto() {
        //Make the robot go UP! Used in Lift
        torqueLinearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftTorqueLinearMotor.setDirection(DcMotor.Direction.REVERSE);
        torqueLinearMotor.setPower(0.35);
        leftTorqueLinearMotor.setPower(0.35);
    }

    public void RobotDescendAuto() {
        //Make the robot go DOWN! Used in Land
        torqueLinearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftTorqueLinearMotor.setDirection(DcMotor.Direction.FORWARD);
        torqueLinearMotor.setPower(0.7);
        leftTorqueLinearMotor.setPower(0.7);
    }

//*************************************************************************************************
//          TeleOp
//*************************************************************************************************

    public void tiltBasketTeleUp()
    {
        plateTiltMotor.setDirection(DcMotor.Direction.FORWARD);
        plateTiltMotor.setPower(.4);
    }
    public void tiltBasketTeleDown()
    {
        plateTiltMotor.setDirection(DcMotor.Direction.FORWARD);
        plateTiltMotor.setPower(-.4);
    }
    public void RobotAscendTele(double raisePower)
    {
        torqueLinearMotor.setDirection(DcMotor.Direction.FORWARD);
        torqueLinearMotor.setPower(raisePower);
        leftTorqueLinearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftTorqueLinearMotor.setPower(raisePower);
    }
    public void RobotDescendTele(double lowerPower)
    {
        torqueLinearMotor.setDirection(DcMotor.Direction.REVERSE);
        torqueLinearMotor.setPower(lowerPower);
        leftTorqueLinearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftTorqueLinearMotor.setPower(lowerPower);
    }

}
