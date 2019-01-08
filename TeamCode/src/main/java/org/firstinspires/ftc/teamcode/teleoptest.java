
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOpMode_Test", group="Linear Opmode")
//@Disabled
public class teleoptest extends LinearOpMode {

    // Declare OpMode members.
    FTCBaseRobot baseRobot = new FTCBaseRobot();
    private ElapsedTime runtime = new ElapsedTime();

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower =0;
    double rightPower =0;
    double mRP = 0;
    double mLP = 0;
    double armPowerUp;
    double armPowerDown;
    double raisePower;
    double lowerPower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        baseRobot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
//*************************************************************************************************
//          GAMEPAD 1
//          1. Left and Right Trigger: Gives power to the drive motors
//          2. Dpad left and right buttons: Will turn the robot left or right
//          3. Right joy stick up: Ascend the robot on the pod
//          4. Right joy stick down: Descends the robot grom the pod
//          5. Right joy stick press: Will stop the latch motor
//          6. Left joy stick: NOT USED
//          7. X,Y,A,B: NOT USED
//*************************************************************************************************

            //Regular Wheel Robot Control
            rightPower = -0.8 * gamepad1.right_stick_y;
            leftPower = -0.8 * gamepad1.left_stick_y;
            baseRobot.DriveAWDRobot(leftPower, rightPower);

            //Mecanum Drive Test. Test what side robot glides and adjust accordingly.
            mRP = gamepad1.right_trigger;
            mLP = gamepad1.left_trigger;
            baseRobot.omniRobot(mRP, mLP);

            //Stop all motors and servos
            if(gamepad1.right_bumper)
            {
                baseRobot.StopRobot();
                baseRobot.spinServoAuto(FTCBaseRobot.CRServoPosition.STOP);
            }

            //Sweeper Basket Up
            if(gamepad2.y)
            {
                baseRobot.tiltBasketTeleUp();
                sleep(200);
                baseRobot.StopRobot();
            }
            //Sweeper Basket Down
            if(gamepad2.a)
            {
                baseRobot.tiltBasketTeleDown();
                sleep(200);
                baseRobot.StopRobot();
            }
            //Sweep in
            if(gamepad2.x)
            {
                baseRobot.spinServoAuto(FTCBaseRobot.CRServoPosition.FORWARD);
                sleep(500);
                baseRobot.spinServoAuto(FTCBaseRobot.CRServoPosition.STOP);
            }
            //Sweep out
            if(gamepad2.b)
            {
                baseRobot.spinServoAuto(FTCBaseRobot.CRServoPosition.REVERSE);
                sleep(50);
                baseRobot.spinServoAuto(FTCBaseRobot.CRServoPosition.STOP);
            }

            //Make the linear arm go down, hence raising the robot
            raisePower = gamepad2.right_stick_y;
            baseRobot.RobotAscendTele(raisePower);

            //Make the linear arm go up, hence lowering the robot
            lowerPower = gamepad2.left_stick_y;
            baseRobot.RobotDescendTele(lowerPower);

            //Basket Up
            armPowerUp = 0.8 * gamepad2.left_trigger;
            baseRobot.liftArm(armPowerUp);

            //Basket Down
            armPowerDown = 0.8 * gamepad2.right_trigger;
            baseRobot.dropArm(armPowerDown);

            //Stop All Motors and servos
            if(gamepad2.right_bumper)
            {
                baseRobot.StopRobot();
                baseRobot.spinServoAuto(FTCBaseRobot.CRServoPosition.STOP);
            }

            //*************************************************************************************************
//          Update Driver Station with telemetry data
//*************************************************************************************************
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}