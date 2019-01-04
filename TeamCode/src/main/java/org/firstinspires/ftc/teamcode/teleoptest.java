
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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        baseRobot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
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
            rightPower = 0.8*gamepad1.right_stick_y;
            leftPower = 0.8*gamepad1.left_stick_y;
            baseRobot.DriveAWDRobot(leftPower, rightPower);

            //Mecanum Drive Test. Test what side robot glides and adjust accordingly.
            mRP = gamepad1.right_trigger;
            mLP = gamepad1.left_trigger;
            baseRobot.omniRobot(mRP, mLP);












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