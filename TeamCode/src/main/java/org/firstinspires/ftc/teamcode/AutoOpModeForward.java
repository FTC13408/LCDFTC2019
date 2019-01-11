package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import junit.framework.Test;


@Autonomous(name="AutoOpMode_Forward", group="Linear Movements")
@Disabled

public class AutoOpModeForward extends LinearOpMode {

    FTCBaseRobot baseRobot = new FTCBaseRobot();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    private static final int    CYCLE_MS    =   50;     // period of each cycle
    private static final double MAX_POS     =  1.0;     // Maximum rotational position
    private static final double MIN_POS     =  0.0;     // Minimum rotational position

    public DcMotor  leftMotor  = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  armMotor = null;
    public DcMotor  sweeperMotor     = null;
    public Servo armServo = null;

    private double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    private boolean rampUp = true;

    @Override
    public void runOpMode() {

        baseRobot.init(hardwareMap);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();
        runtime.reset();


        // Run code till stop pressed.
        while(opModeIsActive()){
            baseRobot.DriveAWDRobot(.25, .25 );
            sleep(1000);                                      //forward
            baseRobot.StopRobot();
            stop();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}