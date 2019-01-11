package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous(name = "AutoOpRobotDepot", group = "Auto Linear Opmode")
//@Disabled
public class AutoOpRobotDepot extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    private static final int CYCLE_MS = 50;     // period of each cycle
    private static final double MAX_POS = 1.0;     // Maximum rotational position
    private static final double MIN_POS = 0.0;     // Minimum rotational position

    private double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    private boolean rampUp = true;
    private int goldPos = 0;

    FTCBaseRobot baseRobot = new FTCBaseRobot();
    vuforiaNavRecognize vuforiaNR = new vuforiaNavRecognize();
    SensorColorLCD senseMineralColor = new SensorColorLCD();

    @Override
    public void runOpMode() {

        ColorSensor sensorColor;
        DistanceSensor sensorDistance;
        float[] hsvValues;

        baseRobot.init(hardwareMap);
        vuforiaNR.initVuforia(hardwareMap);
        senseMineralColor.init(hardwareMap);

        telemetry.addData(">", "Press Start.");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //Step1: Recognize Gold
            goldPos = vuforiaNR.findGold();
            telemetry.addData("Gold Position", goldPos);
            telemetry.update();

                    //Step2: Descend
                    //baseRobot.RobotDescendAuto();
                    //sleep(5000);
                    //baseRobot.StopRobot();

            //Step2: Descend
            while (Boolean.TRUE) {
                baseRobot.RobotDescendAuto();
                sleep(500);
                sensorDistance = senseMineralColor.getSensorDistance();
                if (sensorDistance.getDistance(DistanceUnit.CM) <= 7) break;
            }
            baseRobot.StopRobot();

            //Step2.1: Move the Robot to it's  Left to remove the latch from the hook
            baseRobot.omniRobot(-0.25, -0.25);
            sleep(1500);
            baseRobot.StopRobot();

            //Step3: Look for Gold one more time in case the last attempt failed.
            if (goldPos == 0) {
                vuforiaNR.setGoldFound(false);
                goldPos = vuforiaNR.findGold();
                telemetry.addData("Gold Position Again", goldPos);
                telemetry.update();
            }
            //Step3.1: If the recognition fails again, assume it is in middle. This way we can navigate into depot easily
            if (goldPos == 0) {
                goldPos = 2;
                telemetry.addData("Gold Position Default", goldPos);
                telemetry.update();
            }

            //If I am the Robot , goldPos = 1 is to my left, 3 to my right and 2 in center

            //Step4: Go towards Gold and Knock off Gold

            //Step4.1 : MINERAL ON LEFT
            if (goldPos == 1) {

                //Step 4.1.1: Move the Robot LEFT a bit laterally to face the mineral at LEFT
                baseRobot.omniRobot(0.8, 0.0);
                sleep(500);
                baseRobot.StopRobot();

                //Step 4.1.2: Move the Robot forward to knock the mineral
                baseRobot.DriveAWDRobot(0.4, 0.4); //go straight
                sleep(2000);
                baseRobot.StopRobot();

                //Step 4.1.3: Reverse of Step 4.1.2
                baseRobot.DriveAWDRobot(-.4, -.4);
                sleep(2000);
                baseRobot.StopRobot();

                //Step 4.1.4: Reverse of Step 4.1.1
                baseRobot.omniRobot(0, .8);
                sleep(500);
                baseRobot.StopRobot();
            }

            //Step 4.2 : MINERAL IN CENTER
            if (goldPos == 2) {

                //Step 4.2.1: After landing and unlatching move forward a little bit so the Robot stays clear of the Latch
                baseRobot.DriveAWDRobot(0.4, 0.4);
                sleep(500);
                baseRobot.StopRobot();

                //Step 4.2.2: Move laterally to the Robot's right face the mineral at CENTER
                baseRobot.omniRobot(0.0, 0.8);
                sleep(500);
                baseRobot.StopRobot();

                //Step 4.2.3: Move the Robot forward to knock the mineral
                baseRobot.DriveAWDRobot(0.4, 0.4); //go straight
                sleep(2000);
                baseRobot.StopRobot();

                //Step 4.2.4: Reverse of Step 4.2.3
                baseRobot.DriveAWDRobot(-.4, -.4);
                sleep(2000);
                baseRobot.StopRobot();

            }

            //Step4.3 : MINERAL ON RIGHT
            if (goldPos == 3) {

                //Step 4.3.1: After landing and unlatching move forward a little bit so the Robot stays clear of the Latch
                baseRobot.DriveAWDRobot(0.4, 0.4);
                sleep(500);
                baseRobot.StopRobot();

                //Step 4.3.2: Move laterally to the Robot's right face the mineral at RIGHT
                baseRobot.omniRobot(0.0, 0.8);
                sleep(1000);
                baseRobot.StopRobot();

                //Step 4.3.3: Move the Robot forward to knock the mineral
                baseRobot.DriveAWDRobot(0.4, 0.4); //go straight
                sleep(2000);
                baseRobot.StopRobot();

                //Step 4.3.4: Reverse of Step 4.3.3
                baseRobot.DriveAWDRobot(-.4, -.4);
                sleep(2000);
                baseRobot.StopRobot();

                //Step 4.3.5: Reverse of Step 4.3.4
                baseRobot.omniRobot(.8, 0);
                sleep(1000);
                baseRobot.StopRobot();

            }
/*
            //Step 5: Move to the Depot

            //Step 5.1: Move laterally all the way towards the wall to the RIGHT
            baseRobot.omniRobot(0.0, 0.8);
            sleep(8000);
            baseRobot.StopRobot();

            //Step 5.2: Move the Robot forward into the Depot
            baseRobot.DriveAWDRobot(0.4, 0.4);
            sleep(8000);
            baseRobot.StopRobot();

            //Step 6: Place the Team Marker
            baseRobot.spinServoAuto(FTCBaseRobot.CRServoPosition.FORWARD);
            baseRobot.StopRobot();

            //Step 7: Drive in reverse ALL the way to Crater
            baseRobot.DriveAWDRobot(-0.4, -0.4);
            sleep(10000);
            baseRobot.StopRobot();

*/

            stop();
            sleep(6000);
        }

        vuforiaNR.shutdown();
        telemetry.addData("Status", "Run Time: " + runtime.seconds());
        telemetry.update();
    }

}