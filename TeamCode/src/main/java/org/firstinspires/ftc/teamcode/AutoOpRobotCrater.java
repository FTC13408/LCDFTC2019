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

@Autonomous(name = "AutoOpRobotCrater", group = "Auto Linear Opmode")
//@Disabled
public class AutoOpRobotCrater extends LinearOpMode {

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
            baseRobot.RobotDescendAuto();
            sleep(6000);
            baseRobot.StopRobot();

            baseRobot.omniRobot(0, .5);
            sleep(100);
            baseRobot.StopRobot();

            baseRobot.DriveAWDRobot(.3, .3);
            sleep(500);
            baseRobot.StopRobot();

            baseRobot.omniRobot(0.5,0);
            sleep(100);
            baseRobot.StopRobot();

			//Step3: Look for Gold one more time in case the last attempt failed.
            // Below is looking using vuforia. Replace with Color Sensor Code, if it works
			if ( goldPos == 0 ) {
                vuforiaNR.setGoldFound(false);
                goldPos = vuforiaNR.findGold();
                telemetry.addData("Gold Position Again", goldPos);
                telemetry.update();
			}
            if ( goldPos == 0 ) {
			    goldPos = 3;
            }
            //Step4: Go towards Gold and Knock off Gold
			if ( goldPos == 1 ) {
                baseRobot.omniRobot(0.0,0.8); //right turn
                sleep(100);
                baseRobot.StopRobot();

                baseRobot.DriveAWDRobot(0.4,0.4); //go straight
                sleep(2000);
                baseRobot.StopRobot();

                baseRobot.DriveAWDRobot(-.4, -.4);
                sleep(2000);
                baseRobot.StopRobot();

                baseRobot.omniRobot(.8, 0);
                sleep(100);
                baseRobot.StopRobot();

			} else if ( goldPos == 2 ) {
				baseRobot.DriveAWDRobot(0.4,0.4); //go straight
				sleep(2000);
				baseRobot.StopRobot();

				baseRobot.DriveAWDRobot(-.4, -.4);
				sleep(2000);
				baseRobot.StopRobot();

			} else if ( goldPos == 3 ) {
				baseRobot.omniRobot(0.8,0.0); //right turn
				sleep(500);
				baseRobot.StopRobot();

                baseRobot.DriveAWDRobot(0.4,0.4); //go straight
                sleep(2000);
                baseRobot.StopRobot();

                baseRobot.DriveAWDRobot(-.4, -.4);
                sleep(2000);
                baseRobot.StopRobot();

                baseRobot.omniRobot(0, .8);
                sleep(500);
                baseRobot.StopRobot();

			}  else {
			    //GOD HELP YE !
            }

            //Step5: See if there is a Crater ahead of you using color distance sensor
//            sensorColor = senseMineralColor.getSensorColor();
//            sensorDistance = senseMineralColor.getSensorDistance();
//            hsvValues = senseMineralColor.getHsvValues();
//
//            int alpha = sensorColor.alpha();
//            int red = sensorColor.red();
//            int green = sensorColor.green();
//            int blue = sensorColor.blue();
//            char craterDepot = 'N';
//
//            if ( red > 100 || blue > 100 ) {
//                craterDepot = 'D';
//            } else  if ( red < 60 && blue < 60 && green < 60) {
//                craterDepot = 'C';
//            }
//
//            //telemetry.addData("Distance (cm)",
//              //      String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//            //telemetry.addData("Alpha", alpha);
//            ///telemetry.addData("Red  ", red);
//            //telemetry.addData("Green", green);
//            //telemetry.addData("Blue ", blue);
//            telemetry.addData("Crater Depot ", craterDepot);
//            //telemetry.addData("Hue", hsvValues[0]);
//            telemetry.update();

                // UNCOMMENT AS WE PROGRESS
                /** vuforiaNR.findLocation();
                VectorF translation1 = vuforiaNR.getTranslation();
                Orientation rotation1 = vuforiaNR.getRotation();

                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation1.get(0) / mmPerInch, translation1.get(1) / mmPerInch, translation1.get(2) / mmPerInch);
                telemetry.addData("Navigation Found" , vuforiaNR.getTrackableName());
                telemetry.update();
                **/
            //Step 6: Drive towards Depot
            //Step 7: Place the Team Marker
            //Step 8: Drive into Crater

            stop();
            sleep(6000);
        }

        vuforiaNR.shutdown();
        telemetry.addData("Status", "Run Time: " + runtime.seconds());
        telemetry.update();
//        baseRobot.StopRobot(null);
    }

}
