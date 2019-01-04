package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class vuforiaNavRecognize {

    private static final String VUFORIA_KEY = "AZdLFbz/////AAABmahok7S+mEIqsnfiwGhimGdUHpZvl3tUj4HKGkQpj3S1d8TXXqrigtu0PveQ1oze1nXYaxEvbPqxloQUFNKWAbYGcMmDp7tr0ohqpNw43LDU37wuM3VG5PPKHKNnUgH7DdR4h4TK/PJYpl5Cas4hWRtyADEFaK/S0aYb5VE1sWZrl/WCnIctxIjLAG3SzXhr9owyn+FLNRotT2jwLkAsLQ2XSzMzKCd5F5X2WtFG3CG4gnNElluuDbYLbNtsoDpSX6c8lXkPpTHaL6iVE8FpVrWEiaVwOowxTCRCc2loU3i8xq8qtcGkanEQVpcgXfqlcI5Y7V9oUEKmBhXA+4xrne70izJCq35HnRMIvJbdS/vy";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
    private final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    private final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private VectorF translation;
    private Orientation rotation;
    private String trackableName;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private boolean goldFound = false;
    private int goldPos = 0;

    public String getTrackableName() {
        return trackableName;
    }

    public void setGoldFound(boolean found) {
        goldFound = found;
    }

    public VectorF getTranslation() {
        return translation;
    }

    public Orientation getRotation() {
        return rotation;
    }

    public void initVuforia(HardwareMap hmap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        goldFound = false;
        goldPos = 0;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hmap);
        } else {
            System.out.println("Sorry! , This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /*  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /* Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    public void initTfod(HardwareMap ahwMap) {
        int tfodMonitorViewId = ahwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public int findGold() {
        ElapsedTime runtimegold = new ElapsedTime();
        runtimegold.reset();
        while (!goldFound && runtimegold.seconds() < 5) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        System.out.println(" START Recognition");
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                                System.out.println("Silver Recognition" + recognition);
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                                System.out.println("Silver2 Recognition" + recognition);
                            }
                        }
                        System.out.println(" END Recognition");
                        //Update the status on Driver Controller regarding the position of Gold
                        if (goldMineralX != -1 && silverMineral1X != -1 && Math.abs(goldMineralX - silverMineral1X) >= 4) {
                            if (goldMineralX < silverMineral1X) {
                                System.out.println("Gold Mineral Position Left");
                                goldFound = true;
                                goldPos = 1;

                            } else {
                                System.out.println("Gold Mineral Position Center");
                                goldFound = true;
                                goldPos = 2;
                            }
                        }
                        if (silverMineral1X != -1 && silverMineral2X != -1 && Math.abs(silverMineral2X - silverMineral1X) >= 4) {
                            System.out.println("Gold Mineral Position Right");
                            goldFound = true;
                            goldPos = 3;
                        }
                    }
                }
            }
        }
        //vuforia = null;
        return goldPos;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void findLocation() {
        while (!targetVisible) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    System.out.println("Visible Target" + trackable.getName());
                    trackableName = trackable.getName();
                    targetVisible = true;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
        }
        if (targetVisible) {
            translation = lastLocation.getTranslation();
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        } else {
            System.out.printf("Visible Target : none");
        }
    }
}
