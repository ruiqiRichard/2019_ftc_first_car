package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by 1ssem0 on 2019/5/11.
 */
@Autonomous(name = "first_red_2")
public class auto_red_two extends LinearOpMode{
    public static RobotMapFirst map = new RobotMapFirst();


    private static final String VUFORIA_KEY = "ASycpA3/////AAABmZlKpSHVQE0hh2bFhiowD2pIHMnNaULf9umyqYzoDIRDmjxsUOsVuA/D6bEEakL4ZgkYapm0TvIyUnra29r4naJ3GxLXYsGCZiwLPFkOWNQa2Ho/MAL3znh991UuQDxN/Et5HAEM67KUI4cI70ptW6n1esamplEnP+umtoAgAudNt+Xdb6ocrO2VMCZkKtwSTu+t8C2rTGhoHqJ8TtXy1H5pkAHcwQih/nD75Tma3T9V0rlBDyRM5Je6ImcpXwqkrshspeeIU59+B8k3L3z2pcpuZCjKYr9vqAYFoRd3XWFl9388KS21DRS/D8KsTmUnv7ps+bYHuerKQMAxgrZN9iJwcmc92LFe2L2bOUoJp51k";
//    private static final float mmPerInch        = 25.4f;
//    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
//    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static String position = "";

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;


    private static boolean teamCode = true;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        map.RobotInit(hardwareMap);
        map.basket.setPosition(map.basketDown);
        map.initDcmotor(map.leftFront);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        sleep(500);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        if (teamCode) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (teamCode) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    position = "Left";
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    position = "Right";
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    position = "Center";
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();

                    }
                }

                if (!position.isEmpty()) {
                    teamCode = false;
                }

            }
        }

        switch (position) {
            case "Left":
                caseLeft();
            case "Right":
                caseCenter();
            case "Center":
                caseRight();
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------





    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
       /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void caseLeft() {

    }

    private void caseRight() {

    }

    private void caseCenter() {

    }

    public void translationWithPID(double target, double zTranslation){
        double power = zTranslation;
        double motorPosition;
        map.initDcmotor(map.leftFront);
        while (power != 0.0){
            map.driveMecanum(0,0, power);
            motorPosition = map.leftFront.getCurrentPosition() / target;
            power *= map.pid_output(map.Kp_standard, map.Kd_standard, map.Ki_standard, motorPosition, 1, false);

            telemetry.addData("power", power);
            telemetry.addData("pos", map.leftFront.getCurrentPosition());
            telemetry.update();
        }

        map.driveMecanum(0,0,0);
    }

    public void rotationWithGyro(double targetAngle, double yRotation) {
        double power = yRotation;
        double motorPosition;
        map.initDcmotor(map.leftFront);
        while (power != 0.0) {
            map.driveMecanum(0, power, 0);
            motorPosition = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) / targetAngle;
            power *= map.pid_output(map.Kp_standard, map.Kd_standard, map.Ki_standard, motorPosition, 1, false);

            telemetry.addData("power", power);
            telemetry.addData("angle", map.dustPan.getCurrentPosition());
            telemetry.update();
        }
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}






