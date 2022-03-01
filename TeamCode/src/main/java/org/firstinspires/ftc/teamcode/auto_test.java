package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by 1ssem0 on 2019/5/11.
 */
@Autonomous(name = "auto_test_first")
@Disabled
public class auto_test extends LinearOpMode {
    public RobotMapTest map = new RobotMapTest();

    private static final String VUFORIA_KEY = "ASycpA3/////AAABmZlKpSHVQE0hh2bFhiowD2pIHMnNaULf9umyqYzoDIRDmjxsUOsVuA/D6bEEakL4ZgkYapm0TvIyUnra29r4naJ3GxLXYsGCZiwLPFkOWNQa2Ho/MAL3znh991UuQDxN/Et5HAEM67KUI4cI70ptW6n1esamplEnP+umtoAgAudNt+Xdb6ocrO2VMCZkKtwSTu+t8C2rTGhoHqJ8TtXy1H5pkAHcwQih/nD75Tma3T9V0rlBDyRM5Je6ImcpXwqkrshspeeIU59+B8k3L3z2pcpuZCjKYr9vqAYFoRd3XWFl9388KS21DRS/D8KsTmUnv7ps+bYHuerKQMAxgrZN9iJwcmc92LFe2L2bOUoJp51k";
//    private static final float mmPerInch        = 25.4f;
//    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
//    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final double translation = -700;
    private static final double driveFirst = 800;
    private static final double angle = -1700;
    private static final double driveSecond = -4500;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static String position = "";

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


    private static boolean teamCode = true;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        map.robotInit(hardwareMap);
        map.initDcmotor(map.leftFront);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();



        driveWithPID(400, -0.8);
        sleep(500);
        rotationWithPID(1500,-0.8);
        sleep(500);
        driveWithPID(800, -1);
        sleep(500);
        rotationWithPID(-1500,0.8);
//        sleep(500);
//        rotationWithPID(1500, -0.8);
//        sleep(500);
//        rotationWithPID(1500, -0.8);
//        sleep(500);
        driveWithPID(2500,-1);

//        driveWithPID(-400, -0.8);
//        sleep(500);
//        rotationWithPID(-1500,-0.8);
//        sleep(500);
//        driveWithPID(-800, -1);
//        sleep(500);
//        rotationWithPID(1500,0.8);
////        sleep(500);
////        rotationWithPID(1500, -0.8);
////        sleep(500);
////        rotationWithPID(1500, -0.8);
////        sleep(500);
//        driveWithPID(-2500,-1);


//        translationWithPID(2000,1.0);
        while (opModeIsActive()){
            telemetry.addData("pos", map.leftFront.getCurrentPosition());
            telemetry.update();
        }



//        if (opModeIsActive()) {
//            /** Activate Tensor Flow Object Detection. */
//            if (tfod != null) {
//                tfod.activate();
//            }
//
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        if (updatedRecognitions.size() == 3) {
//                            int goldMineralX = -1;
//                            int silverMineral1X = -1;
//                            int silverMineral2X = -1;
//                            for (Recognition recognition : updatedRecognitions) {
//                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                    goldMineralX = (int) recognition.getLeft();
//                                } else if (silverMineral1X == -1) {
//                                    silverMineral1X = (int) recognition.getLeft();
//                                } else {
//                                    silverMineral2X = (int) recognition.getLeft();
//                                }
//                            }
//                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                }
//                            }
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }
    }

    private void caseLeft() {


        driveWithPID(-driveFirst, 1);
        rotationWithPID(angle, 0.9);
        driveWithPID(driveSecond,1);
    }

    private void caseRight() {

        driveWithPID(driveFirst, -1);
        rotationWithPID(angle, 0.9);
        driveWithPID(driveSecond,1);

    }

    private void caseCenter() {
        rotationWithPID(angle,0.9);
        driveWithPID(driveSecond,1);
    }


    //useless
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


    /**
     * Method for driving as a tank using PID
     * @param target the target value of encoder
     * @param xSpeed the set power of motors; negative values for left movements
     */
    public void driveWithPID(double target, double xSpeed){
        double power = xSpeed;
        double motorPosition;
        map.initDcmotor(map.leftFront);
        while (power != 0.0){
            map.driveMecanum(power,0, 0);
            motorPosition = map.leftFront.getCurrentPosition() / target;
            power *= map.pid_output(map.Kp_standard, map.Kd_standard, map.Ki_standard, motorPosition, 1, false);

            telemetry.addData("power", power);
            telemetry.addData("pos", map.leftFront.getCurrentPosition());
            telemetry.update();
        }

        map.driveMecanum(0,0, 0);
    }

    /**
     * Method for driving as a tank using PID
     * @param targetAngle the target value of encoder which starts from zero
     * @param yRotation the set power of motors; negative values for counterclockwise movements
     */

    public void rotationWithPID(double targetAngle, double yRotation){
        double power = yRotation;
        double currentAngle;
        map.initDcmotor(map.leftFront);
        while (power != 0.0){
            map.driveMecanum(0, power, 0);
            currentAngle = map.leftFront.getCurrentPosition() / targetAngle;
            power *= map.pid_output(map.Kp_standard, map.Kd_standard, map.Ki_standard, currentAngle, 1, false);

            telemetry.addData("power", power);
            telemetry.addData("pos", map.leftFront.getCurrentPosition());
            telemetry.update();
        }
        map.driveMecanum(0,0, 0);
    }

    /**
     * Reserved method for lifting
     * @param targetHeight the target value of encoder which starts from zero
     * @param liftSpeed set power
     */
    public void lift(int targetHeight, double liftSpeed){

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}


