package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.code.Attribute;

import java.lang.reflect.Array;
import java.util.Arrays;

/**
 * Created by 1ssem0 on 2018/12/7.
 */
//@Disabled
public class RobotMapFirst {
    public HardwareMap hwmap;

    public static DcMotor leftFront = null;
    public static DcMotor leftRear = null;
    public static DcMotor rightFront = null;
    public static DcMotor rightRear = null;
    public static DcMotor dustPan = null;
//    public static DcMotor leftElevator = null;
//    public static DcMotor rightElevator = null;
    public static DcMotor elevator = null;
    public static DcMotor extension = null;
    public static DcMotor hook = null;
    public static Servo brush = null;
    public static Servo basket = null;
//    public static ColorSensor colorSensor = null;
    public static DigitalChannel digitalChannel = null;




    //    public static DistanceSensor sensorRange = null;
    public static final double basketUp = 0.30d;
    public static final double basketDown = 0.76d;
    public static final double kDefaultDeadband = 0.02;
    public static final double kDefaultMaxOutput = 1.0;
    public static final double Ki_standard = 0.1;
    public static final double Kp_standard = 0.8;
    public static final double Kd_standard = 0.35;
    public static final double dustPanPath = 1800;
    public static final int hookPath = 10700;

    protected double m_deadband = kDefaultDeadband;
    protected double m_maxOutput = kDefaultMaxOutput;
    protected double pid_previous = 0;
    protected double pid_sum = 0;

    public void RobotInit(HardwareMap hardwareMap) {
        this.hwmap = hardwareMap;
        this.leftFront = hwmap.dcMotor.get("leftFront");
        this.leftRear = hwmap.dcMotor.get("leftrear");
        this.rightFront = hwmap.dcMotor.get("rightfront");
        this.rightRear = hwmap.dcMotor.get("rightrear");
//        this.leftElevator = hwmap.dcMotor.get("l-elevator");
//        this.rightElevator = hwmap.dcMotor.get("r-elevator");
        this.elevator = hwmap.dcMotor.get("elevator");
        this.dustPan = hwmap.dcMotor.get("shovel");
        this.extension = hwmap.dcMotor.get("extension");
        this.hook = hwmap.dcMotor.get("hook");
        this.brush = hwmap.servo.get("dust_brush");
        this.basket = hwmap.servo.get("basket");
        this.digitalChannel = hwmap.digitalChannel.get("digitalChannel");
//        this.colorSensor = hwmap.colorSensor.get("colorsensor");
//        this.sensorRange = hwmap.get(DistanceSensor.class, "sensor_range");

    }

    public void Drivearcade_tank(double xSpeed, double zRotation, boolean squaredInputs) {
        xSpeed = limit(xSpeed);
        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = limit(zRotation);
        zRotation = applyDeadband(zRotation, m_deadband);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squaredInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

//        this.leftrear.setPower(limit(leftMotorOutput) * m_maxOutput);
//        this.leftFront.setPower(limit(leftMotorOutput) * m_maxOutput);
//        this.rightrear.setPower(-limit(rightMotorOutput) * m_maxOutput);
//        this.rightfront.setPower(-limit(rightMotorOutput) * m_maxOutput);
    }

    public void driveMecanum(double xSpeed, double zRotation, double yTranslation) {
        xSpeed = limit(xSpeed);
//        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = limit(zRotation);
//        zRotation = applyDeadband(zRotation, m_deadband);

        double[] leftMotorOutput = new double[2];
        double[] rightMotorOutput = new double[2];

        if (xSpeed != 0.0){
            leftMotorOutput [0] = -xSpeed;
            leftMotorOutput [1] = -xSpeed;
            rightMotorOutput [0] = xSpeed;
            rightMotorOutput [1] = xSpeed;
        }

//        else if (xSpeed < 0.0){
//            leftMotorOutput [0] = -xSpeed;
//            leftMotorOutput [1] = -xSpeed;
//            rightMotorOutput [0] = xSpeed;
//            rightMotorOutput [1] = xSpeed;
//        }

        else if (zRotation != 0.0){
            leftMotorOutput [0] = -zRotation;
            leftMotorOutput [1] = -zRotation;
            rightMotorOutput [0] = -zRotation;
            rightMotorOutput [1] = -zRotation;
        }


        else if (yTranslation != 0.0){
            leftMotorOutput[0] = yTranslation;
            leftMotorOutput[1] = -yTranslation;
            rightMotorOutput[0] = yTranslation;
            rightMotorOutput[1] = -yTranslation;
        }

//        else if (yTranslation < 0.0){
//            leftMotorOutput[0] = yTranslation;
//            leftMotorOutput[1] = -yTranslation;
//            rightMotorOutput[0] = yTranslation;
//            rightMotorOutput[1] = -yTranslation;
//        }

        else {
            Arrays.fill(leftMotorOutput, 0);
            Arrays.fill(rightMotorOutput, 0);
        }


        this.leftRear.setPower(limit(leftMotorOutput[0]));
        this.leftFront.setPower(limit(leftMotorOutput[1]));
        this.rightRear.setPower(limit(rightMotorOutput[0]));
        this.rightFront.setPower(limit(rightMotorOutput[1]));




    }

    public double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    protected double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Function to get PID result
     * @param Kp is proportional constant
     * @param Kd is differential constant
     * @param Ki is integral constant
     * @param present_input is the present instant value reached
     * @param target is the target terminal value reached
     * @param simple is used to switch to the simpler way to get PID result
     * @return get the PID result
     */
    public double pid_output(double Kp, double Kd, double Ki, double present_input, double target, boolean simple) {
        double output = 1.0;
        double error = target - present_input;
        if (simple) {
            output = output * error / target;

        } else {

            if (pid_previous != 0.0){
                output = Kp * error + Ki * pid_sum + Kd * (error - pid_previous);
            }

            pid_previous = error;
            pid_sum += pid_previous;
            if (pid_previous <= 0.0) {
                output = 0.0;
            }


        }
        return output;
    }

    public void initDcmotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void translationWithPID(double target, double zTranslation){
        double power = zTranslation;
        double motorPosition = 0.0;
        initDcmotor(leftFront);
        while (power != 0.0){
            driveMecanum(0,0, power);
            motorPosition = leftFront.getCurrentPosition() / target;
            power *= pid_output(Kp_standard, Kd_standard, Ki_standard, motorPosition, 1, false);

//            telemetry.addData("power", power);
//            telemetry.addData("pos", map.dustPan.getCurrentPosition());
//            telemetry.update();
        }

        leftFront.setPower(0);
    }


//    public double pid_output(double Kp, double Kd, double Ki, double present_input, double target) {
//        double output = 1.0;
//        double error = target - present_input;
////        if (simple) {
////            output = output * error / target;
////
////        }
////else if (!simple){
//
//            output = Kp * error + Ki * pid_sum + Kd * (error - pid_previous);
//
//            pid_previous = error;
//            pid_sum += pid_previous;
//            if (pid_previous < 0.0) {
//                output = 0.0;
//            }



//        return output;
//    }
}