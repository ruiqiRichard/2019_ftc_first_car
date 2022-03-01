package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Arrays;

/**
 * Created by 1ssem0 on 2019/5/11.
 */

public class RobotMapTest {

    public HardwareMap hwmap;

    public static DcMotor leftFront = null;
    public static DcMotor leftRear = null;
    public static DcMotor rightFront = null;
    public static DcMotor rightRear = null;

    public static double pid_previous = 0.0;
    public static double pid_sum = 0.0;

    public static final double Ki_standard = 0.1;
    public static final double Kp_standard = 0.8;
    public static final double Kd_standard = 0.35;
    public static final double kDefaultDeadband = 0.02;
    public static final int rotateRightAngle = -1500;
    public static final double hookLimit = 1.0;
    public static final int hookPath = 0;
    public static final double hookWithEncInitPower = 0.5;

    protected double m_deadband = kDefaultDeadband;

    public void robotInit(HardwareMap hardwareMap){
        this.hwmap = hardwareMap;
        this.leftFront = hwmap.dcMotor.get("leftFront");
        this.leftRear = hwmap.dcMotor.get("leftrear");
        this.rightFront = hwmap.dcMotor.get("rightfront");
        this.rightRear = hwmap.dcMotor.get("rightrear");
    }

    public void initDcmotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void driveMecanum(double xSpeed, double zRotation, double yTranslation) {
        xSpeed = limit(xSpeed);
//        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = limit(zRotation);
//        zRotation = applyDeadband(zRotation, m_deadband);

        double[] leftMotorOutput = new double[2];
        double[] rightMotorOutput = new double[2];

        if (xSpeed > 0.0){
            leftMotorOutput [0] = -xSpeed;
            leftMotorOutput [1] = -xSpeed;
            rightMotorOutput [0] = xSpeed;
            rightMotorOutput [1] = xSpeed;
        }

        else if (xSpeed < 0.0){
            leftMotorOutput [0] = -xSpeed;
            leftMotorOutput [1] = -xSpeed;
            rightMotorOutput [0] = xSpeed;
            rightMotorOutput [1] = xSpeed;
        }

        else if (zRotation != 0.0){
            leftMotorOutput [0] = -zRotation;
            leftMotorOutput [1] = -zRotation;
            rightMotorOutput [0] = -zRotation;
            rightMotorOutput [1] = -zRotation;
        }


        else if (yTranslation > 0.0){
            leftMotorOutput[0] = yTranslation;
            leftMotorOutput[1] = -yTranslation;
            rightMotorOutput[0] = yTranslation;
            rightMotorOutput[1] = -yTranslation;
        }

        else if (yTranslation < 0.0){
            leftMotorOutput[0] = yTranslation;
            leftMotorOutput[1] = -yTranslation;
            rightMotorOutput[0] = yTranslation;
            rightMotorOutput[1] = -yTranslation;
        }

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
}
