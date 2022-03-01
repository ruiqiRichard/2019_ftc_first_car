package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by 1ssem0 on 2019/5/11.
 */
@TeleOp(name = "driveTestSecond", group = "Test")
//@Disabled
public class driveTestSecond extends OpMode {
    public static RobotMapSecond map = new RobotMapSecond();
    public static double firstTranslation = 800;
    @Override
    public void init() {
        map.robotInit(hardwareMap);
        map.initDcmotor(map.leftFront);
    }

    @Override
    public void loop() {

        if (gamepad1.a){
            map.driveMecanum(0,0,-1);
        }
        else map.driveMecanum(0,0,0);

//        if (gamepad1.b){
//            driveWithPID(-800,0.5);
//        }

        if (gamepad1.x){
            map.driveMecanum(0, -1, 0);
        }

        else {
            map.driveMecanum(0,0,0);
        }

        if (gamepad1.y){
            map.driveMecanum(1,0,0);
        }
        else map.driveMecanum(0,0,0);

        telemetry.addData("position", map.leftFront.getCurrentPosition());
        telemetry.update();

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

        map.driveMecanum(0,0, 0);
    }

    public void rotationWithGyro(double targetAngle, double yRotation){
        double power = yRotation;
        double currentAngle;
        map.initDcmotor(map.leftFront);
        while (power != 0.0){
            map.driveMecanum(0, power, 0);
            currentAngle = map.leftFront.getCurrentPosition() / targetAngle;
            power *= map.pid_output(map.Kp_standard, map.Kd_standard, map.Ki_standard, currentAngle, 1, false);

//            telemetry.addData("power", power);
//            telemetry.addData("pos", map.dustPan.getCurrentPosition());
//            telemetry.update();
        }
        map.driveMecanum(0,0, 0);
    }

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


}
