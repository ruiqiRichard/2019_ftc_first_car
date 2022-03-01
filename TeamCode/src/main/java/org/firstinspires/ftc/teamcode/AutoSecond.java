package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by 1ssem0 on 2019/5/11.
 */
@Autonomous(name = "auto_second")
public class AutoSecond extends LinearOpMode{
    public static RobotMapSecond map = new RobotMapSecond();
    @Override
    public void runOpMode() throws InterruptedException {
        map.robotInit(hardwareMap);

        waitForStart();

        map.locker.setPosition(0.0d);
        map.hook.setPower(1.0);
        sleep(100);
//        translationWithPID(-700, 0.6);

//

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
}
