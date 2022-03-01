package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by 1ssem0 on 2018/12/14.
 */
@Disabled
@TeleOp(name = "2019tele_test", group = "Test")
public class TeleopBasic extends OpMode {
    public static RobotMapFirst map = new RobotMapFirst();
    @Override
    public void init() {
        map.RobotInit(hardwareMap);
    }

    @Override
    public void loop() {
        double lv = velocityLeft();
        double rv = velocityRight();

//        map.leftFront.setPower(lv);
//        map.leftrear.setPower(lv);
//        map.rightfront.setPower(rv);
//        map.rightrear.setPower(rv);
        displayValues(lv, rv);

    }

    /**
     * Method for calculating wheel speed
     * @return get the motor power for left wheel
     */
    public double velocityLeft(){
        double lv = 0;
        if (gamepad1.dpad_up){
            lv = lv - 0.5;
        } else {
            lv = lv + 0.5;
        }

        if (gamepad1.dpad_down){
            lv = lv +0.5;
        } else {
            lv = lv - 0.5;
        }

        if (gamepad1.dpad_left){
            lv = lv + 0.25;
        } else {
            lv = lv - 0.25;
        }

        if (gamepad1.dpad_right){
            lv = lv - 0.25;
        } else {
            lv = lv + 0.25;
        }

        lv = lv + gamepad1.left_stick_y;

        lv = Range.clip(lv, -1, 1);
        return lv;


    }

    /**
     * Method for right wheels' output
     * @return get the motor power for right wheel
     */
    public double velocityRight(){
        double rv = 0;
        if (gamepad1.dpad_up){
            rv = rv + 0.5;
        } else {
            rv = rv - 0.5;
        }

        if (gamepad1.dpad_down){
            rv = rv - 0.5;
        } else {
            rv = rv + 0.5;
        }

        if (gamepad1.dpad_left){
            rv = rv + 0.25;
        } else {
            rv = rv - 0.25;
        }

        if (gamepad1.dpad_right){
            rv = rv - 0.25;
        } else {
            rv = rv + 0.25;
        }

        rv = rv - gamepad1.right_stick_y;

        rv = Range.clip(rv, -1, 1);
        return rv;


    }
    public void displayValues(double lv, double rv){
        telemetry.addData("LeftVelocity:",lv);
        telemetry.addData("RightVelocity",rv);
    }
}
