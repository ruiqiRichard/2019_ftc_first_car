package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 1ssem0 on 2019/5/9.
 */

@TeleOp(name = "2019_secondTeleop")
public class Tele_Op_Second extends OpMode {
    public static RobotMapSecond map = new RobotMapSecond();

    public static double lv = 0.0;
    public static double rv = 0.0;
    public static double init_power = 0.5;
    public static double lockerTestPosition = 0.0d;


    public static final double ki = 0.1;
    public static final double kp = 0.8;
    public static final double kd = 0.35;

    @Override
    public void init() {
        map.robotInit(hardwareMap);
//        map.locker.setPosition(lockerTestPosition);
        map.locker.setDirection(Servo.Direction.FORWARD);
//        map.initDcmotor(map.leftFront);
        map.initDcmotor(map.hook);

    }



    @Override
    public void loop() {

        velocityLeft();
        velocityRight();
        map.leftRear.setPower(lv);
        map.leftFront.setPower(lv);
        map.rightRear.setPower(rv);
        map.rightFront.setPower(rv);
        translation();
        hookTest();
        lockerTest();
        displayValues();
//        if (gamepad2.y) map.locker.setPosition(0.0);
//        else if (gamepad2.x) map.locker.setPosition(1.0);




    }

    public void lockerTest(){
//        if (gamepad2.a) {
//            lockerTestPosition += 0.001;
//            map.locker.setPosition(lockerTestPosition);
////            map.locker.setPosition(0.4d);
//        }
//        else if (gamepad2.b) {
//            lockerTestPosition -= 0.001;
//            map.locker.setPosition(lockerTestPosition);
////            map.locker.setPosition(0.6d);
//        }
//        else map.locker.setPosition(0.5d);

        if (gamepad2.a) map.locker.setPosition(1.0d);
        else if (gamepad2.y) map.locker.setPosition(0.0d);

        lockerTestPosition = Range.clip(lockerTestPosition, 0.0, 1.0);
    }

    public void hookTest(){
        if (gamepad2.left_bumper){
            map.hook.setPower(0.5);
        }

        else if (gamepad2.right_bumper){
            map.hook.setPower(-0.5);
        }

//        else if (gamepad1.y){
//            map.hook.setPower(0.5);
//        }
//
//        else if (gamepad1.a) map.hook.setPower(-0.5);

        else map.hook.setPower(0.0);
    }

    public void translation(){
        if (gamepad1.b){
            map.leftFront.setPower(-1.0);
            map.leftRear.setPower(1.0);
            map.rightFront.setPower(-1.0);
            map.rightRear.setPower(1.0);

        }

        else if (gamepad1.x){
            map.leftFront.setPower(1.0);
            map.leftRear.setPower(-1.0);
            map.rightFront.setPower(1.0);
            map.rightRear.setPower(-1.0);
        }

        else {
            map.leftFront.setPower(0);
            map.leftRear.setPower(0);
            map.rightFront.setPower(0);
            map.rightRear.setPower(0);
        }
    }

    /**
     * Function for hook to lift using PID
     */
    public void hookMove(double target_pos){
        double hookPosition;
        double power = map.hookWithEncInitPower;
        // down
        if (gamepad2.dpad_up){

            if (map.hook.getCurrentPosition() < map.hookLimit){
//                power = map.hookWithEncInitPower;
                map.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                map.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (power != 0.0){
                    map.hook.setPower(power);
                    hookPosition = map.hook.getCurrentPosition() / target_pos;
                    power *= map.pid_output(kp,kd, ki, hookPosition,1,false);

                    telemetry.addData("power", power);
                    telemetry.addData("pos", map.hook.getCurrentPosition());
                    telemetry.update();
                }
            }

            map.hook.setPower(0);
        }


        // up
        if (gamepad2.dpad_down) {

            if (map.hook.getCurrentPosition() > -map.hookLimit){
//                power = -init_power;
                map.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                map.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (power != 0.0){
                    map.hook.setPower(-power);
                    hookPosition = map.hook.getCurrentPosition() / -target_pos;
                    power *= map.pid_output(kp,kd, ki, hookPosition,1,false);

                    telemetry.addData("power", -power);
                    telemetry.addData("pos", map.hook.getCurrentPosition());
                    telemetry.update();
                }
            }

            map.hook.setPower(0);
        }
    }

    /**
     * Method for calculating wheel speed
     * @return get the motor power for left wheel
     */
    public void velocityLeft(){

//        double lv = 0.0;

        if (gamepad1.dpad_up)
            lv = -1.0;
//        } else {
//            rv = rv - 0.5;
//        }

        else if (gamepad1.dpad_down)
            lv = 1.0;
//        } else {
//            rv = rv + 0.5;
//        }

        else if (gamepad1.dpad_left){
            lv = 1.0;
        }

        else if (gamepad1.dpad_right) {
            lv = -1.0;
        }

        else if (gamepad1.left_stick_y != 0.0) lv = gamepad1.left_stick_y;

        else lv = 0.0;



        lv = Range.clip(lv, -1, 1);
//        return lv;


    }

    /**
     * Method for right wheels' output
     * @return  get the motor power for right wheel
     */
    public void velocityRight(){


        if (gamepad1.dpad_up)
            rv = 1.0;
//        } else {
//            rv = rv - 0.5;
//        }

        else if (gamepad1.dpad_down)
            rv = -1.0;
//        } else {
//            rv = rv + 0.5;
//        }

        else if (gamepad1.dpad_left){
            rv = 1.0;
        }

        else if (gamepad1.dpad_right) {
            rv = -1.0;
        }

        else if (gamepad1.right_stick_y != 0.0) rv = -gamepad1.right_stick_y;

        else rv = 0.0;



        rv = Range.clip(rv, -1, 1);
//        return rv;


    }

    public void displayValues(){
        telemetry.addData("hookPosition",map.hook.getCurrentPosition());
//        telemetry.addData("motorPosition", map.leftFront.getCurrentPosition());
        telemetry.addData("lockerPosition", map.locker.getPosition());
        telemetry.addData("leftMotorPower", lv);
        telemetry.addData("rightMotorPower",rv);
        telemetry.addData("lockerNumPos", lockerTestPosition);
        telemetry.update();
    }
}
