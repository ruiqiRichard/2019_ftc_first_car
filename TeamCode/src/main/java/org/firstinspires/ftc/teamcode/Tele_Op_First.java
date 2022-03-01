package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by 1ssem0 on 2018/12/7.
 */

//@Disabled
@TeleOp(name = "2019_firstTeleOp")
public class Tele_Op_First extends OpMode {
    public static RobotMapFirst map = new RobotMapFirst();
    public static double leftMotorPower = 0.0;
    public static double rv = 0.0;
    public static double lv = 0.0;
    public static double rightMotorPower = 0.0;
//    public static double dust_position = 0.0;
//    public static double power = 0.0;
//    public static double brushPower = 0.5;
    public static final double init_power = 1.0d;
//    public static final double target_pos = 1900;
    public static double init_brush_pos = map.basketDown;
    public static final double ki = 0.1;
    public static final double kp = 0.8;
    public static final double kd = 0.35;
//    public static double pos = init_pos;
    @Override
    public void init() {
        map.RobotInit(hardwareMap);
        map.initDcmotor(map.hook);
        map.initDcmotor(map.dustPan);
//        map.brush.setDirection(Servo.Direction.FORWARD);
        map.basket.setPosition(map.basketDown);
        map.digitalChannel.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public void loop() {

        velocityLeft();
        velocityRight();
        map.leftRear.setPower(lv);
        map.leftFront.setPower(lv);
        map.rightFront.setPower(rv);
        map.rightRear.setPower(rv);


        brushMove();
        elevator();
//        hookMove(3000);
        translation();
//        dustPanMove(map.dustPanPath);
        hook();
        handMode();
        extensionStructure();
        displayValues(leftMotorPower, rightMotorPower);


    }


    /**
     * Function for two servos
     */
    public void brushMove() {

        if (gamepad2.y){
            map.basket.setPosition(map.basketUp);
        }

        else if (gamepad2.a){
            map.basket.setPosition(map.basketDown);

        }

//        else {
//            map.brush.setPosition(brush_position);
//        }

        if (gamepad2.x) {
//            brushPower += 0.001;
//            map.brush.setPosition(brushPower);
            map.brush.setPosition(0.05);
        }

        else if (gamepad2.b) {
//            brushPower -= 0.001;
//            map.brush.setPosition(brushPower);
            map.brush.setPosition(0.9);
        }

        else {
            map.brush.setPosition(0);
        }

        if (gamepad2.left_stick_button){
            init_brush_pos += 0.001;
            map.basket.setPosition(init_brush_pos);
        }

        else if (gamepad2.right_stick_button){
            init_brush_pos -= 0.001;
            map.basket.setPosition(init_brush_pos);
        }

        init_brush_pos = Range.clip(init_brush_pos,0.0,1.0);







    }

    /**
     * Function for elevator motors
     */
    public void elevator(){

        if (gamepad2.dpad_up){
//            map.leftElevator.setPower(-1);
//            map.rightElevator.setPower(-1);
            map.elevator.setPower(-1);
        }

        else if (gamepad2.dpad_down){
//            map.leftElevator.setPower(1);
//            map.rightElevator.setPower(1);
            map.elevator.setPower(1);
        }

        else {
//            map.leftElevator.setPower(0);
//            map.rightElevator.setPower(0);
            map.elevator.setPower(0);
        }

        if (map.digitalChannel.getState() == false){
            map.elevator.setPower(0);
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
     * Function is for displaying various values
     * @param lv is left wheels' power
     * @param rv is right wheels' power
     */
    public void displayValues(double lv, double rv){
        telemetry.addData("LeftVelocity:",lv);
        telemetry.addData("RightVelocity",rv);
        telemetry.addData("dustpan_position", map.dustPan.getCurrentPosition());
        telemetry.addData("hook_position", map.hook.getCurrentPosition());
//        telemetry.addData("current_dust_power",power);
//        telemetry.addData("brush_power",brushPower);
        telemetry.addData("brush_pos", map.brush.getPosition());
        telemetry.addData("basket_position",map.basket.getPosition());
        telemetry.update();
    }

    /**
     * Hand Mode for manipulating dustPan
     */
    public void handMode(){
        if (gamepad1.right_trigger >0.5f){
            map.dustPan.setPower(init_power);
        }

        else if (gamepad1.left_trigger>0.5f){
            map.dustPan.setPower(-init_power);
        }

//        else if (gamepad1.a){
//            map.hook.setPower(init_power);
//        }
//
//        else if (gamepad1.b) map.hook.setPower(-init_power);


        else {
            map.dustPan.setPower(0);
        }


    }

    /**
     * Function for extension bridge
     */
    public void extensionStructure(){
        if (gamepad2.dpad_left) map.extension.setPower(1);
        else if (gamepad2.dpad_right) map.extension.setPower(-1);
        else map.extension.setPower(0);
//        map.extension.setPower(-gamepad2.left_trigger);
//        map.extension.setPower(gamepad2.right_trigger);
    }

    public void hook(){
//        map.initDcmotor(map.hook);
        int position = map.hook.getCurrentPosition();


        if (position > 10000){
            if (gamepad1.a) map.hook.setPower(-1);
            else if (gamepad1.y) map.hook.setPower(1);
            if (gamepad2.left_bumper) map.hook.setPower(-1);
            else map.hook.setPower(0);
        }

        else if (position < 0){
            if (gamepad1.a) map.hook.setPower(-1);
            else if (gamepad1.y) map.hook.setPower(1);
            if (gamepad2.dpad_right) map.hook.setPower(1);
            else map.hook.setPower(0);
        }

        else {
            if (gamepad2.right_bumper) map.hook.setPower(1);
            else if (gamepad2.left_bumper) map.hook.setPower(-1);
            else map.hook.setPower(0);
        }
    }


    /**
     * Function for hook to lift using PID
     */
//    public void hookMove(double target_pos){
//        double hookPosition;
//        // down
//        if (gamepad2.right_bumper){
//
//            if (map.hook.getCurrentPosition() < 1){
//                power = init_power;
//                map.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                map.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                while (power != 0.0){
//                    map.hook.setPower(power);
//                    hookPosition = map.hook.getCurrentPosition() / target_pos;
//                    power *= map.pid_output(kp,kd, ki, hookPosition,1,false);
//
//                    telemetry.addData("power", power);
//                    telemetry.addData("pos", map.hook.getCurrentPosition());
//                    telemetry.update();
//                }
//            }
//
//            map.hook.setPower(0);
//        }
//
//
//        // up
//        if (gamepad2.left_bumper) {
//
//            if (map.hook.getCurrentPosition() > -1){
//                power = -init_power;
//                map.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                map.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                while (power != 0.0){
//                    map.hook.setPower(power);
//                    hookPosition = map.hook.getCurrentPosition() / -target_pos;
//                    power *= map.pid_output(kp,kd, ki, hookPosition,1,false);
//
//                    telemetry.addData("power", power);
//                    telemetry.addData("pos", map.hook.getCurrentPosition());
//                    telemetry.update();
//                }
//            }
//
//            map.hook.setPower(0);
//        }
//    }

    /**
     * Function for dustPan to lift using PID
     */
    public void hookMove(double target_pos){
        double hookPosition;
        double power;
        // down
        if (gamepad2.right_bumper){

            if (map.hook.getCurrentPosition() < 1){
                power = init_power;
                map.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                map.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (power != 0.1){
                    map.hook.setPower(power);
                    hookPosition = map.hook.getCurrentPosition() / target_pos;
                    power *= map.pid_output(kp,kd, ki, hookPosition,1,false);

                    telemetry.addData("power", power);
                    telemetry.addData("proportion", hookPosition);
                    telemetry.addData("pos", map.hook.getCurrentPosition());
                    telemetry.update();
//                    if (power < 0.1){
//                        break;
//                    }
                }
            }

            map.hook.setPower(0);
        }


        // up
        if (gamepad2.left_bumper){

            if (map.hook.getCurrentPosition() > -1){
                power = -init_power;
                map.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                map.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (power != 0.0){
                    map.hook.setPower(power);
                    hookPosition = map.hook.getCurrentPosition() / -target_pos;
                    power *= map.pid_output(kp, kd, ki, hookPosition,1,false);

                    telemetry.addData("power", power);
                    telemetry.addData("pos", map.hook.getCurrentPosition());
                    telemetry.update();

                }
            }

            map.hook.setPower(0);
        }
    }



}

