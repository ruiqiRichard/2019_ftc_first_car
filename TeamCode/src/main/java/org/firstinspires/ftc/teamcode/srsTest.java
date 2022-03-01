package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by 1ssem0 on 2019/5/7.
 */
@Disabled
@TeleOp(name = "SRS_TEST")
public class srsTest extends OpMode {
    public srsServoTest map = new srsServoTest();
    public static double srsTestPosition = 0.5;

    @Override
    public void init() {
        map.robotInit(hardwareMap);
        map.srs.setPosition(0.5);
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            srsTestPosition += 0.001;
            map.srs.setPosition(srsTestPosition);
        }

        if (gamepad1.b){
            srsTestPosition -= 0.001;
            map.srs.setPosition(srsTestPosition);
        }

        srsTestPosition = Range.clip(srsTestPosition, 0.0, 1.0);

        if (gamepad1.y) map.srs.setPosition(1.0);

        if (gamepad1.a) map.srs.setPosition(0.0);

        map.srs.setPosition(0.5);

        telemetry.addData("srsPosition",map.srs.getPosition());
        telemetry.update();

    }
}
