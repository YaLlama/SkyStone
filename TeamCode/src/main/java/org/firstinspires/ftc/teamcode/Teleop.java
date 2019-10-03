package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Teleop extends OpMode {
    @Override
    public void init() {
        telemetry.addData("left y;", -gamepad1.left_stick_y);
        telemetry.addData("left x;", -gamepad1.left_stick_x);
        telemetry.addData("right y;", -gamepad1.right_stick_y);
        telemetry.addData("right x;", -gamepad1.right_stick_x);
    }

    @Override
    public void loop() {
        telemetry.update();
    }
}
