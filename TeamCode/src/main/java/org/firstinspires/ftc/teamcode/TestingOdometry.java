package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "testodometry")

public class TestingOdometry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor RFE = hardwareMap.dcMotor.get("rightEncoder");
        DcMotor LFE = hardwareMap.dcMotor.get("leftEncoder");
        DcMotor LBE = hardwareMap.dcMotor.get("backEncoder");
        DcMotor RB = hardwareMap.dcMotor.get("rightBack");
        SKRTOdometry srt = new SKRTOdometry(RFE, LFE, LBE, RB);
        telemetry.addData("x", srt.posX());
        telemetry.addData("y", srt.posY());
        telemetry.update();
        waitForStart();
        srt.moveTo(45, 45, .3, 0, 90, 0);

    }
}
