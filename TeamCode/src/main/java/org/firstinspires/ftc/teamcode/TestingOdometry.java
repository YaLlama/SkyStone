package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "testodometry")

public class TestingOdometry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor RFE = hardwareMap.dcMotor.get("rightFrontEncoder");
        DcMotor LFE = hardwareMap.dcMotor.get("leftFrontEncoder");
        DcMotor RBE = hardwareMap.dcMotor.get("rightBackEncoder");
        DcMotor LB = hardwareMap.dcMotor.get("leftBack");
        SKRTOdometry srt = new SKRTOdometry(RFE, RBE, LFE, LB);
        waitForStart();
        srt.moveTo(45, 45, .3);
        srt.moveTo(-90, -90, .3);
        srt.moveTo(45, 45, .3);
        srt.moveTo(60, 30, .3);
        srt.moveTo(-120, -60, .3);
        srt.moveTo(60, 30, .3);
        srt.moveTo(30, 60, .3);
        srt.moveTo(-60, -120, .3);
        srt.moveTo(-45, 45, .3);
        srt.moveTo(90, -90, .3);
        srt.moveTo(-45,45 , .3);
        srt.moveTo(-60, 30, .3);
        srt.moveTo(120, -60, .3);
        srt.moveTo(-60, 30, .3);
        srt.moveTo(-30, 60, .3);
        srt.moveTo(60, -120, .3);
        srt.moveTo(-30, 60, .3);
        srt.moveTo(100, 0, .3);
        srt.moveTo(-200, 0, .3);
        srt.moveTo(100, 0, .3);
        srt.moveTo(0, 100, .3);
        srt.moveTo(0, -200, .3);
        srt.moveTo(0, 100, .3);
        srt.moveTo(45, 45, .3, 90);
    }
}
