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
        telemetry.addData("x", srt.posX());
        telemetry.addData("y", srt.posY());
        telemetry.update();
        waitForStart();
        srt.moveTo(45, 45, .3, 1);

    }
}
