package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SKIRTOdometry;

@Autonomous(name = "testodometry")

public class TestingOdometry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SKIRTOdometry srt = new SKIRTOdometry();
        DcMotor RFE = hardwareMap.dcMotor.get("rightFrontEncoder");
        DcMotor LFE = hardwareMap.dcMotor.get("leftFrontEncoder");
        DcMotor RBE = hardwareMap.dcMotor.get("rightBackEncoder");
        DcMotor LB = hardwareMap.dcMotor.get("leftBack");
        srt.initalize(RFE, RBE, LFE, LB);
        waitForStart();
        srt.move(45, 45, .2);
    }
}
