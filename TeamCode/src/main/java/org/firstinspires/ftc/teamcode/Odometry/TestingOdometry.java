package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "testodometry")

public class TestingOdometry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SKIRTOdometry srt = new SKIRTOdometry();
        DcMotor RFE = hardwareMap.dcMotor.get("rightFront");
        DcMotor LFE = hardwareMap.dcMotor.get("leftFront");
        DcMotor RBE = hardwareMap.dcMotor.get("rightBack");
        DcMotor LB = hardwareMap.dcMotor.get("leftBack");
        waitForStart();
        srt.initalize(RFE, RBE, LFE, LB);
        srt.move(45, 45, .2);
    }
}
