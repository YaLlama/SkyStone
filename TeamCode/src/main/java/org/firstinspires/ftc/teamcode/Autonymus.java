package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Basic Auto", group = "Autonomous")
public class Autonymus extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SKIIIIIIIIIRT swerve = new SKIIIIIIIIIRT();
        DcMotor RF = hardwareMap.dcMotor.get("rightFront");
        DcMotor RB = hardwareMap.dcMotor.get("rightBack");
        DcMotor LF = hardwareMap.dcMotor.get("leftFront");
        DcMotor LB = hardwareMap.dcMotor.get("leftBack");
        swerve.initalize(RF, RB, LF, LB, 1);
        swerve.moveTo(0, 100);
        swerve.moveTo(0, -100);
        swerve.moveTo(90, 90);
        swerve.moveTo(-90, -90);
        swerve.moveTo(-45, 45);
        swerve.moveTo(45, -45);
        swerve.moveTo(30, 60);
        swerve.moveTo(-30, -60);
        swerve.moveTo(-30, 60);
        swerve.moveTo(30, -60);
        swerve.moveTo(60, 30);
        swerve.moveTo(-60, -30);
        swerve.moveTo(-60, 30);
        swerve.moveTo(60, -30);

    }
}
