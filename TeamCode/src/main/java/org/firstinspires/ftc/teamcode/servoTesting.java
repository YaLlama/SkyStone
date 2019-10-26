package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "testServo")
public class servoTesting extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        Servo rS = hardwareMap.servo.get("rightServo");
        Servo lS = hardwareMap.servo.get("leftServo");

        Servos s = new Servos(rS, lS);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.x){
                s.servosDown();
            }else if(gamepad1.y){
                s.servosUp();
            }
        }
    }

}
