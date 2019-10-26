package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Servos{

    Servo rightServo;
    Servo leftServo;

    static final double RIGHT_DOWN = .4;
    static final double LEFT_DOWN = .35;
    static final double LEFT_UP = 0;
    static final double RIGHT_UP = 0;

    public Servos(Servo rs, Servo ls){
        rightServo = rs;
        rs.setDirection(Servo.Direction.FORWARD);
        rs.setPosition(0);

        leftServo = ls;
        ls.setDirection(Servo.Direction.REVERSE);
        ls.setPosition(0);
    }

    public void moveServos(double position){
        rightServo.setPosition(position);
        leftServo.setPosition(position);
    }
    public void servosDown(){
        rightServo.setPosition(RIGHT_DOWN);
        leftServo.setPosition(LEFT_DOWN);
    }

    public void servosUp(){
        rightServo.setPosition(RIGHT_UP);
        leftServo.setPosition(LEFT_UP);
    }


}