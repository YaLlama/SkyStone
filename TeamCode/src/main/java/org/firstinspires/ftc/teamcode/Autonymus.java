package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


import java.lang.*;

public class Autonymus {

    //VARIABLES
    //
    //
    //
    //

    //location ,location, location
    //robot staring location
    public int robotX = 0;
    public int robotY = 0;

    //for moving command
    boolean swapped = false;

    // case for movement command
    public int Case = 0;

    //declaring motors
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    //importing other stuff

    //ticks per motor full rotation
    static final double MOTOR_TICK_COUNT = 537.6;

    //wheel diameter cm
    static final int WHEEL_DIAMETER = 10;

    //circumference of wheels in centimeters
    static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    //Power limit
    static final double POWER_FACTOR = .6;

    //number of rotations of wheels to move wheel (circumference) at 45degree strafe
    static final int STRAFE_FACTOR = 2;

    //gear ratio wheel divided by motor
    static final double GEER_RATIO = 4/3;


    //SHORTCUTS / COMMANDS
    //
    //
    //
    //

    public int distanceToTics(double d) {
        //d is centimeters want to travel
        //divides the distance yoi want to got by circumfrance to get number of motor rotations necessary then converts to number to tics needed to do so
        return (int) (d / WHEEL_CIRCUMFRENCE * MOTOR_TICK_COUNT * GEER_RATIO);
    }

    public void initalize(DcMotor RF, DcMotor RB, DcMotor LF, DcMotor LB) {
        //declaring motors
        rightFront = RF;
        leftFront = LF;
        rightBack = RB;
        leftBack = LB;

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting motors to fun with encoders
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void testMotors(){
        leftFront.setTargetPosition(distanceToTics(10));
        rightFront.setTargetPosition(distanceToTics(10));
        rightBack.setTargetPosition(distanceToTics(10));
        leftBack.setTargetPosition(distanceToTics(10));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.2);
        while(leftFront.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        rightFront.setPower(.2);
        while(rightFront.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        rightBack.setPower(.2);
        while(rightBack.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        leftBack.setPower(.2);
        while(leftBack.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void move(int x, int y) {
        // x and y are distances in centimeters


        //creating variables for distances each motor needs to travel and power levels and special stuff
        int distanceLF;
        int distanceLB;
        int distanceRF;
        int distanceRB;

        double PowerLF;
        double PowerLB;
        double PowerRF;
        double PowerRB;

        double specialPower;
        int Specialdistanjce;

        boolean PerfectStrafe = false;


        //if no horizontal movement is necessary
        if (x == 0) {
            distanceLF = STRAFE_FACTOR * y;
            distanceLB = STRAFE_FACTOR * y;
            distanceRF = STRAFE_FACTOR * y;
            distanceRB = STRAFE_FACTOR * y;

            PowerLB = POWER_FACTOR;
            PowerLF = POWER_FACTOR;
            PowerRB = POWER_FACTOR;
            PowerRF = POWER_FACTOR;

        } else {
            //calculates how much each motor needs to move
            Specialdistanjce = (int)(Math.sqrt((x * x) + (y * y)));
            // currently wronmg needs to be fixed
            specialPower = Math.toDegrees(Math.atan(Math.abs((double)y/(double)x)));


            if(specialPower == 45){
                specialPower = 0;
                PerfectStrafe = true;
            }
            else if(specialPower > 45){
                specialPower = specialPower / 45 - 1;
            }else{
                specialPower = -specialPower / 45 +1;
            }

            if(swapped){
                int placeholder = x;
                x = y;
                y = placeholder;
                swapped = false;
            }

            if(Case == 1){
                //motor power levels
                PowerRF = POWER_FACTOR;
                PowerLB = POWER_FACTOR;
                PowerLF = specialPower * POWER_FACTOR;
                PowerRB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = Specialdistanjce;
                distanceLB = Specialdistanjce;
                distanceLF = (int) -(Specialdistanjce * specialPower);
                distanceRB = (int) -(Specialdistanjce * specialPower);

            }else if(Case == 2){
                //motor power levels
                PowerLF = POWER_FACTOR;
                PowerRB = POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;


                //set motor diances
                distanceLF = Specialdistanjce;
                distanceRB = Specialdistanjce;
                distanceRF = (int) (Specialdistanjce * specialPower);
                distanceLB = (int) (Specialdistanjce * specialPower);


            }else if(Case == 3){
                //motor power levels
                PowerLF = POWER_FACTOR;
                PowerRB = POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = Specialdistanjce;
                distanceRB = Specialdistanjce;
                distanceRF = (int) -(Specialdistanjce * specialPower);
                distanceLB = (int) -(Specialdistanjce * specialPower);
            }else if(Case == 4){
                //motor power levels
                PowerRF = POWER_FACTOR;
                PowerLB = POWER_FACTOR;
                PowerLF = specialPower * POWER_FACTOR;
                PowerRB = specialPower * POWER_FACTOR;


                //set motor diances
                distanceRF = -Specialdistanjce;
                distanceLB = -Specialdistanjce;
                distanceLF = (int) -(Specialdistanjce * specialPower);
                distanceRB = (int) -(Specialdistanjce * specialPower);

            }else if(Case == 5){
                //motor power levels
                PowerRF = POWER_FACTOR;
                PowerLB = POWER_FACTOR;
                PowerLF = specialPower * POWER_FACTOR;
                PowerRB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = -Specialdistanjce;
                distanceLB = -Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);

            }else if(Case == 6){
                //motor power levels
                PowerLF = POWER_FACTOR;
                PowerRB = POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = -Specialdistanjce;
                distanceRB = -Specialdistanjce;
                distanceRF = (int) -(Specialdistanjce * specialPower);
                distanceLB = (int) -(Specialdistanjce * specialPower);

            }else if(Case == 7){
                //motor power levels
                PowerLF = POWER_FACTOR;
                PowerRB = POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceLF = -Specialdistanjce;
                distanceRB = -Specialdistanjce;
                distanceRF = (int) (Specialdistanjce * specialPower);
                distanceLB = (int) (Specialdistanjce * specialPower);
            }else{
                //motor power levels
                PowerRF = POWER_FACTOR;
                PowerLB = POWER_FACTOR;
                PowerLF = specialPower * POWER_FACTOR;
                PowerRB = specialPower * POWER_FACTOR;

                //set motor diances
                distanceRF = Specialdistanjce;
                distanceLB = Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);
            }

        }


        //calculates number of tics necessary and tells motors to go that many
        leftFront.setTargetPosition(distanceToTics(distanceLF));
        rightFront.setTargetPosition(distanceToTics(distanceRF));
        rightBack.setTargetPosition(distanceToTics(distanceRB));
        leftBack.setTargetPosition(distanceToTics(distanceLB));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(PowerLF);
        rightFront.setPower(PowerRF);
        rightBack.setPower(PowerRB);
        leftBack.setPower(PowerLB);

        if(PerfectStrafe){
            if(Case == 2 || Case == 7 || Case == 3 || Case == 6){
                while (leftFront.isBusy() && rightBack.isBusy()) {
                    try{
                        Thread.sleep(80);
                    }catch(Exception e){
                        System.out.print(e);
                    }
                }
            }else{
                while (rightFront.isBusy() && leftBack.isBusy()) {
                    try{
                        Thread.sleep(80);
                    }catch(Exception e){
                        System.out.print(e);
                    }
                }
            }
        }else{
            while (rightFront.isBusy() && leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy()) {
                try{
                    Thread.sleep(80);
                }catch(Exception e){
                    System.out.print(e);
                }
            }
        }

        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        //Resets
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //tells us new location
        robotX += x;
        robotY += y;


    }

    public void moveTo(int x, int y) {
        //moves to desired x,y
        double slope;
        if(x == 0){
            slope = 0;
        }else{
            slope = (double)y / (double)x;
        }

        if (-1 <= slope && slope <= 1) {
            if (y >= 0) {
                if(x < 0){
                    Case = 1;
                }else{
                    Case = 2;
                }

            } else {
                if(x < 0){
                    Case = 6;
                }else{
                    Case = 5;
                }

            }
        } else {
            if (x > 0) {
                if(y < 0){
                    Case = 4;
                }else{
                    Case = 3;
                }

            } else {
                if(y < 0){
                    Case = 7;
                }else{
                    Case = 8;
                }

            }
        }
        x = Math.abs(x);
        y = Math.abs(y);

        if(x > y){
            int placeholder = x;
            x = y;
            y = placeholder;
            swapped = true;
        }

        move(x, y);
    }

}




