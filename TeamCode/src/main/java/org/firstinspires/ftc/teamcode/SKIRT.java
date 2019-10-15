package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.*;

public class SKIRT {

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
    public static DcMotor rightFront;
    public static DcMotor leftFront;
    public static DcMotor rightBack;
    public static DcMotor leftBack;

    //importing other stuff

    //ticks per motor full rotation
    static final double MOTOR_TICK_COUNT = 537.6;

    //wheel diameter cm
    static final int WHEEL_DIAMETER = 10;

    //circumference of wheels in centimeters
    static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    //gear ratio wheel divided by motor
    static final double GEER_RATIO = 4/3;

    //robot readious, distance from center of a wheel to center of the robot
    static final double ROBOT_RADIOUS = 20;

    //turn factor is the amount of distance, along the circle of the robot, is made per full wheel rotation) divided by wheel circumfrance
    static final double TURN_FACTOR = 10 / WHEEL_CIRCUMFRENCE;

    //movement variables for move

    double correcction;

    int distanceLF = 0;
    int distanceLB = 0;
    int distanceRF = 0;
    int distanceRB = 0;

    double PowerLF = 0;
    double PowerLB = 0;
    double PowerRF = 0;
    double PowerRB = 0;

    double specialPower;
    int distanceToPoint;

    boolean PerfectStrafe = false;

    int quadrantSlope = 0;

    //movement variables for moveTurning

    double strafeFactor;




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

    public void move(int x, int y, double power) {
        // x and y are distances in centimeters

        //creating variables for distances each motor needs to travel and power levels and special stuff
        correcction = power / 6;

        //moves to desired x,y
        double slope;
        if (x == 0) {
            slope = 0;
        } else {
            slope = (double) y / (double) x;
        }

        if (-1 <= slope && slope <= 1) {
            if (y >= 0) {
                if (x < 0) {
                    Case = 1;
                } else {
                    Case = 2;
                }

            } else {
                if (x < 0) {
                    Case = 6;
                } else {
                    Case = 5;
                }

            }
        } else {
            if (x > 0) {
                if (y < 0) {
                    Case = 4;
                } else {
                    Case = 3;
                }

            } else {
                if (y < 0) {
                    Case = 7;
                } else {
                    Case = 8;
                }

            }
        }
        x = Math.abs(x);
        y = Math.abs(y);

        if (x > y) {
            int placeholder = x;
            x = y;
            y = placeholder;
            swapped = true;
        }


        //if no horizontal movement is necessary
        if (x == 0) {
            distanceLF = y;
            distanceLB = y;
            distanceRF = y;
            distanceRB = y;

            PowerLB = power;
            PowerLF = power;
            PowerRB = power;
            PowerRF = power;

        } else {
            // currently wronmg needs to be fixed
            specialPower = Math.toDegrees(Math.atan(Math.abs((double) y / (double) x)));
            //calculates how much each motor needs to move
            distanceToPoint = (int) (Math.sqrt((x * x) + (y * y)));

            if (specialPower == 45) {
                specialPower = 0;
                PerfectStrafe = true;
            } else if (specialPower > 45) {
                specialPower = specialPower / 45 - 1;
            } else {
                specialPower = -specialPower / 45 + 1;
            }

            distanceToPoint = (int) (distanceToPoint + distanceToPoint * 3 * (1 - specialPower));


            if (swapped) {
                int placeholder = x;
                x = y;
                y = placeholder;
                swapped = false;
            }
            switch (Case){

                case 1:
                //motor power levels
                PowerRF = power;
                PowerLB = power;
                PowerLF = specialPower * power;
                PowerRB = specialPower * power;

                quadrantSlope = -1;

                break;

                case 2:

                //motor power levels
                PowerLF = power;
                PowerRB = power;
                PowerRF = specialPower * power;
                PowerLB = specialPower * power;

                quadrantSlope = 1;

                break;

                case 3:

                //motor power levels
                PowerLF = power;
                PowerRB = power;
                PowerRF = -specialPower * power;
                PowerLB = -specialPower * power;

                quadrantSlope = 1;

                break;

                case 4:

                //motor power levels
                PowerRF = -power;
                PowerLB = -power;
                PowerLF = specialPower * power;
                PowerRB = specialPower * power;

                quadrantSlope = -1;

                break;

                case 5:

                //motor power levels
                PowerRF = -power;
                PowerLB = -power;
                PowerLF = -specialPower * power;
                PowerRB = -specialPower * power;

                quadrantSlope = -1;

                break;

                case 6:
                //motor power levels
                PowerLF = -power;
                PowerRB = -power;
                PowerRF = -specialPower * power;
                PowerLB = -specialPower * power;

                quadrantSlope = 1;

                break;

                case 7:

                //motor power levels
                PowerLF = -power;
                PowerRB = -power;
                PowerRF = specialPower * power;
                PowerLB = specialPower * power;

                quadrantSlope = 1;

                break;

                case 8:

                //motor power levels
                PowerRF = power;
                PowerLB = power;
                PowerLF = -specialPower * power;
                PowerRB = -specialPower * power;

                quadrantSlope = -1;

            }

        }


        //calculates number of tics necessary and tells motors to go that many
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(PowerLF);
        rightFront.setPower(PowerRF);
        rightBack.setPower(PowerRB);
        leftBack.setPower(PowerLB);


            if (PerfectStrafe) {

                if (quadrantSlope == 1) {

                    distanceRB = distanceToPoint;
                    distanceLF = distanceToPoint;

                    while (leftFront.getCurrentPosition() < distanceToTics(distanceLF) || rightBack.getCurrentPosition() < distanceToTics(distanceRB)) {

                        if (leftFront.getCurrentPosition() == rightBack.getCurrentPosition()) {

                        } else if (leftFront.getCurrentPosition() < rightBack.getCurrentPosition()) {
                            leftFront.setPower(PowerLF + correcction);
                            PowerLF = PowerLF + correcction;
                        } else {
                            leftFront.setPower(PowerRB + correcction);
                            PowerRB = PowerRB + correcction;
                        }
                    }
                } else {

                    distanceRF = distanceToPoint;
                    distanceLB = distanceToPoint;

                    while (rightFront.getCurrentPosition() < distanceToTics(distanceRF) || leftBack.getCurrentPosition() < distanceToTics(distanceLB)) {
                        if (rightFront.getCurrentPosition() == leftBack.getCurrentPosition()) {

                        } else if (rightFront.getCurrentPosition() < leftBack.getCurrentPosition()) {
                            rightFront.setPower(PowerRF + correcction);
                            PowerRF = PowerRF + correcction;
                        } else {
                            leftBack.setPower(PowerLB + correcction);
                            PowerLB = PowerLB + correcction;
                        }
                    }
                }
            } else {

                if(quadrantSlope == 1){
                    //set motor diances
                    distanceRB = distanceToPoint;
                    distanceLF = distanceToPoint;
                    distanceLB = (int) (distanceToPoint * specialPower);
                    distanceRF = (int) (distanceToPoint * specialPower);
                }else{
                    //set motor diances
                    distanceRF = distanceToPoint;
                    distanceLB = distanceToPoint;
                    distanceLF = (int) (distanceToPoint * specialPower);
                    distanceRB = (int) (distanceToPoint * specialPower);
                }

                while (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() < distanceToTics(distanceRF + distanceRB) || leftBack.getCurrentPosition() + leftFront.getCurrentPosition() < distanceToTics(distanceLB + distanceLF)) {

                    if (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() == leftBack.getCurrentPosition() + leftFront.getCurrentPosition()) {

                    } else if (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() < leftBack.getCurrentPosition() + leftFront.getCurrentPosition()) {
                        rightFront.setPower(PowerRF + correcction);
                        rightBack.setPower(PowerRB + correcction);

                        PowerRF = PowerRF + correcction;
                        PowerRB = PowerRB + correcction;
                    } else {
                        leftBack.setPower(PowerLB + correcction);
                        leftFront.setPower(PowerLF + correcction);

                        PowerLB = PowerLB + correcction;
                        PowerLF = PowerLF + correcction;
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
        }

        //tells us new location
        robotX += x;
        robotY += y;


    }

    public void move( int x, int y, double power, int degrees){

        distanceToPoint = (int) (Math.sqrt((x * x) + (y * y)));

        strafeFactor = 28424.460675 * ROBOT_RADIOUS * degrees / (28424.460675 * ROBOT_RADIOUS * degrees + 1628601.631621 * distanceToPoint);

        // x and y are distances in centimeters

        //creating variables for distances each motor needs to travel and power levels and special stuff
        correcction = power / 6;

        //moves to desired x,y
        double slope;
        if (x == 0) {
            slope = 0;
        } else {
            slope = (double) y / (double) x;
        }

        if (-1 <= slope && slope <= 1) {
            if (y >= 0) {
                if (x < 0) {
                    Case = 1;
                } else {
                    Case = 2;
                }

            } else {
                if (x < 0) {
                    Case = 6;
                } else {
                    Case = 5;
                }

            }
        } else {
            if (x > 0) {
                if (y < 0) {
                    Case = 4;
                } else {
                    Case = 3;
                }

            } else {
                if (y < 0) {
                    Case = 7;
                } else {
                    Case = 8;
                }

            }
        }
        x = Math.abs(x);
        y = Math.abs(y);

        if (x > y) {
            int placeholder = x;
            x = y;
            y = placeholder;
            swapped = true;
        }


        //if no horizontal movement is necessary
        if (x == 0) {
            distanceLF = y;
            distanceLB = y;
            distanceRF = y;
            distanceRB = y;

            PowerLB = power;
            PowerLF = power;
            PowerRB = power;
            PowerRF = power;

        } else {
            // currently wronmg needs to be fixed
            specialPower = Math.toDegrees(Math.atan(Math.abs((double) y / (double) x)));
            //calculates how much each motor needs to move
            distanceToPoint = (int) (Math.sqrt((x * x) + (y * y)));

            if (specialPower == 45) {
                specialPower = 0;
                PerfectStrafe = true;
            } else if (specialPower > 45) {
                specialPower = specialPower / 45 - 1;
            } else {
                specialPower = -specialPower / 45 + 1;
            }

            distanceToPoint = (int) (distanceToPoint + distanceToPoint * 3 * (1 - specialPower));


            if (swapped) {
                int placeholder = x;
                x = y;
                y = placeholder;
                swapped = false;
            }
            switch (Case){

                case 1:
                    //motor power levels
                    PowerRF = power - (int) (strafeFactor);
                    PowerLB = power + (int) (strafeFactor);
                    PowerLF = specialPower * power + (int) (strafeFactor);
                    PowerRB = specialPower * power - (int) (strafeFactor);

                    quadrantSlope = -1;

                    break;

                case 2:

                    //motor power levels
                    PowerLF = power + (int) (strafeFactor);
                    PowerRB = power - (int) (strafeFactor);
                    PowerRF = specialPower * power - (int) (strafeFactor);
                    PowerLB = specialPower * power + (int) (strafeFactor);

                    quadrantSlope = 1;

                    break;

                case 3:

                    //motor power levels
                    PowerLF = power + (int) (strafeFactor);
                    PowerRB = power - (int) (strafeFactor);
                    PowerRF = -specialPower * power + (int) (strafeFactor);
                    PowerLB = -specialPower * power - (int) (strafeFactor);

                    quadrantSlope = 1;

                    break;

                case 4:

                    //motor power levels
                    PowerRF = -power + (int) (strafeFactor);
                    PowerLB = -power - (int) (strafeFactor);
                    PowerLF = specialPower * power + (int) (strafeFactor);
                    PowerRB = specialPower * power - (int) (strafeFactor);

                    quadrantSlope = -1;

                    break;

                case 5:

                    //motor power levels
                    PowerRF = -power + (int) (strafeFactor);
                    PowerLB = -power - (int) (strafeFactor);
                    PowerLF = -specialPower * power - (int) (strafeFactor);
                    PowerRB = -specialPower * power + (int) (strafeFactor);

                    quadrantSlope = -1;

                    break;

                case 6:
                    //motor power levels
                    PowerLF = -power - (int) (strafeFactor);
                    PowerRB = -power + (int) (strafeFactor);
                    PowerRF = -specialPower * power + (int) (strafeFactor);
                    PowerLB = -specialPower * power - (int) (strafeFactor);

                    quadrantSlope = 1;

                    break;

                case 7:

                    //motor power levels
                    PowerLF = -power - (int) (strafeFactor);
                    PowerRB = -power + (int) (strafeFactor);
                    PowerRF = specialPower * power - (int) (strafeFactor);
                    PowerLB = specialPower * power + (int) (strafeFactor);

                    quadrantSlope = 1;

                    break;

                case 8:

                    //motor power levels
                    PowerRF = power - (int) (strafeFactor);
                    PowerLB = power + (int) (strafeFactor);
                    PowerLF = -specialPower * power - (int) (strafeFactor);
                    PowerRB = -specialPower * power + (int) (strafeFactor);

                    quadrantSlope = -1;

            }

        }


        //calculates number of tics necessary and tells motors to go that many
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(PowerLF);
        rightFront.setPower(PowerRF);
        rightBack.setPower(PowerRB);
        leftBack.setPower(PowerLB);


        if (PerfectStrafe) {

            if (quadrantSlope == 1) {

                distanceRB = distanceToPoint - (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceLF = distanceToPoint + (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);

                while (leftFront.getCurrentPosition() < distanceToTics(distanceLF) || rightBack.getCurrentPosition() < distanceToTics(distanceRB)) {

                    if (leftFront.getCurrentPosition() == rightBack.getCurrentPosition()) {

                    } else if (leftFront.getCurrentPosition() < rightBack.getCurrentPosition()) {
                        leftFront.setPower(PowerLF + correcction);
                        PowerLF = PowerLF + correcction;
                    } else {
                        leftFront.setPower(PowerRB + correcction);
                        PowerRB = PowerRB + correcction;
                    }
                }
            } else {

                distanceRF = distanceToPoint - (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceLB = distanceToPoint + (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);

                while (rightFront.getCurrentPosition() < distanceToTics(distanceRF) || leftBack.getCurrentPosition() < distanceToTics(distanceLB)) {
                    if (rightFront.getCurrentPosition() == leftBack.getCurrentPosition()) {

                    } else if (rightFront.getCurrentPosition() < leftBack.getCurrentPosition()) {
                        rightFront.setPower(PowerRF + correcction);
                        PowerRF = PowerRF + correcction;
                    } else {
                        leftBack.setPower(PowerLB + correcction);
                        PowerLB = PowerLB + correcction;
                    }
                }
            }
        } else {

            if(quadrantSlope == 1){
                //set motor diances
                distanceRB = distanceToPoint - (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceLF = distanceToPoint + (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceLB = (int) (distanceToPoint * specialPower) + (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceRF = (int) (distanceToPoint * specialPower) - (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
            }else{
                //set motor diances
                distanceRF = distanceToPoint - (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceLB = distanceToPoint + (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceLF = (int) (distanceToPoint * specialPower) + (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
                distanceRB = (int) (distanceToPoint * specialPower) - (int) (strafeFactor * WHEEL_CIRCUMFRENCE * TURN_FACTOR);
            }

            while (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() < distanceToTics(distanceRF + distanceRB) || leftBack.getCurrentPosition() + leftFront.getCurrentPosition() < distanceToTics(distanceLB + distanceLF)) {

                if (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() == leftBack.getCurrentPosition() + leftFront.getCurrentPosition()) {

                } else if (rightFront.getCurrentPosition() + rightBack.getCurrentPosition() < leftBack.getCurrentPosition() + leftFront.getCurrentPosition()) {
                    rightFront.setPower(PowerRF + correcction);
                    rightBack.setPower(PowerRB + correcction);

                    PowerRF = PowerRF + correcction;
                    PowerRB = PowerRB + correcction;
                } else {
                    leftBack.setPower(PowerLB + correcction);
                    leftFront.setPower(PowerLF + correcction);

                    PowerLB = PowerLB + correcction;
                    PowerLF = PowerLF + correcction;
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
        }

        //tells us new location
        robotX += x;
        robotY += y;

    }

}
