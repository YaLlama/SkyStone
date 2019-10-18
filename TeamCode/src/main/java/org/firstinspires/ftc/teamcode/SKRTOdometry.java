package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SKRTOdometry {

    //VARIABLES
    //
    //
    //
    //

    //declaring motors
    public static DcMotor rightFront;
    public static DcMotor leftFront;
    public static DcMotor rightBack;
    public static DcMotor leftBack;

    Odometer odo;


    double PowerLF = 0;
    double PowerLB = 0;
    double PowerRF = 0;
    double PowerRB = 0;

    double specialPower;

    double strafeFactor;

    //robot readious, distance from center of a wheel to center of the robot
    static final double ROBOT_RADIOUS = 20;




    //SHORTCUTS / COMMANDS
    //
    //
    //
    //


    public SKRTOdometry(DcMotor RFE, DcMotor RBE, DcMotor LFE, DcMotor LB) {
        //declaring motors
        rightFront = RFE;
        leftFront = LFE;
        rightBack = RBE;
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

        odo = new Odometer(rightFront, leftFront, rightBack, -1, -1, 1);
        odo.initializeOdometry();

    }


    public void moveTo(int x, int y, double power) {
        // x and y are distances in centimeters
        int DisX;
        int DisY;

        //moves to desired x,y
        double slope;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        do {
            DisX = x - (int)odo.getposition()[0];
            DisY = y - (int)odo.getposition()[1];

            odo.updateOdometry();

            //if no horizontal movement is necessary
            if(DisY < 0){
                power = -power;
            }
            if (DisX == 0) {
                PowerLB = power;
                PowerLF = power;
                PowerRB = power;
                PowerRF = power;
            } else {
                slope = (double) DisY / (double) DisX + Math.tan(odo.getHeadingRad());

                specialPower = Math.abs(Math.toDegrees(Math.atan((slope)))) / 45 - 1;

                if(slope > 0){
                    PowerRF = power * specialPower;
                    PowerLB = power * specialPower;
                    PowerRB = power;
                    PowerLF = power;
                }else{
                    PowerLF = power * specialPower;
                    PowerRB = power * specialPower;
                    PowerLB = power;
                    PowerRF = power;
                }

            }


            //calculates number of tics necessary and tells motors to go that many
            leftFront.setPower(PowerLF);
            rightFront.setPower(PowerRF);
            rightBack.setPower(PowerRB);
            leftBack.setPower(PowerLB);

        }while(DisX != (int)odo.getposition()[0] && DisY != (int)odo.getposition()[1]);

        while(odo.getHeadingRad() != 0){

            odo.updateOdometry();

            leftFront.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
            leftBack.setPower(power);
        }

        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);



    }
    public void moveTo(int x, int y, double power, double degrees){

        // x and y are distances in centimeters
        int DisX;
        int DisY;
        double DegD;

        //moves to desired x,y
        double slope;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        do {
            DisX = x - (int)odo.getposition()[0];
            DisY = y - (int)odo.getposition()[1];

            DegD = degrees - odo.getHeadingDeg();

            odo.updateOdometry();

            //if no horizontal movement is necessary
            if(DisY < 0){
                power = -power;
            }

            if (DisX == 0) {
                PowerLB = power + strafeFactor;
                PowerLF = power + strafeFactor;
                PowerRB = power - strafeFactor;
                PowerRF = power - strafeFactor;

            } else {
                slope = (double) DisY / (double) DisX + Math.tan(odo.getHeadingRad());

                specialPower = Math.abs(Math.toDegrees(Math.atan((slope)))) / 45 - 1;

                strafeFactor = 28424.460675 * ROBOT_RADIOUS * DegD / (28424.460675 * ROBOT_RADIOUS * DegD + 1628601.631621 * Math.sqrt(x^2 + y^2));

                if(slope > 0){
                    PowerRF = power * specialPower - strafeFactor;
                    PowerLB = power * specialPower + strafeFactor;
                    PowerRB = power - strafeFactor;
                    PowerLF = power + strafeFactor;
                }else{
                    PowerLF = power * specialPower + strafeFactor;
                    PowerRB = power * specialPower - strafeFactor;
                    PowerLB = power + strafeFactor;
                    PowerRF = power - strafeFactor;
                }

            }

            //calculates number of tics necessary and tells motors to go that many

            leftFront.setPower(PowerLF);
            rightFront.setPower(PowerRF);
            rightBack.setPower(PowerRB);
            leftBack.setPower(PowerLB);

        }while(DisX != (int)odo.getposition()[0] && DisY != (int)odo.getposition()[1]);

        //stops everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);


    }

}
