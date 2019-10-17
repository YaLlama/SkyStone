package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SKRTOdometry {

    //VARIABLES
    //
    //
    //
    //

    //location ,location, location

    // case for movement command
    public int Case = 0;

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

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odo = new Odometer(rightFront, leftFront, rightBack, -1, -1, 1);
        odo.initializeOdometry();

    }


    public void move(int x, int y, double power) {
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

                if(DisY < 0){
                    specialPower = -specialPower;
                }
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

        //Resets
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
}
