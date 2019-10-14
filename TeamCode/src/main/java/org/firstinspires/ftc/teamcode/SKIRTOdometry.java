package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Odometer;

public class SKIRTOdometry {

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
    public static DcMotor rightFrontEncoder;
    public static DcMotor leftFrontEncoder;
    public static DcMotor rightBackEncoder;
    public static DcMotor leftBack;

    //odomentry srtuff
    private final double omniRadius = 1.85; //Radius of Omni wheels
    private final double gearing = 1.5; //How many times does the Omni spin for each spin of the encoder
    private final double robotRadius = 9.5;
    private final double distanceBack = 31;

    private Odometer Adham;


    //declaring odometry
    private DcMotor EncoderRight;
    private DcMotor EncoderLeft;
    private DcMotor EncoderBack;

    //importing other stuff

    //ticks per motor full rotation
    static final double MOTOR_TICK_COUNT = 537.6;

    //wheel diameter cm
    static final int WHEEL_DIAMETER = 10;

    //circumference of wheels in centimeters
    static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    //Power limit
    public static double POWER_FACTOR;

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

    public void initalize(DcMotor RFE, DcMotor RBE, DcMotor LFE, DcMotor LB) {
        //declaring motors
        rightFrontEncoder = RFE;
        leftFrontEncoder = LFE;
        rightBackEncoder = RBE;
        leftBack = LB;

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBackEncoder.setDirection(DcMotor.Direction.REVERSE);
        leftFrontEncoder.setDirection(DcMotor.Direction.FORWARD);
        rightFrontEncoder.setDirection(DcMotor.Direction.REVERSE);

        leftFrontEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting motors to fun with encoders
        leftFrontEncoder.setPower(0);
        rightFrontEncoder.setPower(0);
        rightBackEncoder.setPower(0);
        leftBack.setPower(0);

        leftFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Adham = new Odometer(rightFrontEncoder, leftFrontEncoder, rightBackEncoder, robotRadius, distanceBack, omniRadius, gearing);
        Adham.initializeOdometry();

    }

    public void testMotors(){


        leftFrontEncoder.setPower(.2);
        while(leftFrontEncoder.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        rightFrontEncoder.setPower(.2);
        while(rightFrontEncoder.isBusy()){
            try{
                Thread.sleep(80);
            }catch(Exception e){
                System.out.print(e);
            }
        }
        rightBackEncoder.setPower(.2);
        while(rightBackEncoder.isBusy()){
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

        leftFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void move(int x, int y, double power) {
        // x and y are distances in centimeters


        //creating variables for distances each motor needs to travel and power levels and special stuff
        double correcction = power / 6;
        POWER_FACTOR = power;

        double PowerLF;
        double PowerLB;
        double PowerRF;
        double PowerRB;

        double specialPower;

        boolean PerfectStrafe = false;

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

            PowerLB = power;
            PowerLF = power;
            PowerRB = power;
            PowerRF = power;

        } else {
            // currently wronmg needs to be fixed
            specialPower = Math.toDegrees(Math.atan(Math.abs((double) y / (double) x)));
            //calculates how much each motor needs to move


            if (specialPower == 45) {
                specialPower = 0;
                PerfectStrafe = true;
            } else if (specialPower > 45) {
                specialPower = specialPower / 45 - 1;
            } else {
                specialPower = -specialPower / 45 + 1;
            }


            if (swapped) {
                int placeholder = x;
                x = y;
                y = placeholder;
                swapped = false;
            }

            if (Case == 1) {
                //motor power levels
                PowerRF = power;
                PowerLB = power;
                PowerLF = -specialPower * power;
                PowerRB = -specialPower * power;


            } else if (Case == 2) {
                //motor power levels
                PowerLF = power;
                PowerRB = power;
                PowerRF = specialPower * power;
                PowerLB = specialPower * power;



            } else if (Case == 3) {
                //motor power levels
                PowerLF = power;
                PowerRB = power;
                PowerRF = -specialPower * power;
                PowerLB = -specialPower * power;

            } else if (Case == 4) {
                //motor power levels
                PowerRF = -power;
                PowerLB = -power;
                PowerLF = -specialPower * power;
                PowerRB = -specialPower * power;


            } else if (Case == 5) {
                //motor power levels
                PowerRF = -power;
                PowerLB = -power;
                PowerLF = specialPower * power;
                PowerRB = specialPower * power;


            } else if (Case == 6) {
                //motor power levels
                PowerLF = -power;
                PowerRB = -power;
                PowerRF = -specialPower * power;
                PowerLB = -specialPower * power;


            } else if (Case == 7) {
                //motor power levels
                PowerLF = -power;
                PowerRB = -power;
                PowerRF = specialPower * power;
                PowerLB = specialPower * power;

            } else {
                //motor power levels
                PowerRF = power;
                PowerLB = power;
                PowerLF = specialPower * power;
                PowerRB = specialPower * power;

            }

        }


        //calculates number of tics necessary and tells motors to go that many
        leftFrontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontEncoder.setPower(PowerLF);
        rightFrontEncoder.setPower(PowerRF);
        rightBackEncoder.setPower(PowerRB);
        leftBack.setPower(PowerLB);

            if (PerfectStrafe) {

                if (Case == 2 || Case == 7 || Case == 3 || Case == 6) {

                    while (Adham.getposition()[0] < x || Adham.getposition()[1] < y) {

                        Adham.updateOdometry();

                    }
                } else {
                    while (Adham.getposition()[0] < x || Adham.getposition()[1] < y) {

                        Adham.updateOdometry();

                    }
                }
            } else {
                while (Adham.getposition()[0] < x || Adham.getposition()[1] < y) {

                    Adham.updateOdometry();

                }


            //stops everything
            leftFrontEncoder.setPower(0);
            rightFrontEncoder.setPower(0);
            rightBackEncoder.setPower(0);
            leftBack.setPower(0);

            //Resets
            leftFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //tells us new location
        robotX += x;
        robotY += y;


    }


}
