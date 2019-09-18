package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;





@Autonomous(name="Autonymus")
public class Autonymus extends LinearOpMode {
    //VARIABLES
    //
    //
    //
    //

    //location ,location, location
    //robot staring location
    int robotX = 100;
    int robotY = 0;
    //robot direction facing
    int direction = 0;

    //declaring motors
    DcMotor motorFrontRight = null;
    DcMotor motorFrontLeft = null;
    DcMotor motorBackRight = null;
    DcMotor motorBackLeft = null;

    //ticks per revolution
    static final double MOTOR_TICK_COUNT = 537.6;

    //wheel diameter
    static final int WHEEL_DIAMETER = 8;

    //circumfrance of wheels in centimeters
    static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;

    //Bot radious as in distance from center point to wheels
    static final int ROBOT_RADIOUS = 16;

    //Power limit
    static final int POWER_FACTOR = 1;

    //number of rotations of wheels for the robot to move the circumfarance of the wheel 45degrees
    static final int STRAFE_FACTOR = 2;


    //SHORTCUTS
    //
    //
    //
    //

    public int distanceToTics(double d) {
        //d is centimers want to travel
        return (int) (d / WHEEL_CIRCUMFRENCE * MOTOR_TICK_COUNT);
    }


    public boolean motorsRunning() {
        if (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    public void rotate(int degrees) {
        //rotates robot specific degrees requested

        double ditsagacne = ROBOT_RADIOUS * 2 * Math.PI * degrees / 360;

        motorFrontLeft.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);
        motorFrontRight.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);
        motorBackRight.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);
        motorBackLeft.setTargetPosition((int) ditsagacne * STRAFE_FACTOR);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(POWER_FACTOR);
        motorFrontRight.setPower(POWER_FACTOR);
        motorBackRight.setPower(POWER_FACTOR);
        motorBackLeft.setPower(POWER_FACTOR);

        while (motorsRunning()) {

        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        direction = direction + degrees;
        while (direction < 0 || 360 < direction) {
            if (direction < 0) {
                direction += 360;
            } else {
                direction -= 360;
            }
        }
    }

    public void rotateTo(int degrees) {
        rotate(degrees - direction);
    }


    public void initalize() {
        //declaring motors
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);

        //setting motors to fun with encoders
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void testEncoders() {
        //print out encoder value for set time
        //tell motors to float
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        for (int i=0; i < 10000000; i++) {
            System.out.println(motorBackLeft.getCurrentPosition() + " "
                    + motorBackRight.getCurrentPosition() + " "
                    + motorFrontLeft.getCurrentPosition() + " "
                    + motorFrontRight.getCurrentPosition());
        }
        //lock motos
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            if (x < 0) {
                //Special distnaces too fking hours thx eddie
                specialPower = (STRAFE_FACTOR / ((x - y) / x)) + 1;
                Specialdistanjce = (int) Math.sqrt(x ^ 2 + (y ^ 2));

                //motor power levels
                PowerRF = 1;
                PowerLB = 1;
                PowerLF = specialPower;
                PowerRB = specialPower;

                //set motor diances
                distanceRF = Specialdistanjce;
                distanceLB = Specialdistanjce;
                distanceLF = (int) (Specialdistanjce * specialPower);
                distanceRB = (int) (Specialdistanjce * specialPower);

            } else {
                //Special distnaces too fking hours thx eddie
                specialPower = -(STRAFE_FACTOR / ((x + y) / x)) + 1;
                Specialdistanjce = (int) Math.sqrt(x ^ 2 + (y ^ 2));

                //motor power levels
                PowerRB = POWER_FACTOR;
                PowerLF = POWER_FACTOR;
                PowerLB = specialPower * POWER_FACTOR;
                PowerRF = specialPower * POWER_FACTOR;

                //set motor diances
                distanceRB = Specialdistanjce;
                distanceLF = Specialdistanjce;
                distanceLB = (int) (Specialdistanjce * specialPower);
                distanceRF = (int) (Specialdistanjce * specialPower);

            }
        }
        //calculates number of tics necessary and tells motors to go that many
        motorFrontLeft.setTargetPosition(distanceToTics(distanceLF));
        motorFrontRight.setTargetPosition(distanceToTics(distanceRF));
        motorBackRight.setTargetPosition(distanceToTics(distanceRB));
        motorBackLeft.setTargetPosition(distanceToTics(distanceLB));

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(PowerLF);
        motorFrontRight.setPower(PowerRF);
        motorBackRight.setPower(PowerRB);
        motorBackLeft.setPower(PowerLB);

        while (motorsRunning()) {

        }

        //stops everything
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        //Resets
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //tells us new location
        robotX += x;
        robotY += y;


    }

    public void moveTo(int x, int y) {
        //moves to desired x,y
        double slope = y / x;
        if (-1 <= slope && slope < 1) {
            if (y >= 0) {
                rotateTo(0);
                move(x, y);
            } else {
                rotateTo(180);
                move(-x, -y);
            }
        } else {
            if (x > 0) {
                rotateTo(270);
                move(x, -y);
            } else {
                rotateTo(90);
                move(-x, y);
            }
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initalize();
        testEncoders();
        moveTo(27, 7);
        waitForStart();
    }

}

