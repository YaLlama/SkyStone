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


    //SHORTCUTS
    //
    //
    //
    //

    public int distanceToTics(double d) {
        //d is centimers want to travel
        return (int) (d / WHEEL_CIRCUMFRENCE * MOTOR_TICK_COUNT);
    }

    public void runEncoders(DcMotor m) {
        //runs motors with encoders
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void resetEncoders(DcMotor m) {
        //resets encoder value
        m.setMode(DcMotor.RunMode.RESET_ENCODERS);
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

        motorFrontLeft.setTargetPosition((int) ditsagacne * 2);
        motorFrontRight.setTargetPosition((int) ditsagacne * 2);
        motorBackRight.setTargetPosition((int) ditsagacne * 2);
        motorBackLeft.setTargetPosition((int) ditsagacne * 2);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);

        while (motorsRunning()) {

        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        resetEncoders(motorFrontRight);
        resetEncoders(motorFrontLeft);
        resetEncoders(motorBackRight);
        resetEncoders(motorBackLeft);


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
        resetEncoders(motorFrontRight);
        resetEncoders(motorFrontLeft);
        resetEncoders(motorBackRight);
        resetEncoders(motorBackLeft);

        //setting max speed
        //
        //
        //
    }

    public void testEncoders() {
        //print out encoder value for set time
        int i = 0;
        while (i < 10000000) {
            System.out.println(motorBackLeft.getCurrentPosition() + " " + motorBackRight.getCurrentPosition() + " " + motorFrontLeft.getCurrentPosition() + " " + motorFrontRight.getCurrentPosition());
            i++;
        }
        resetEncoders(motorFrontRight);
        resetEncoders(motorFrontLeft);
        resetEncoders(motorBackRight);
        resetEncoders(motorBackLeft);
    }

    public void move(int x, int y) {
        // x and y are distances in centimeters


        //creating variables for distances each motor needs to travel and power levels and special stuff
        int distanceLF = 0;
        int distanceLB = 0;
        int distanceRF = 0;
        int distanceRB = 0;

        double PowerLF = 0;
        double PowerLB = 0;
        double PowerRF = 0;
        double PowerRB = 0;

        double specialPower = 0;
        int Specialdistanjce = 0;


        //if no horizontal movement is necessary
        if (x == 0) {
            distanceLF = 2 * y;
            distanceLB = 2 * y;
            distanceRF = 2 * y;
            distanceRB = 2 * y;

            PowerLB = 1;
            PowerLF = 1;
            PowerRB = 1;
            PowerRF = 1;

        } else {
            //calculates how much each motor needs to move
            if (x < 0) {
                //Special distnaces too fking hours thx eddie
                specialPower = (2 / ((x - y) / x)) + 1;
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
                specialPower = -(2 / ((x + y) / x)) + 1;
                Specialdistanjce = (int) Math.sqrt(x ^ 2 + (y ^ 2));

                //motor power levels
                PowerRB = 1;
                PowerLF = 1;
                PowerLB = specialPower;
                PowerRF = specialPower;

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
        resetEncoders(motorFrontRight);
        resetEncoders(motorFrontLeft);
        resetEncoders(motorBackRight);
        resetEncoders(motorBackLeft);


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

