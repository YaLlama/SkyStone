package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class AutoMovementKaden {

    //opmode
    LinearOpMode opMode;

    //Constants
    double ODOMETRY_POD_GEAR_RATIO = 1; //gearspokes on wheel /(div) geer spokes on encoder
    double ODOMETRY_WHEEL_DIAMETER = 6; //in desired units
    double ODOMETRY_ENCODER_TICS_PER_ROTATION = 8192;

    //Motors for drivetrain
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;

    //OdometryPod motor ports
    DcMotor odometryRight;
    DcMotor odoemtryLeft;
    DcMotor odometryCenter;

    //The imu
    BNO055IMU imu;

    //Number raw odometry inputs need to be multiplied by to get desired direction and scale
    double odoPodRightDirectionalScale = -Math.PI * ODOMETRY_WHEEL_DIAMETER * ODOMETRY_POD_GEAR_RATIO / ODOMETRY_ENCODER_TICS_PER_ROTATION;
    double odoPodLeftDirectionalScale = Math.PI * ODOMETRY_WHEEL_DIAMETER * ODOMETRY_POD_GEAR_RATIO / ODOMETRY_ENCODER_TICS_PER_ROTATION;
    double odoPodCenterDirectionalScale = Math.PI * ODOMETRY_WHEEL_DIAMETER * ODOMETRY_POD_GEAR_RATIO / ODOMETRY_ENCODER_TICS_PER_ROTATION;


    public AutoMovementKaden(DcMotor MotorRightFront, DcMotor MotorLeftFront, DcMotor MotorRightBack, DcMotor MotorLeftBack, DcMotor OdometryRight, boolean RightOdoReversed, DcMotor OdoemtryLeft, boolean LeftOdoReversed, DcMotor OdometryCenter, boolean CenterOdoReversed, BNO055IMU IMU, LinearOpMode OpMode){
        //gertting Opmode
        opMode = OpMode;

        //getting drivetrain motors
        motorLeftBack = MotorLeftBack;
        motorLeftFront = MotorLeftFront;
        motorRightBack = MotorRightBack;
        motorRightFront = MotorRightFront;

        //setting drive motor direction
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //setting 0 power behavior
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Getting odoetry pods
        odoemtryLeft = OdoemtryLeft;
        odometryRight = OdometryRight;
        odometryCenter = OdometryCenter;

        //setting motor modes
        odoemtryLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //getting the IMU
        imu = IMU;

        //dealing with if odometry is reversed
        if(RightOdoReversed){
            odoPodRightDirectionalScale *= -1;
        }
        if(LeftOdoReversed){
            odoPodLeftDirectionalScale *= -1;
        }
        if(CenterOdoReversed){
            odoPodCenterDirectionalScale *= -1;
        }
    }

    private double getImuHeading() {
        //may need to change axis unit to work with vertical hubs -- depending on how u orient hubs, axis may have to be different.
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double d = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        //d += headingOffset;
        return (d+360) % 360;
    }

    public void StrafeToPointOrient(double DesiredX, double DesiredY, double DesiredHeading, double XYThreshold, double HeadingThreshold, double Power){

        //creating variablkes for possition
        double currentX = odometryCenter.getCurrentPosition() * odoPodCenterDirectionalScale;
        double currentY = (odoemtryLeft.getCurrentPosition() * odoPodLeftDirectionalScale) + (odometryRight.getCurrentPosition() * odoPodRightDirectionalScale);
        double currentHeading = getImuHeading();

        //creating seperate power variable that can be changed without affecting other programs
        double powerDirection;

        //variable used for calculating motor powers
        double pathSlope;

        //special calculated power to move the boit on the correct slope
        double fractionalPower;

        while((Math.abs(DesiredHeading - currentHeading) > HeadingThreshold || Math.abs(DesiredX - currentX) > XYThreshold || Math.abs(DesiredY - currentY) > XYThreshold) && opMode.opModeIsActive()){ //keep moving untiol in desired position or program is stopped

            //updating possition
            currentX = odometryCenter.getCurrentPosition() * odoPodCenterDirectionalScale;
            currentY = (odoemtryLeft.getCurrentPosition() * odoPodLeftDirectionalScale) + (odometryRight.getCurrentPosition() * odoPodRightDirectionalScale);
            currentHeading = getImuHeading();

            while(Math.abs(DesiredHeading - currentHeading) > HeadingThreshold){ //while not at desired heading

                if((int)(DesiredHeading - currentHeading) % 360 < 180){ //if we want to rotate counter clockwise
                    //rotate counter clockwise
                    motorLeftBack.setPower(-Power);
                    motorLeftFront.setPower(-Power);
                    motorRightBack.setPower(Power);
                    motorRightFront.setPower(Power);

                }else{ // if we want to rotate clockwise
                    //rotate clockwise
                    motorLeftBack.setPower(Power);
                    motorLeftFront.setPower(Power);
                    motorRightBack.setPower(-Power);
                    motorRightFront.setPower(-Power);

                }
                //reset odometry t
                odoemtryLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odometryRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odometryCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                odoemtryLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                odometryRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                odometryCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            while(Math.abs(DesiredX - currentX) > XYThreshold || Math.abs(DesiredY - currentY) > XYThreshold){ //while not at desired x or y

                if(DesiredY - currentY > 0){ // if the desired point is infront of the robot
                    powerDirection = Power;
                }else{ //if the desired point is behind the robot
                    powerDirection = -Power;
                }

                if (currentX - DesiredX == 0) { //to make sure you dont devide by 0 / are moving strait foward
                    motorLeftBack.setPower(powerDirection);
                    motorLeftFront.setPower(powerDirection);
                    motorRightBack.setPower(powerDirection);
                    motorRightFront.setPower(powerDirection);
                } else { // when you dont devide by 0 / not moving strait foward

                    //slope of the desiured path for the robotr
                    pathSlope = (DesiredY - currentY) / (DesiredX - currentX);

                    //fraction of motor power needed to be given to seccondzry motors to strafe at correct slope
                    fractionalPower = Math.abs(Math.toDegrees(Math.atan(pathSlope))) / 45 - 1;

                    if(pathSlope > 0){ //if the robot is traveling on a positive slope
                        //setting power to motors to move us on coirrect slope
                        motorLeftBack.setPower(powerDirection * fractionalPower);
                        motorLeftFront.setPower(powerDirection);
                        motorRightBack.setPower(powerDirection);
                        motorRightFront.setPower(powerDirection * fractionalPower);

                    }else{ //if robot is moving on  a negative slope
                        //setting power to motors to move us on coirrect slope
                        motorLeftBack.setPower(powerDirection);
                        motorLeftFront.setPower(powerDirection * fractionalPower);
                        motorRightBack.setPower(powerDirection * fractionalPower);
                        motorRightFront.setPower(powerDirection);
                    }

                }
            }
        }

        //stops everything when reached desired possition
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
    }
}
