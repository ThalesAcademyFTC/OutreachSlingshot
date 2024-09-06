package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Zegloub {

    /**
     * DECLARE the HardwareMap object used for this library
     * A HardwareMap is a mapping of the different hardware components to the
     * name set in the configuration
     */
    private HardwareMap hwMap;

    LinearOpMode auton;

    /**
     * An ENUM is a custom variable type that we can define the options for
     * This variable can be declared as a type Drivetrain, and then can be set to one of the options
     * listed in the list inside {}
     */
    public enum Drivetrain {
        MECHANUM,
        MISTRO,
        TEST
    }

    /**
     * This ENUM lists the teams that you can be in FIRST Tech Challenge. Useful for simplifying TeleOp
     */
    public enum Team {
        RED,
        BLUE
    }

    /**
     * Here is an example of us DECLARING the Drivetrain variable type drive
     * This drive can be set to any values inside of the Drivetrain enum list above
     */
    private Drivetrain drive;

    /**
     * This is thetelem object that the library will be using to write to the driver hub
     *telem is how we output to the drivers while they are using the robot
     * Also can be used for us to debug
     */
    private Telemetry telem;

    // Now, below you can define any global variables that you want to use in this library file.

    public DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;



    public DcMotor[] allDriveMotors;

    public DcMotor armMotor, intakeMotor, motorSuspend, droneMotor;

    public Servo clawServoL, clawServoR, angleServo, hookServo;

    public CRServo revolveServo, intakeServo;
    private IMU imu;

    private IMU.Parameters parameters;

    public WebcamName webcamName;

    // Put CONSTANTS here

    static final double HOOK_DROP_POSITION = 0.45;

    static final double HOOK_UP_POSITION = 0.42;

    /**
     * Encoder ticks for an INCH of movement
     * A good way to get this value is to run a test auton that moves forward for 500 ticks and then measure the distance.
     */
    static final double INCH_TICKS = 40;

    /**
     * Encoder ticks for turning 90 degrees
     * May take some trial and error to get this value
     */
    static final double NINETY_DEGREE_TICKS = 500;


    /**
     * The CONSTRUCTOR for the library class. This constructor pulls the HardwareMap from the opmode
     * and runs the setupHardware function
     * @param opmode the opmode that is being used. NOTE: This only works for TELEOP opmodes, not AUTON
     */
    public Zegloub( OpMode opmode, Drivetrain drivetrain ) {

        //Set the hardwaremap for this library to the opmode's hardwareMap
        this.hwMap = opmode.hardwareMap;

        // Set the drive in the library to the opmode's selected drivetrain
        this.drive = drivetrain;

        this.telem = opmode.telemetry;

        // Run the setupHardware function to map variables to their hardware object
        setupHardware();

    }

    public Zegloub(LinearOpMode opmode, Drivetrain type) {

        //Set the auton opmode directly to the opmode object.
        //Objects can be passed in to functions, so this object will be updated live.
        this.auton = opmode;

        //Sets the hardwareMap
        hwMap = opmode.hardwareMap;

        //Sets thetelem
        telem = opmode.telemetry;

        //Sets the drivetrain
        drive = type;

        //Run setHardwareMap function to map variables to their hardware object.
        setupHardware();
    }


    RevBlinkinLedDriver lights;
    /**
     * This constructor is used for testing the hardwareMap and drivetrain
     * @param hardwareMap the hardwareMap being tested
     * @param drivetrain the drivetrain being tested
     */
    public Zegloub( HardwareMap hardwareMap, Drivetrain drivetrain ) {

        this.hwMap = hardwareMap;

        this.drive = drivetrain;

        setupHardware();
    }

    /**
     * This function maps the variables declared above to a specific hardware object,
     * as defined by the configuration on the driver hub
     * Note: This function can only be used inside the library, since it is private
     */
    private void setupHardware() {

        // This switch statement is used to choose which drivetrain to setup,
        // depending on the drive variable.
        switch ( drive ) {

            // If drive is MECHANUM, then everything in here will setup the MECHANUM hardware variables
            case MISTRO:

                //First, setup the motors that are used for the drivetrain
                motorFrontLeft = hwMap.dcMotor.get( "motorFrontLeft" );
                motorFrontRight = hwMap.dcMotor.get( "motorFrontRight" );
                motorBackLeft = hwMap.dcMotor.get( "motorBackLeft" );
                motorBackRight = hwMap.dcMotor.get( "motorBackRight" );

                //Next, reverse motors that need to spin the other direction
                // Tip: All motors should move the robot forward if set to power 1
                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

                //Here would go any additional hardware devices for the robot

                // Map the imu to the hardware device
                imu = hwMap.get( IMU.class, "imu" );

                //Set parameters for the imu.
                //Check the direction that the logo is facing.
                //Check the direction that the USB plugs are facing
                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)
                );

                imu.initialize( parameters );

                webcamName = hwMap.get(WebcamName.class, "Webcam 1");

                //Add arm mechanism hardware devices

                armMotor = hwMap.dcMotor.get( "armMotor" );

                // clawServo = hwMap.servo.get( "clawServo" );


                break;



            case MECHANUM:

                //First, setup the motors that are used for the drivetrain
                motorFrontLeft = hwMap.dcMotor.get( "motorFrontLeft" );
                motorFrontRight = hwMap.dcMotor.get( "motorFrontRight" );
                motorBackLeft = hwMap.dcMotor.get( "motorBackLeft" );
                motorBackRight = hwMap.dcMotor.get( "motorBackRight" );

                //Next, reverse motors that need to spin the other direction
                // Tip: All motors should move the robot forward if set to power 1
                motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

                //Lights :D
                lights = hwMap.get(RevBlinkinLedDriver.class, "lights");
                //Here would go any additional hardware devices for the robot

                // Map the imu to the hardware device
                imu = hwMap.get( IMU.class, "imu" );

                //Set parameters for the imu.
                //Check the direction that the logo is facing.
                //Check the direction that the USB plugs are facing
                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

                imu.initialize( parameters );

                //Camera setup
                webcamName = hwMap.get(WebcamName.class, "Webcam 1");

                //Add arm mechanism hardware devices


                armMotor = hwMap.dcMotor.get( "armMotor" );
                droneMotor = hwMap.dcMotor.get( "droneMotor" );
                //motorSuspend = hwMap.dcMotor.get( "motorSuspend" );
                //revolveServo = hwMap.crservo.get( "revolveServo" );
                //intakeServo = hwMap.crservo.get( "intakeServo" );
                //hookServo = hwMap.servo.get("hookServo");
                //ledControl = hwMap.servo.get( "ledControl");
                angleServo = hwMap.servo.get( "angleServo" );
                clawServoL = hwMap.servo.get( "clawServoL" );
                clawServoR = hwMap.servo.get( "clawServoR" );
                allDriveMotors = new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};
                //motorSuspend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                break;

            case TEST:

                //First, setup the motors that are used for the drivetrain
                motorFrontLeft = hwMap.dcMotor.get( "motorFrontLeft" );
                motorFrontRight = hwMap.dcMotor.get( "motorFrontRight" );
                motorBackLeft = hwMap.dcMotor.get( "motorBackLeft" );
                motorBackRight = hwMap.dcMotor.get( "motorBackRight" );

                //Next, reverse motors that need to spin the other direction
                // Tip: All motors should move the robot forward if set to power 1
                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


                //Here would go any additional hardware devices for the robot

                // Map the imu to the hardware device
                imu = hwMap.get( IMU.class, "imu" );

                //Set parameters for the imu.
                //Check the direction that the logo is facing.
                //Check the direction that the USB plugs are facing
                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)
                );

                imu.initialize( parameters );

                //camera setup!
                //webcamName = hwMap.get(WebcamName.class, "Webcam 1");
                allDriveMotors = new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};

                break;

            default:

                //Here would go what would happen if drive is not one of the options above.
                //Really, this just means you made a coding mistake. So nothing needs to go here.
                telem.addLine("Invalid type " + drive + " passed to Spark's init function. Nothing has been set up.");
                break;

        }

    }

    /**
     * Set motor power for all drivetrain motors on robot to 0
     */
    public void rest() {
        motorBackLeft.setPower( 0 );
        motorBackRight.setPower( 0 );
        motorFrontLeft.setPower( 0 );
        motorFrontRight.setPower( 0 );
    }




    /**
     * This function controls movement for the robot.
     * @param x the x speed value
     * @param y the y speed value
     * @param turn the turn speed value
     */
    public void move( double x, double y, double turn ) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]`
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

        // Save values for the power of each motor
        double frontLeftPower = ( y + x + turn ) / denominator;
        double backLeftPower = ( y - x + turn ) / denominator;
        double frontRightPower = ( y - x - turn ) / denominator;
        double backRightPower = ( y + x - turn ) / denominator;

        //Now, assign that motor power to each motor
        motorFrontLeft.setPower( frontLeftPower );
        motorBackLeft.setPower( backLeftPower );
        motorFrontRight.setPower( frontRightPower );
        motorBackRight.setPower( backRightPower );

    }

    public void moveLeft( double speed ){
        move( -speed, 0, 0 );
    }

    public void moveRight( double speed ){
        move( speed, 0, 0 );
    }

    public void moveForward( double speed ){
        move( 0, speed, 0 );
    }

    public void moveBackward( double speed ){
        move( 0, -speed, 0 );
    }

    public void turnLeft( double speed ){
        move( 0, 0, -speed );
    }

    public void turnRight( double speed ){
        move( 0, 0, speed );
    }

    /**
     * Resets the yaw of the IMU. This fixes any deviation that might occur, but assumes the robot is
     * back in starting orientation.
     */
    public void resetYaw() {
        imu.resetYaw();
    }

    /**
     * Gets the yaw heading in degrees
     * @return the yaw heading in degrees
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw( AngleUnit.DEGREES );
    }

    /*/**
     * Sets the arm motor to the given power
     * @param power the power to send to the arm motor
     */
    public void liftArm() {
        armMotor.setPower(0.75);
    }

    public void lowerArm() {
        armMotor.setPower(-0.5);
    }

    public void setArmMotor( double power ) {
        armMotor.setPower( power );
    }

    public void setDroneMotor( double power ) {
        droneMotor.setPower( power );
    }

    public void launchDrone() {
        droneMotor.setPower(-1);
    }

    /*public void setMotorSuspend( double power ) {
        motorSuspend.setPower( power );
    }

    public void setIntakeMotor (double power) {
        intakeMotor.setPower( power );
    }*/
    /*
    /**
     * Set the claw servo to the given position
     * @param position the position to set the claw servo to
     */

    //public void setClawServo( double position ) {
    //        clawServo.setPosition( position );
    //}

    public void tiltClaw() {
        angleServo.setPosition( 0.4 );
    }

    public void resetClaw() {
        angleServo.setPosition( 0.46 );
    }

    public void openClawL() {
        clawServoL.setPosition(0.6);
    }

    public void openClawR() {
        clawServoR.setPosition(0.3);
    }

    public void closeClawR() {
        clawServoR.setPosition(0.01);
    }

    public void closeClawL() {
        clawServoL.setPosition(.9);
    }


    public void setRevolvePower( double power) {
        revolveServo.setPower( power );
    }

    public void runIntake( double power ) {
        intakeServo.setPower(power);
    }

    public void setHookServo( double position) {
        hookServo.setPosition( position );
    }

    public void dropHook(){
        setHookServo(HOOK_DROP_POSITION);
    }

    public void liftHook(){
        setHookServo(HOOK_UP_POSITION);
    }

    public void pixelRelease( double power) {
        revolveServo.setPower(power);
    }

    /*public void turnRightDegrees( double degrees, double speed ) {

        //If this is turning in the wrong direction, swap the + to a - below
        double target = getHeading() + degrees;

        // Degree of error allowed for degrees. If set to 0.5, the robot could be 1 degree off.
        final double DEGREE_OF_ERROR = 0.5;

        // Set motors to turn right power
        move( 0, 0, speed );

        // Loop while opmode is active and heading is not within desired rate.
        while( auton.opModeIsActive() && !auton.isStopRequested()
                && (getHeading() >= target + DEGREE_OF_ERROR || getHeading() <= target + DEGREE_OF_ERROR ) ) {
            // Keep looping until it reaches near target
        }

        //Once it is near target, rest.
        rest();

    }

    public void turnLeftDegrees( double degrees, double speed ) {

        //haha, I am using another function so I don't have to rewrite the code.
        turnRightDegrees( -degrees, -speed );

    } */

    public void moveForwardInches( double inches, double speed ) {

        // Converts to integer by rounding. CASTS to int after rounding
        int tickTarget = (int)Math.round( inches * INCH_TICKS );


        for ( DcMotor x: allDriveMotors ) {

            x.setTargetPosition( tickTarget );
            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        // Move forward. Think of this like a coordinate plane :)
        move( 0, speed, 0 );

        for ( DcMotor x: allDriveMotors ) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }



        // Wait for motors to reach the desired tick level
        waitForMotors();

        // Reset motors for normal movement
        resetDriveEncoders();

    }

    public void moveBackwardInches( double inches, double speed ) {

        // haha, I am using another function so I don't have to rewrite the code.
        moveForwardInches( -inches, -speed );

    }

    public void moveRightInches( double inches, double speed ) {

        // Converts to integer by rounding. CASTS to int after rounding
        int tickTarget = (int)Math.round( inches * INCH_TICKS );

        resetDriveEncoders();

        // Motor movement to go right
        motorFrontLeft.setTargetPosition( tickTarget );
        motorFrontRight.setTargetPosition( -tickTarget );
        motorBackLeft.setTargetPosition( -tickTarget );
        motorBackRight.setTargetPosition( tickTarget );

        for( DcMotor x: allDriveMotors ) {

            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        // Move right. Think of this like a coordinate plane :)
        move( speed, 0, 0 );

        for ( DcMotor x: allDriveMotors ) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        // Wait for motors to reach the desired tick level
        waitForMotors();

        // Reset motors for normal movement
        resetDriveEncoders();

    }

    public void moveLeftInches( double inches, double speed ) {

        // haha, I am using another function so I don't have to rewrite the code.
        moveRightInches( -inches, -speed );

    }

    public void turnRightDegrees( int degrees, double speed ) {

        // Converts to integer by rounding. CASTS to int after rounding
        int tickTarget = (int)Math.round( degrees * ( NINETY_DEGREE_TICKS / 90 ) );

        resetDriveEncoders();

        // Motor movement to turn right
        motorFrontLeft.setTargetPosition( tickTarget );
        motorFrontRight.setTargetPosition( -tickTarget );
        motorBackLeft.setTargetPosition( tickTarget );
        motorBackRight.setTargetPosition( -tickTarget );

        for( DcMotor x: allDriveMotors ) {

            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        // Move right. Think of this like a coordinate plane :)
        move(0 , 0, speed );

        for ( DcMotor x: allDriveMotors ) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        // Wait for motors to reach the desired tick level
        waitForMotors();

        // Reset motors for normal movement
        resetDriveEncoders();

    }

    public void turnLeftDegrees( int degrees, double speed ) {

        turnRightDegrees(-degrees, -speed );

    }

    public void waitForMotors(){ // This method safely loops while checking if the opmode is active.
        boolean finished = false;
        while (auton.opModeIsActive() && !finished && !auton.isStopRequested()) {
            if (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                telem.addData("front left encoder:", "%7d / % 7d", motorFrontLeft.getCurrentPosition(), motorFrontLeft.getTargetPosition());
                telem.addData("back left encoder:", "%7d / % 7d", motorBackLeft.getCurrentPosition(), motorBackLeft.getTargetPosition());
                telem.addData("front right encoder:", "%7d / % 7d", motorFrontRight.getCurrentPosition(), motorFrontRight.getTargetPosition());
                telem.addData("back right encoder:", "%7d / % 7d", motorBackRight.getCurrentPosition(), motorBackRight.getTargetPosition());
                telem.update();
            } else {
                finished = true;
            }
        }
    }

    public void resetDriveEncoders(){
        for (DcMotor x : allDriveMotors) {
            x.setPower(0);
            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}