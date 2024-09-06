package org.firstinspires.ftc.teamcode;

import static java.nio.file.Files.move;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp(name = "Telephone")
public class Telephone extends OpMode {

    double speed = .3;
    double speedBadWheel = .25;

    DcMotor leftMotor, rightMotor;
    Servo servo1;
    


    double maxSpeed = 0.5;



    @Override
    public void init() {

        //defines motors through hardware map

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        //defines servos through hardware map

        servo1 = hardwareMap.get(Servo.class, "servo1");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void loop() {

        if (gamepad1.right_trigger > .1) {
            //shoots the duck
            servo1.setPosition(.3);
        } else {
            //brings the servo to loading position
            servo1.setPosition(.6);
        }

        //movement code

        if (gamepad1.left_stick_y > .25) {
            //robot moves forward
            leftMotor.setPower(-.25);
            rightMotor.setPower(-speed);
        } else if (gamepad1.left_stick_y < -.25) {
            //robot moves backward
            leftMotor.setPower(.25);
            rightMotor.setPower(speed);
        } else if (gamepad1.left_stick_x <+ -.25) {
            //robot turns left
            leftMotor.setPower(-speed);
            rightMotor.setPower(speed);
        } else if (gamepad1.left_stick_x > .25) {
            //robot turns right
            leftMotor.setPower(speed);
            rightMotor.setPower(-speed);
        } else {
            //robot stops when no joystick input
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        //if stick is not moving, robot is not moving



        }
    }




