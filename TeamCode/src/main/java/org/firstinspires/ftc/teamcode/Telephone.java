package org.firstinspires.ftc.teamcode;

import static java.nio.file.Files.move;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Telephone")
public class Telephone extends OpMode {

    double speed;

    DcMotor leftMotor, rightMotor;
    Servo servo1;



    double maxSpeed = 0.5;

    public double scaleStickValue(double value) {
        double squared = value*value;
        if (value < 0.0) {
            squared = -squared;
        }
        return squared*maxSpeed;
    }

    @Override
    public void init() {



        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        servo1 = hardwareMap.get(Servo.class, "servo1");


    }
    public void loop() {

        if (gamepad1.right_trigger > .1) {
            servo1.setPosition(.5);
        } else {
            servo1.setPosition(1);
        }

        }
    }




