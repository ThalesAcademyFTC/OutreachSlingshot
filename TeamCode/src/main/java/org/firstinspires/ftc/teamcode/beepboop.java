package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "lights")
public class beepboop extends OpMode {

    Zegloub robot;

    public void init() {
        robot = new Zegloub(this, Zegloub.Drivetrain.MECHANUM );
    }

    @Override
    public void loop() {

        if (gamepad2.y) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }
        if (gamepad2.x) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
        }
        if (gamepad2.b) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
        }
        if (gamepad2.a) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
        }

    }
}
