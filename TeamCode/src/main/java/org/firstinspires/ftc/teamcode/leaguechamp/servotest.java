package org.firstinspires.ftc.teamcode.leaguechamp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")
@Disabled
public class servotest extends OpMode {
    Servo test;
    @Override
    public void init()
    {
        test = hardwareMap.servo.get("test");
    }
    @Override
    public void loop()
    {

        telemetry.update();
    }
}
