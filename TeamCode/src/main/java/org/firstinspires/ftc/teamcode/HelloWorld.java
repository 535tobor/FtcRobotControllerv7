package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HelloWorld")
public class HelloWorld extends OpMode {
    @Override
    public void init()
    {
        telemetry.addData("Hello,"," World!");
    }

    @Override
    public void loop() {
        telemetry.addData("Hello,"," World!");
    }
    @Override
    public void init_loop() {

    }


    @Override
    public void start() {

    }
    @Override
    public void stop() {

    }
}
