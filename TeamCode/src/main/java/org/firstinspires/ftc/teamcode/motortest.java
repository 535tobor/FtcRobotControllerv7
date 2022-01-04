package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "MotorTest")
@Disabled
public class motortest extends OpMode {
    DcMotor test;
    @Override
    public void init()
    {
        test = hardwareMap.dcMotor.get("test");
    }
    @Override
    public void loop()
    {
        if(Math.abs(gamepad1.left_stick_y)>.1)
        {
            test.setPower(gamepad1.left_stick_y);
        }
        else
        {
            test.setPower(0);
        }
        telemetry.update();
    }
}
