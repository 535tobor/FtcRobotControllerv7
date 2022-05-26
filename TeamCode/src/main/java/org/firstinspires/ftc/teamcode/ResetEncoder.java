package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "resetEncoder")
public class ResetEncoder extends HwMapIterState{
    @Override
    public void init() {
        initHwMap();
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void init_loop()
    {
        telemetry.addData("extender counts: ", extender.getCurrentPosition());
    }
    @Override
    public void start()
    {

    }
    @Override
    public void loop()
    {
    }
    @Override
    public void stop(){

    }
}
