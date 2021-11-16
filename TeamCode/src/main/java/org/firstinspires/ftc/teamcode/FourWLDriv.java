package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "4WLDR")
//@Disabled
public class FourWLDriv extends HwMapIter{
    @Override
    public void init()
    {
        initHwMap();
    }
    @Override
    public void init_loop()
    {

    }
    @Override
    public void start()
    {

    }
    @Override
    public void loop()
    {
        if(Math.abs(gamepad1.left_stick_y)>.1) {
            fl.setPower(-gamepad1.left_stick_y);
            bl.setPower(-gamepad1.left_stick_y);
        }
        else {
            fl.setPower(0);
            bl.setPower(0);
        }
        if(Math.abs(gamepad1.right_stick_y)>.1) {
            fr.setPower(-gamepad1.right_stick_y);
            br.setPower(-gamepad1.right_stick_y);
        }
        else {
            fr.setPower(0);
            br.setPower(0);
        }

    }
    @Override
    public void stop()
    {

    }
}
