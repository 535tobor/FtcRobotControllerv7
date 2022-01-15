package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Initialization")

public class Initialization extends HwMap{
    @Override
    public void runOpMode()
    {
        initHwMap();
        while(!isStarted()&&!isStopRequested())
        {
            telemetry.addData("pincer, extend, arm",  pincer.getPosition() +" " + extend.getCurrentPosition()+ " " + arm.getCurrentPosition());
            telemetry.update();
        }
        if(opModeIsActive())
        {
            pincer.setPosition(0.025);
            sleep(2000);
            runEncoder(extend, -1583, 250, .75);
            sleep(2000);
            runEncoder(arm, 1127, 250, .4);
        }
    }
}
