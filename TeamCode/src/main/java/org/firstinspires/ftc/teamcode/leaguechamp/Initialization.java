package org.firstinspires.ftc.teamcode.leaguechamp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Initialization")
@Disabled
public class Initialization extends HwMap {
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
            //pincer.setPosition(0.025);
            //sleep(2000);
            runEncoder(extend, -10, 40, .75);
            sleep(2000);
            runEncoder(arm, 730, 20, .4);
        }
    }
}
