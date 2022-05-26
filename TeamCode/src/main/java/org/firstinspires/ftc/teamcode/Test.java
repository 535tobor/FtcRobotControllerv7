package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Test")
public class Test extends HwMapLinearState {
    @Override
    public void runOpMode()
    {
        initHwMap();
        updateGyro();
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if(opModeIsActive())
        {
            waitFor(1);
            runtime.reset();
            while(opModeIsActive()&&runtime.time()<2.5){
                turnExtender.setPower(1);
            }
            turnExtender.setPower(0);
            waitFor(1);
        }
    }
}
