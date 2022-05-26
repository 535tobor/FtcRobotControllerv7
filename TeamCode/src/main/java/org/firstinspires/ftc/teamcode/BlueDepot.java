package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "State Blue Depot")
public class BlueDepot extends HwMapLinearState {
    @Override
    public void runOpMode()
    {
        initHwMap();
        updateGyro();
        basket.setPosition(.125);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if(opModeIsActive())
        {
            driveByTime(.8, .25);
            motorRTP(extender, extender.getCurrentPosition()-2300,.8);//extend arm to tower
            extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            waitFor(.5);
            basket.setPosition(0);//drop block
            waitFor(3);
            basket.setPosition(.125);
            waitFor(2);
            motorRTP(extender,extender.getCurrentPosition()+2200,-.8);//start retract
            robotTurnToAngle(-120,4,1,1);
            driveByTime(.8,.6);

        }
    }
}
