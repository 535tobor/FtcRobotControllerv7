package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "State BluwWH")
public class AutoState extends HwMapLinearState {
    @Override
    public void runOpMode()
    {
        initHwMap();
        updateGyro();
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if(opModeIsActive())
        {
            driveByTime(.8, 1.5);
            motorRTPIdle(extender, extender.getCurrentPosition()-2300,.8);//extend arm to tower
            extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            basket.setPosition(.3);//drop block
            waitFor(3);
            basket.setPosition(.125);
            waitFor(2);
            motorRTP(extender,extender.getCurrentPosition()+2200,-.8);//start retract
            driveByTime(.8,1.5);
            robotTurnToAngle(-100,4,1,1);
            driveByTime(-1,5);
            robotTurn(-100, 5);//Turn back bot to opposing wh

        }
    }
}
