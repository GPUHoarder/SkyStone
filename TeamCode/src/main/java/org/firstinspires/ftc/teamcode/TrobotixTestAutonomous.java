package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test Autonomous", group="LinearOpMode")

public class TrobotixTestAutonomous extends LinearOpMode {


    public void runOpMode() {
        Configure configure = new Configure(hardwareMap);
        configure.Initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);


        waitForStart();

        DistanceDrive(10);



    }

    public void DistanceDrive(double distance) {

        Configure configure = new Configure(hardwareMap);
        configure.Initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);

        int driveTime = 0;
        driveTime = (int)((distance/(72.5/2))*1000);

        drive.DriveForward(1, 5);
        sleep(drive.getTime());
        drive.Stop();
    }
}
