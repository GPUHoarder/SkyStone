package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test Autonomous", group="LinearOpMode")

public class TrobotixTestAutonomous extends LinearOpMode {
    public void runOpMode() {
        Configure configure = new Configure(hardwareMap);
        configure.initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);

        waitForStart();

        distanceDrive(1, 10); //Drive forward 10 cm at 30% power
        sleep(500);
        drive.latch();
        sleep(500);
        distanceDrive(1, -10); //Drive backwards 10 cm at 30% power



    }

    public void distanceDrive(double speed, double distance) {

        Configure configure = new Configure(hardwareMap);
        configure.initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);

        drive.move(speed, distance);
        sleep(drive.getTime());
        drive.stop();
    }
}
