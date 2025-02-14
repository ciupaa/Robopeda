package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
@Config
@Autonomous(name = "NextFTC Autonomous Program Java")
public class autotest extends PedroOpMode {

    public class Lift extends Subsystem {
        // BOILERPLATE
        public final Lift INSTANCE = new Lift();
        private Lift() { }

        // USER CODE
        public MotorEx motor;

        private PIDController lcontroller;
        public static double lp = 0.01, li = 0, ld = 0.0002;
        public static double lf = 0.14;

        public static double ltarget = 0 ;

        private final double ticks_in_mm = 3.20;
        public String name = "lift_motor";

        public Command toLow() {
            return new RunToPosition(motor, // MOTOR TO MOVE
                    0.0, // TARGET POSITION, IN TICKS
                    controller, // CONTROLLER TO IMPLEMENT
                    this); // IMPLEMENTED SUBSYSTEM
        }

        public Command toMiddle() {
            return new RunToPosition(motor, // MOTOR TO MOVE
                    500.0, // TARGET POSITION, IN TICKS
                    controller, // CONTROLLER TO IMPLEMENT
                    this); // IMPLEMENTED SUBSYSTEM
        }

        public Command toHigh() {
            return new RunToPosition(motor, // MOTOR TO MOVE
                    1200.0, // TARGET POSITION, IN TICKS
                    controller, // CONTROLLER TO IMPLEMENT
                    this); // IMPLEMENTED SUBSYSTEM
        }

        @Override
        public void initialize() {
            motor = new MotorEx(name);
        }
    }
    @Override
    public void onInit() {
        super.onInit();



    }

    @Override
    public void onStartButtonPressed() {
        super.onStartButtonPressed();

    }
}
