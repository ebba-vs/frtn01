import SimEnvironment.*;

public class BallAndBeamRegul extends Thread {

    // IO Declarations 
    private AnalogSource analogInAngle;
    private AnalogSource analogInPosition;
    private AnalogSink analogOut;
    private AnalogSink analogRef;

    private ReferenceGenerator refGen;
    private PID pid; // Outer controller for ball position
    private PI pi;   // Inner controller for beam angle

    // Saturation limits for the control signal (actuator voltage)
    private final double UMIN = -10, UMAX = 10;

    // Constructor
    public BallAndBeamRegul(ReferenceGenerator refgen, BallAndBeam bb, int priority) {
        // Initialize I/O: 
        analogInPosition = bb.getSource(0);
        analogInAngle = bb.getSource(1);
        analogOut = bb.getSink(0);
        analogRef = bb.getSink(1);

        refGen = refgen;
        pid = new PID("Outer PID Controller");
        pi = new PI("Inner PI Controller");

        setPriority(priority);
    }
    
    /**
     * Method limit:
     *
     * @param u (double): The signal to saturate.
     * @return double: the saturated value.
     */
    private double limit(double u) {
        if (u < UMIN) 
            return UMIN;
        else if (u > UMAX) 
            return UMAX;
        else 
            return u;
    }

    public void run() {
        // Use the inner controller's sampling interval for periodic timing.
        long t = System.currentTimeMillis();
        
        double measuredPosition;
        double measuredAngle;
        double reference;
        double desiredBeamAngle; // Output from outer loop: desired beam angle.
        double controlSignal;    // Final control signal for the actuator.

        while (!interrupted()) {
            // === Outer Loop: Ball Position Control ===
            measuredPosition = analogInPosition.get();
            reference = refGen.getRef(); 
            analogRef.set(reference);

            // Synchronize on the PID controller to prevent parameter updates mid-calculation.
            synchronized (pid) {
                desiredBeamAngle = pid.calculateOutput(measuredPosition, reference);
                pid.updateState(desiredBeamAngle);
            }
            
            // === Inner Loop: Beam Angle Control ===
            measuredAngle = analogInAngle.get();

            // Synchronize on the PI controller.
            synchronized (pi) {
                // Use the desired beam angle from the outer loop as the setpoint.
                controlSignal = pi.calculateOutput(measuredAngle, desiredBeamAngle);
                controlSignal = limit(controlSignal);
                pi.updateState(controlSignal);
            }
            
            analogOut.set(controlSignal);

            t = t + pi.getHMillis();
            long duration = t - System.currentTimeMillis();
            if (duration > 0) {
                try {
                    sleep(duration);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }
    }
}