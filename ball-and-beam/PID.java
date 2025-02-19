// PID class to be written by you
public class PID {
	// Current PID parameters
	private PIDParameters p;

    private double I = 0; // Integral part of PID
    private double D = 0; // Derivative part of PID
    private double v = 0; // Computed control signal
    private double e = 0; // Error signal
    private double y = 0; // Measurement signal
    private double yOld = 0; // Old measurement signal
    private double ad; // Help variable for Derivative calculation
    private double bd; // Help variable for Derivative calculation
	
	// Constructor
	public PID(String name) {
        p = new PIDParameters();
        p.Beta = 1.0;
        p.H = .02;
        p.K = -0.06;
        p.N = 6;
        p.Td = 2.0;
        p.Ti = 0.0;
        p.integratorOn = false;
        p.Tr = 1.0;
        new PIDGUI(this, p, name);
        setParameters(p);

    }
	
	// Calculates the control signal v.
	// Called from BeamRegul.
	public synchronized double calculateOutput(double y, double yref) {
        this.e = yref - y;
        double P = p.K * (p.Beta * yref - y);
        v = P + I + D;
        return v;
    }
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BeamRegul.
	public synchronized void updateState(double u) {
        I = I + (p.K * p.H / p.Ti) * e + (p.H / p.Tr) * (u - v);
        this.yOld = y;
    }
	
	// Returns the sampling interval expressed as a long.
	// Note: Explicit type casting needed
	public synchronized long getHMillis() {
        return (long) (p.H * 1000);
    }
	
	// Sets the PIDParameters.
	// Called from PIDGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters) {
        p = (PIDParameters) newParameters.clone();
        if(!p.integratorOn) {
			I = 0.0;
		}
        // Recalculate the derivative approximation parameters.
        this.ad = p.Td / (p.Td + p.N * p.H);
        this.bd = p.K * ad * p.N;
    }
}
