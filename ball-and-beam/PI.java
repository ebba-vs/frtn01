// PI class to be written by you
public class PI {
	// Current PI parameters
	private PIParameters p;

    private double I = 0; // Integral part of controller
    private double e = 0; // Error signal
    private double v = 0; // Output from controller
	
	// Constructor
	public PI(String name) {
		p = new PIParameters();
		p.K = 3.0;
		p.Ti = 0.0;
		p.integratorOn = false;
		p.Beta = 1.0;
		p.H = 0.02;
		new PIGUI(this, p, name);

    }
	
	// Calculates the control signal v.
	// Called from BeamRegul.
	public synchronized double calculateOutput(double y, double yref) {
		this.e = yref - y;
		double P = p.K * (p.Beta * yref - y);
		v = P + I;
        return v;
    }
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BeamRegul.
	public synchronized void updateState(double u) {
		if (p.integratorOn) {
			I = I + (p.K * p.H / p.Ti) * e + (p.H / p.Tr) * (u - v);
		}else {
			I = 0.0;
		}
		
    }
	
	// Returns the sampling interval expressed as a long.
	// Note: Explicit type casting needed
	public synchronized long getHMillis() {
        return (long)(p.H * 1000);
    }
	
	// Sets the PIParameters.
	// Called from PIGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIParameters newParameters) {
		p = (PIParameters) newParameters.clone();
		if(!p.integratorOn) {
			I = 0.0;
		}
    }
}
