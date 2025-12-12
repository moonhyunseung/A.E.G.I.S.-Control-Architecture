import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. Filtered PID Controller (Strong Rival)
# ==========================================
class FilteredPIDController:
    """ 
    Standard PID with Low-Pass Filter on D-term 
    (Realistic Industrial PID - A worthy opponent)
    """
    def __init__(self, kp, ki, kd, tau_f=0.1, out_min=0.0, out_max=100.0, dt=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tau_f = tau_f # Filter Time Constant
        self.out_min = out_min
        self.out_max = out_max
        self.dt = dt
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_term_filtered = 0.0 

    def update(self, sp, pv):
        error = sp - pv
        
        # P Term
        p_term = self.kp * error
        
        # I Term (with Anti-windup)
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -100, 100)
        i_term = self.ki * self.integral
        
        # D Term (Filtered)
        d_raw = (error - self.prev_error) / self.dt
        # LPF: alpha depends on tau_f
        if self.tau_f > 0:
            alpha = self.tau_f / (self.tau_f + self.dt)
            self.d_term_filtered = alpha * self.d_term_filtered + (1 - alpha) * d_raw
        else:
            self.d_term_filtered = d_raw
            
        d_term = self.kd * self.d_term_filtered
        
        output = p_term + i_term + d_term
        output = np.clip(output, self.out_min, self.out_max)
        
        self.prev_error = error
        return output

# ==========================================
# 2. AEGIS (User's Tuned Version)
# ==========================================
class ZGatedPIController:
    """
    Improved Version: Adaptive PI-Controller
    Configured with User's parameters (High Base Gain, Low Gamma)
    """
    def __init__(self, base_gain, ki, gain_max=10.0, target_noise=0.5, 
                 p=1.5, gamma=0.01, anchor_decay=0.01, dt=1.0):
        self.base_gain = base_gain
        self.current_gain = base_gain # Starts high based on user input
        self.ki = ki
        self.gain_max = gain_max
        
        self.target_noise = target_noise
        self.p = p
        self.gamma = gamma
        self.anchor_decay = anchor_decay
        self.dt = dt
        
        self.m_error = 0.0
        self.ep_energy = 0.0
        self.integral = 0.0
        self.beta = 0.9

    def update(self, sp, pv):
        error = sp - pv
        
        # --- Z-Gating (Adaptive Logic) ---
        self.m_error = self.beta * self.m_error + (1 - self.beta) * error
        delta = error - self.m_error
        instability = abs(delta) ** self.p
        self.ep_energy = self.beta * self.ep_energy + (1 - self.beta) * instability
        sigma = (self.ep_energy + 1e-8) ** (1.0 / self.p)
        r = sigma / (self.target_noise + 1e-6)
        
        # Gain Dynamics
        # User Logic: base_gain=100 pulls gain UP, gamma=0.01 pushes DOWN slowly
        da = -self.gamma * (r - 1.0) * self.current_gain
        da -= self.anchor_decay * (self.current_gain - self.base_gain)
        
        self.current_gain += da
        # Clamped between 0.1 and gain_max (User implied 5.0, let's give it 10.0 headroom)
        self.current_gain = np.clip(self.current_gain, 0.1, self.gain_max)
        
        # --- PI Control ---
        p_term = self.current_gain * error
        
        self.integral += error * self.dt
        # Anti-windup scaled by Ki
        limit = 100.0 / (self.ki + 1e-6)
        self.integral = np.clip(self.integral, -limit, limit)
        i_term = self.ki * self.integral
        
        output = p_term + i_term
        output = np.clip(output, 0.0, 100.0)
        
        return output, self.current_gain

# ==========================================
# 3. Environment (With Severe Disturbance)
# ==========================================
class IndustrialHeater:
    def __init__(self, tau=20.0, gain=2.0, dt=1.0, initial_temp=20.0):
        self.tau = tau
        self.gain = gain
        self.dt = dt
        self.temp = initial_temp
        
    def step(self, u_input, external_noise=0.0, load_disturbance=0.0):
        # process: tau*y' + y = K*u + Disturbance
        d_temp = (self.gain * u_input - self.temp + load_disturbance) / self.tau * self.dt
        self.temp += d_temp
        return self.temp + external_noise

# ==========================================
# 4. Main Battle Loop
# ==========================================
def run_ultimate_battle():
    # Settings
    np.random.seed(42)
    duration = 500
    # Scenario: Start 50 -> Up to 80 -> Disturbance hits while at 80
    sp_schedule = [50] * 150 + [80] * 350
    
    # Noise Profile
    noise_std = 0.8
    noise_profile = np.random.normal(0, noise_std, size=duration)
    
    # [DISTURBANCE] Sudden Cold Draft (-30 degrees) from t=300 to t=450
    dist_profile = np.zeros(duration)
    dist_profile[300:450] = -30.0 
    
    # Controllers Setup
    
    # 1. Filtered PID (Strong Baseline)
    # Optimized tuned for this process
    pid = FilteredPIDController(kp=3.0, ki=0.15, kd=5.0, tau_f=2.0)
    
    # 2. AEGIS (User's Strategy)
    # Applying user's parameters: base=100, ki=0.05, gamma=0.01
    # Note: gain_max set to 10.0 to allow the high-gain strategy to work
    aegis = ZGatedPIController(
        base_gain=5.0,   # User's request (Strong Pull Up)
        ki=0.4,           # User's request (Gentle Integral)
        gamma=0.008,        # User's request (Slow Adaptation)
        target_noise=noise_std,
        gain_max=15.0      # Hard ceiling
    )
    
    # Simulation Objects
    proc_pid = IndustrialHeater()
    proc_aegis = IndustrialHeater()
    
    history = {
        'pid': [], 'aegis': [], 'gain_log': [], 
        'sp': sp_schedule, 'time': range(duration)
    }
    
    mv_pid, mv_aegis = 0, 0
    
    print("⚔️  STARTING ULTIMATE CONTROL BATTLE ⚔️")
    print(f"Strategy: High Base Gain (100.0) vs Filtered PID")
    print("Event: -30°C Cold Draft at t=300")
    
    for t in range(duration):
        sp = sp_schedule[t]
        noise = noise_profile[t]
        dist = dist_profile[t]
        
        # PID Step
        pv_pid = proc_pid.step(mv_pid, noise, dist)
        mv_pid = pid.update(sp, pv_pid)
        
        # AEGIS Step
        pv_aegis = proc_aegis.step(mv_aegis, noise, dist)
        mv_aegis, current_gain = aegis.update(sp, pv_aegis)
        
        history['pid'].append(pv_pid)
        history['aegis'].append(pv_aegis)
        history['gain_log'].append(current_gain)

    # Metrics (IAE)
    def get_iae(pv_data):
        return np.sum(np.abs(np.array(sp_schedule) - np.array(pv_data)))

    iae_pid = get_iae(history['pid'])
    iae_aegis = get_iae(history['aegis'])
    
    print("-" * 60)
    print(f"{'Metric':<15} | {'Filtered PID':<15} | {'AEGIS (Yours)':<15}")
    print("-" * 60)
    print(f"{'IAE (Error)':<15} | {iae_pid:<15.1f} | {iae_aegis:<15.1f}")
    improvement = (1 - iae_aegis/iae_pid)*100
    print(f"{'Improvement':<15} | {'-':<15} | {improvement:+.1f}%")
    print("-" * 60)

    # Visualization
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    
    # Plot 1: PV Comparison (The Race)
    ax1.plot(history['time'], sp_schedule, 'k--', label='Set Point', alpha=0.5)
    ax1.plot(history['time'], history['pid'], 'r-', label='Filtered PID', alpha=0.6)
    ax1.plot(history['time'], history['aegis'], 'g-', label='AEGIS (High-Gain)', linewidth=2)
    
    # Highlight Disturbance
    ax1.axvspan(300, 450, color='blue', alpha=0.1, label='Cold Draft (-30°C)')
    
    ax1.set_title("1. Robustness Test: Can it survive the storm?")
    ax1.set_ylabel("Temperature (°C)")
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: AEGIS Gain (The Strategy)
    ax2.plot(history['time'], history['gain_log'], 'b-', label='AEGIS Gain ($K_p$)')
    ax2.axhline(aegis.gain_max, color='r', linestyle=':', label='Max Limit')
    ax2.set_title("2. AEGIS Internal Gain State")
    ax2.set_ylabel("Gain Value")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_ultimate_battle()
