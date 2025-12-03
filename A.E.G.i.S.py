import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. Baseline: Standard PID Controller
# ==========================================
class PIDController:
    """
    Standard Position PID Algorithm.
    Note: Uses simple backward difference for D-term.
    """
    def __init__(self, kp, ki, kd, out_min=0.0, out_max=100.0, dt=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.dt = dt
        
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, sp, pv):
        error = sp - pv
        
        # P term
        p_term = self.kp * error
        
        # I term
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        
        # D term
        d_term = self.kd * (error - self.prev_error) / self.dt
        
        output = p_term + i_term + d_term
        output = np.clip(output, self.out_min, self.out_max)
        
        self.prev_error = error
        return output

# ==========================================
# 2. Advanced: Z-Gated Process Controller
# ==========================================
class ZGatedProcessController:
    """
    Adaptive Gain Controller using L^p Norm Energy (Z-Gated Logic).
    Designed for 1st/2nd order industrial processes.
    """
    def __init__(self, 
                 base_gain: float, 
                 target_noise: float = 0.5,
                 p: float = 1.5,
                 gamma: float = 0.05,
                 anchor_decay: float = 0.01,
                 gain_min: float = 0.1,
                 gain_max: float = 5.0,
                 beta: float = 0.9):
        
        self.base_gain = base_gain
        self.current_gain = base_gain
        self.target_noise = target_noise
        self.p = p
        self.gamma = gamma
        self.anchor_decay = anchor_decay
        self.gain_min = gain_min
        self.gain_max = gain_max
        self.beta = beta
        
        # Internal States
        self.m_error = 0.0    
        self.ep_energy = 0.0  

    def update(self, sp, pv):
        """
        Returns: (control_output, debug_info_dict)
        """
        error = sp - pv
        
        # 1. Trend & Fluctuation Extraction
        self.m_error = self.beta * self.m_error + (1 - self.beta) * error
        delta = error - self.m_error 
        
        # 2. Energy Measurement (L^p Norm)
        instability = abs(delta) ** self.p
        self.ep_energy = self.beta * self.ep_energy + (1 - self.beta) * instability
        sigma = (self.ep_energy + 1e-8) ** (1.0 / self.p)
        
        # 3. Z-Gating Ratio
        r = sigma / (self.target_noise + 1e-6)
        
        # 4. Dynamics Update (Gain Scheduling)
        da = -self.gamma * (r - 1.0) * self.current_gain
        da -= self.anchor_decay * (self.current_gain - self.base_gain)
        
        self.current_gain += da
        self.current_gain = np.clip(self.current_gain, self.gain_min, self.gain_max)
        
        # 5. Output Calculation (Adaptive P-Control)
        output = self.current_gain * error
        output = np.clip(output, 0.0, 100.0)
        
        # Return internal state for external logging
        debug_info = {
            'gain': self.current_gain,
            'r': r,
            'sigma': sigma
        }
        return output, debug_info

# ==========================================
# 3. Simulation Environment
# ==========================================
class IndustrialHeater:
    """
    First Order Plus Dead Time (FOPDT) approximated process.
    tau * dy/dt + y = K * u + Noise
    """
    def __init__(self, tau=20.0, gain=2.0, dt=1.0, initial_temp=20.0):
        self.tau = tau
        self.gain = gain
        self.dt = dt
        self.temp = initial_temp
        
    def step(self, u_input, external_noise=0.0):
        """
        u_input: Control signal (0-100%)
        external_noise: Random value passed from main loop (for fairness)
        """
        # Physics update (Euler method)
        d_temp = (self.gain * u_input - self.temp) / self.tau * self.dt
        self.temp += d_temp
        
        # Measurement = Physics + Noise
        measured_temp = self.temp + external_noise
        return measured_temp

# ==========================================
# 4. Benchmark Engine
# ==========================================
def calculate_metrics(sp_list, pv_list, mv_list):
    """Calculate IAE, ISE, and Control Effort"""
    e = np.array(sp_list) - np.array(pv_list)
    iae = np.sum(np.abs(e))             # Integral Absolute Error
    ise = np.sum(e ** 2)                # Integral Squared Error
    
    mv = np.array(mv_list)
    effort = np.sum(np.abs(np.diff(mv))) # Total Variation (Jitter proxy)
    
    return iae, ise, effort

def run_benchmark():
    # --- A. Setup & Reproducibility ---
    np.random.seed(42)  # [Critical] Fixed Seed
    
    duration = 300
    sp_schedule = [50] * 100 + [80] * 100 + [40] * 100 
    
    # Pre-generate ONE noise profile for FAIR comparison
    noise_std = 1.0
    noise_profile = np.random.normal(0, noise_std, size=duration)
    
    # Process Instances (Physics are identical)
    proc_pid = IndustrialHeater()
    proc_zg = IndustrialHeater()
    
    # Controllers
    # PID Tuned for baseline (Zigler-Nichols-ish)
    pid = PIDController(kp=2.5, ki=0.15, kd=1.0)
    
    # Z-Gated Controller
    zg = ZGatedProcessController(
        base_gain=2.5,      # Start similar to PID Kp
        target_noise=1.0,   # Matches the noise_std
        p=1.5,
        gamma=0.1,
        anchor_decay=0.01
    )
    
    # Data Structures
    history = {
        'time': range(duration),
        'sp': sp_schedule,
        'pid': {'pv': [], 'mv': []},
        'zg':  {'pv': [], 'mv': [], 'gain': [], 'r': []}
    }
    
    # --- B. Simulation Loop ---
    # Init MV
    mv_pid_prev = 0
    mv_zg_prev = 0
    
    for t in range(duration):
        sp = sp_schedule[t]
        noise = noise_profile[t] # [Critical] Same noise for both
        
        # 1. PID Loop
        pv_pid = proc_pid.step(mv_pid_prev, external_noise=noise)
        mv_pid = pid.update(sp, pv_pid)
        mv_pid_prev = mv_pid
        
        # 2. Z-Gated Loop
        pv_zg = proc_zg.step(mv_zg_prev, external_noise=noise)
        mv_zg, zg_info = zg.update(sp, pv_zg)
        mv_zg_prev = mv_zg
        
        # 3. Log
        history['pid']['pv'].append(pv_pid)
        history['pid']['mv'].append(mv_pid)
        
        history['zg']['pv'].append(pv_zg)
        history['zg']['mv'].append(mv_zg)
        history['zg']['gain'].append(zg_info['gain'])
        history['zg']['r'].append(zg_info['r'])

    # --- C. Metrics & Report ---
    pid_iae, pid_ise, pid_eff = calculate_metrics(sp_schedule, history['pid']['pv'], history['pid']['mv'])
    zg_iae, zg_ise, zg_eff = calculate_metrics(sp_schedule, history['zg']['pv'], history['zg']['mv'])
    
    print("="*40)
    print("      BENCHMARK REPORT (N=300)")
    print("="*40)
    print(f"{'Metric':<15} | {'PID':<10} | {'Z-Gated':<10} | {'Diff':<10}")
    print("-" * 53)
    print(f"{'IAE (Error)':<15} | {pid_iae:10.2f} | {zg_iae:10.2f} | {(zg_iae-pid_iae)/pid_iae*100:+.1f}%")
    print(f"{'ISE (Sq Error)':<15} | {pid_ise:10.2f} | {zg_ise:10.2f} | {(zg_ise-pid_ise)/pid_ise*100:+.1f}%")
    print(f"{'Effort (Jitter)':<15} | {pid_eff:10.2f} | {zg_eff:10.2f} | {(zg_eff-pid_eff)/pid_eff*100:+.1f}%")
    print("="*40)
    print("* IAE/ISE: Lower is better. Effort: Lower means less wear.")

    # --- D. Visualization ---
    
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    # 1. Tracking Performance
    axes[0].plot(history['time'], history['sp'], 'k--', label='Set Point', alpha=0.6)
    axes[0].plot(history['time'], history['pid']['pv'], 'r-', label='Standard PID', alpha=0.5, linewidth=1)
    axes[0].plot(history['time'], history['zg']['pv'], 'b-', label='Z-Gated Controller', linewidth=1.5)
    axes[0].set_title("1. Process Value (PV) Tracking")
    axes[0].set_ylabel("Temperature (°C)")
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    
    # 2. Control Output (Stability)
    axes[1].plot(history['time'], history['pid']['mv'], 'r-', label='PID Output', alpha=0.4)
    axes[1].plot(history['time'], history['zg']['mv'], 'b-', label='Z-Gated Output', alpha=0.9)
    axes[1].set_title("2. Control Effort (MV) & Stability")
    axes[1].set_ylabel("Heater Power (%)")
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    
    # 3. Adaptive Gain Dynamics
    ax3 = axes[2]
    ax3.plot(history['time'], history['zg']['gain'], 'g-', label='Adaptive Gain (a_t)')
    ax3.axhline(zg.base_gain, color='gray', linestyle=':', label='Base Gain')
    ax3.set_ylabel("Controller Gain", color='g')
    ax3.tick_params(axis='y', labelcolor='g')
    ax3.legend(loc='upper left')
    
    # Twin axis for 'r' ratio
    ax3_r = ax3.twinx()
    ax3_r.plot(history['time'], history['zg']['r'], 'm--', label='Instability Ratio (r)', alpha=0.3)
    ax3_r.axhline(1.0, color='m', linestyle=':', alpha=0.5)
    ax3_r.set_ylabel("Ratio r (Sigma/Z)", color='m')
    ax3_r.tick_params(axis='y', labelcolor='m')
    
    axes[2].set_title("3. Z-Gated Internal Dynamics")
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlabel("Time (Simulation Steps)")
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_benchmark()