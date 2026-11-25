"""Topic taxonomy of the knowledge domains and what the repository actually covers.

Status vocabulary (one value per topic, about CONTENT AND SIMULATION only):
  Planned               no content in the repository
  In Progress           touched inside another notebook, or covered in part; the scope note says how
  Simulation Validated  a notebook treats the topic and its automated checks against analytic or
                        independent references pass (see the experiment result file)
Hardware validation is a separate dimension and is "Not validated" for every topic.
Research verification is a separate dimension: see research/source-register.md.

Notebook ids refer to tools/specs. A topic that is not in COVERAGE is Planned.
"""

from __future__ import annotations

SECTIONS = {
    "3.1 Control Engineering Fundamentals": [
        "Fundamentals of Control Systems", "Mathematical Modeling of Dynamic Systems", "Differential Equations",
        "Laplace Transform", "Transfer Functions", "Block Diagram Reduction", "Signal Flow Graphs",
        "State-Space Representation", "System Stability", "Transient Response", "Steady-State Response",
        "Time-Domain Analysis", "Frequency-Domain Analysis", "Bode Plots", "Nyquist Stability Criterion",
        "Root Locus", "Routh-Hurwitz Stability Criterion", "Controllability and Observability",
        "Sampling Theory", "Discrete-Time Systems", "Z-Transform", "Numerical Integration",
        "System Identification",
    ],
    "3.2 Classical Control Methods": [
        "Open-Loop Control", "Closed-Loop Control", "Feedback Control", "Feedforward Control",
        "Proportional Control", "Integral Control", "Derivative Control", "PI Controller", "PD Controller",
        "PID Controller", "Two-Degree-of-Freedom PID", "Cascade Control", "Ratio Control",
        "Split-Range Control", "Override Control", "Gain Scheduling", "Anti-Windup", "Setpoint Weighting",
        "Derivative Filtering", "Bumpless Transfer", "Deadband Compensation", "Disturbance Rejection",
        "Ziegler-Nichols Tuning", "Cohen-Coon Tuning", "Tyreus-Luyben Tuning", "IMC-Based Tuning",
        "Lambda Tuning", "Relay Feedback Tuning", "Auto-Tuning Methods", "Optimization-Based Tuning",
    ],
    "3.3 Advanced and Modern Control": [
        "State-Space Control", "Pole Placement", "Full-State Feedback", "LQR", "LQG", "Kalman Filter",
        "Extended Kalman Filter", "Unscented Kalman Filter", "Model Predictive Control",
        "Generalized Predictive Control", "Robust Control", "H-Infinity Control", "Sliding Mode Control",
        "Adaptive Control", "Self-Tuning Regulators", "Backstepping Control", "Feedback Linearization",
        "Deadbeat Control", "Disturbance Observer-Based Control", "Internal Model Control",
        "Repetitive Control", "Iterative Learning Control", "Event-Triggered Control",
        "Networked Control Systems", "Distributed Control Systems", "Hybrid Control Systems",
    ],
    "3.4 Intelligent Control": [
        "Fuzzy Logic Control", "Mamdani Fuzzy Control", "Sugeno Fuzzy Control", "Fuzzy PID",
        "Adaptive Fuzzy Control", "Neuro-Fuzzy Control", "Neural Network Control", "Neural PID",
        "Reinforcement Learning-Based Control", "Deep Reinforcement Learning Control",
        "Genetic Algorithm-Based Optimization", "Particle Swarm Optimization", "Differential Evolution",
        "Bayesian Optimization for Controller Tuning", "Hybrid Intelligent Control", "AI-Based Fault Detection",
    ],
    "3.5 Electrical Motor Control": [
        "DC Motor Modeling", "Armature Voltage Control", "Field Control (DC)", "DC Motor Speed Control",
        "DC Motor Position Control", "DC Motor Torque Control", "PWM-Based Control", "PI and PID Motor Control",
        "BLDC Motor Fundamentals", "Trapezoidal Commutation", "Six-Step Commutation", "Hall Sensor-Based Control",
        "Sensorless BLDC Control", "Back-EMF Detection", "BLDC PWM Strategies", "BLDC Speed and Torque Control",
        "PMSM Modeling", "Field-Oriented Control", "Direct Torque Control (PMSM)", "Space Vector PWM",
        "Sinusoidal PWM", "Maximum Torque Per Ampere", "Flux Weakening", "Sensorless PMSM Control",
        "Position and Speed Estimation", "Current Loop Design", "Speed Loop Design", "Position Loop Design",
        "Induction Motor Equivalent Circuit", "Scalar Control", "V/f Control", "Induction Vector Control",
        "Indirect Field-Oriented Control", "Direct Field-Oriented Control", "Direct Torque Control (Induction)",
        "Slip Compensation", "Induction Speed Estimation",
        "Stepper Motor Control", "Servo Motor Control", "Switched Reluctance Motor Control",
        "Synchronous Reluctance Motor Control", "Linear Motor Control",
        "Current Control", "Speed Control", "Position Control", "Torque Control", "Encoder Feedback",
        "Resolver Feedback", "Sensorless Estimation", "Motor Parameter Identification", "Current Sensing",
        "Dead-Time Compensation", "Inverter Control", "DC Bus Management", "Regenerative Braking",
        "Thermal Protection", "Torque Ripple Reduction", "Current Ripple Analysis",
    ],
    "3.6 Sequential and Logic Control": [
        "Finite State Machine", "Extended Finite State Machine", "Statecharts", "State Transition Tables",
        "Sequential Function Chart", "Ladder Logic", "Function Block Diagram", "Structured Text", "Relay Logic",
        "Boolean Logic", "Interlocking", "Latching and Unlatching", "Timers and Counters", "Event-Driven Control",
        "Sequence Control", "Fault Handling", "Alarm Logic", "Recovery Sequences", "Watchdog Mechanisms",
    ],
    "3.7 PLC and Industrial Automation": [
        "PLC Architecture", "PLC Hardware", "PLC Scan Cycle", "Digital and Analog I/O", "IEC 61131-3",
        "Ladder Diagram", "Function Block Diagram (PLC)", "Structured Text (PLC)", "Sequential Function Chart (PLC)",
        "Siemens S7-1200", "Siemens S7-1500", "Siemens TIA Portal", "Mitsubishi PLC", "Omron PLC",
        "Allen-Bradley ControlLogix", "CompactLogix", "Schneider Electric PLC", "CODESYS", "PLC and HMI Integration",
        "PLC and VFD Integration", "PLC and Servo Drive Integration", "Distributed Control Systems (DCS)",
        "Supervisory Control and Data Acquisition",
    ],
    "3.8 Process Control": [
        "Temperature Control", "Pressure Control", "Flow Control", "Liquid Level Control", "pH Control",
        "Pump Control", "Fan Control", "Compressor Control", "Boiler Control", "Heat Exchanger Control",
        "Batch Process Control", "Multivariable Control", "Cascade Process Control", "Feedforward Process Control",
        "Process Optimization", "Model Predictive Process Control",
    ],
    "3.9 Motion Control and Industrial Machinery": [
        "Conveyor Speed Control", "Conveyor Position Control", "Multi-Motor Synchronization", "Electronic Gearing",
        "Electronic Cam", "Servo Positioning", "Multi-Axis Coordination", "Gantry Control", "CNC Motion Control",
        "Robotics Motion Control", "AGV Motion Control", "AMR Navigation and Control", "Packaging Machines",
        "Sorting Machines", "Pick-and-Place Systems", "Automated Material Handling", "Industrial Motion Profiles",
        "Trapezoidal Motion Profiles", "S-Curve Motion Profiles",
    ],
    "3.10 Instrumentation and System Integration": [
        "Sensor Fundamentals", "Actuator Fundamentals", "Encoder", "Resolver", "Proximity Sensor",
        "Photoelectric Sensor", "Load Cell", "Pressure Transmitter", "Temperature Transmitter", "Flow Meter",
        "Signal Conditioning", "ADC and DAC", "PWM", "Filtering", "Noise Reduction", "Sensor Calibration",
        "Sensor Fusion", "Data Acquisition", "Analog and Digital Signal Processing",
    ],
    "3.11 Industrial Communication": [
        "Modbus RTU", "Modbus TCP", "PROFINET", "PROFIBUS", "EtherNet/IP", "EtherCAT", "CAN", "CANopen",
        "OPC UA", "MQTT", "IO-Link", "Industrial Ethernet", "Serial Communication", "PLC-to-PLC Communication",
        "HMI Communication", "Drive Communication", "Industrial Network Diagnostics",
    ],
    "3.12 Diagnostics, Reliability, and Safety": [
        "Fault Detection", "Fault Diagnosis", "Fault Isolation", "Fault-Tolerant Control", "Condition Monitoring",
        "Predictive Maintenance", "Remaining Useful Life Estimation", "Vibration Monitoring",
        "Motor Current Signature Analysis", "Thermal Monitoring", "Alarm Management", "Functional Safety Fundamentals",
        "Safety PLC", "Emergency Stop", "Safety Interlocks", "Fail-Safe Design",
        "Industrial Control System Cybersecurity",
    ],
}

SV, IP = "Simulation Validated", "In Progress"

# topic -> (status, [notebook ids], scope note)
COVERAGE = {
    "Fundamentals of Control Systems": (SV, ["01"], "Feedback, disturbance reduction, loop-delay limit on gain. Not a full course."),
    "Mathematical Modeling of Dynamic Systems": (SV, ["02", "06", "11"], "DC motor and belt conveyor models, verified against exact responses."),
    "Transfer Functions": (SV, ["02"], "DC motor speed/voltage transfer function checked against RK4 and scipy."),
    "State-Space Representation": (IP, ["10", "11", "12"], "Used inside the models and the MPC; there is no dedicated state-space tutorial."),
    "System Stability": (IP, ["01", "04"], "Ultimate gain and delay margin of FOPDT loops, gain and phase margin of tuned loops. No general stability theory."),
    "Transient Response": (IP, ["01", "03", "04"], "Step metrics (rise, overshoot, settling) are computed and tabulated; not taught as a topic."),
    "Steady-State Response": (IP, ["01"], "Steady-state error of a P loop against the closed form."),
    "Time-Domain Analysis": (IP, ["01", "03"], "Step and disturbance responses; no impulse or ramp analysis."),
    "Sampling Theory": (IP, ["07", "12"], "Scan-time and sample-time effects (missed pulses, delay). No sampling-theorem treatment."),
    "Discrete-Time Systems": (IP, ["03", "12"], "Discrete PID against dlsim; exact ZOH discretisation for MPC."),
    "Numerical Integration": (IP, ["02", "06"], "RK4 verified against exact responses; no study of step-size error."),
    "System Identification": (SV, ["02", "11"], "FOPDT step-response identification (two-point, least squares) only."),
    "Open-Loop Control": (IP, ["01"], "Compared with closed loop under a disturbance."),
    "Closed-Loop Control": (SV, ["01"], "Closed-form steady-state error and loop-delay stability verified."),
    "Feedback Control": (SV, ["01"], "See Closed-Loop Control."),
    "Feedforward Control": (IP, ["11", "12"], "Static feed-forward of the steady command on a conveyor; no dynamic feed-forward."),
    "Proportional Control": (SV, ["01", "03"], "P loop offset against the closed form."),
    "Integral Control": (SV, ["03"], "Integral action and windup."),
    "Derivative Control": (SV, ["03"], "Derivative on measurement, filtering, noise effect."),
    "PI Controller": (SV, ["03", "06", "11"], "Used and verified in several notebooks."),
    "PID Controller": (SV, ["03", "04"], "Positional discrete form verified against dlsim."),
    "Two-Degree-of-Freedom PID": (IP, ["03"], "Setpoint weighting only."),
    "Cascade Control": (SV, ["06", "09"], "Current-inside-speed cascades on DC and BLDC motors, compared with single loops."),
    "Gain Scheduling": (SV, ["08", "11"], "Scheduled PI on a nonlinear tank and on payload-varying conveyor."),
    "Anti-Windup": (SV, ["03", "11"], "Clamping and back-calculation compared with none."),
    "Setpoint Weighting": (SV, ["03"], "Effect on overshoot and effort."),
    "Derivative Filtering": (SV, ["03"], "Noise amplification with and without a filter."),
    "Bumpless Transfer": (IP, ["03"], "Tracking method implemented; limited testing."),
    "Disturbance Rejection": (SV, ["06", "11"], "Load-torque and item-drop responses measured."),
    "Ziegler-Nichols Tuning": (SV, ["04"], "Open- and closed-loop rules on four plants."),
    "Cohen-Coon Tuning": (SV, ["04"], "Rules computed from the published formulas; the source's own formulas were only partly verified."),
    "Tyreus-Luyben Tuning": (SV, ["04"], "Rules computed; source only partly verified."),
    "IMC-Based Tuning": (SV, ["04"], "IMC-PID and SIMC rules on four plants."),
    "Lambda Tuning": (SV, ["04", "06"], "Lambda rule and a DC motor application."),
    "Relay Feedback Tuning": (SV, ["04"], "Relay estimate of Ku and Pu with its error measured."),
    "Auto-Tuning Methods": (IP, ["04"], "Relay autotune followed by Ziegler-Nichols only."),
    "Optimization-Based Tuning": (IP, ["08"], "Search-based tuning under a fixed budget, used for the fuzzy/PI comparison."),
    "LQR": (IP, ["12"], "Infinite-horizon LQR gain used only as an independent reference for MPC."),
    "Model Predictive Control": (SV, ["12"], "Linear MPC, single input, box and slew constraints, SciPy solvers, no stability guarantee."),
    "Internal Model Control": (IP, ["04"], "IMC tuning rule only, not an IMC structure."),
    "Fuzzy Logic Control": (SV, ["08"], "Mamdani engine verified against hand-computed properties."),
    "Mamdani Fuzzy Control": (SV, ["08"], "Triangular sets, min/max, centroid."),
    "Fuzzy PID": (IP, ["08"], "An incremental fuzzy PI; no derivative channel."),
    "DC Motor Modeling": (SV, ["02", "06"], "Two-state armature and shaft model."),
    "Armature Voltage Control": (SV, ["06"], "Single loop with voltage output."),
    "DC Motor Speed Control": (SV, ["06"], "Single loop against cascade, with current limit."),
    "PI and PID Motor Control": (SV, ["06"], "PI speed and current loops."),
    "BLDC Motor Fundamentals": (SV, ["09"], "Trapezoidal back-EMF model and commutation table checked."),
    "Trapezoidal Commutation": (SV, ["09"], "Flat-top width and its effect on ripple."),
    "Six-Step Commutation": (SV, ["09"], "Sector table, steady speed, energy balance."),
    "Hall Sensor-Based Control": (SV, ["09"], "Hall offset effect on ripple, current and efficiency."),
    "BLDC Speed and Torque Control": (IP, ["09"], "Speed PI and cascade; torque ripple measured, not controlled."),
    "PMSM Modeling": (SV, ["10"], "Surface-magnet dq model."),
    "Field-Oriented Control": (SV, ["10"], "Clarke/Park, current loops, decoupling, speed loop."),
    "Space Vector PWM": (SV, ["10"], "Averaged SVPWM with linear limit; no switching waveform."),
    "Sinusoidal PWM": (IP, ["10"], "Used only as the reference for the SVPWM linear limit."),
    "Flux Weakening": (IP, ["10"], "Unintended field weakening observed at the voltage limit; no designed flux weakening."),
    "Current Loop Design": (SV, ["06", "10"], "PI from the plant R and L, decoupling and delay compensation studied."),
    "Speed Loop Design": (SV, ["06", "10"], "PI from the inertia and torque constant."),
    "Current Control": (SV, ["06", "09", "10"], "Current PI loops on three motor types."),
    "Speed Control": (SV, ["06", "09", "10", "11"], "Speed loops on DC, BLDC, PMSM and a conveyor drive."),
    "Encoder Feedback": (IP, ["06", "11"], "Speed quantisation effect only; no encoder signal model."),
    "Torque Ripple Reduction": (IP, ["09"], "Ripple quantified; no reduction method implemented."),
    "Current Ripple Analysis": (IP, ["09"], "Peak-to-peak ripple measurements in the six-step model."),
    "Finite State Machine": (SV, ["05"], "Moore machine with structural checks and a fuzz test."),
    "State Transition Tables": (SV, ["05"], "Explicit table, completeness measured."),
    "Boolean Logic": (IP, ["07"], "Seal-in and interlock logic inside the batching program."),
    "Interlocking": (SV, ["05", "07"], "Start/stop and emergency-stop interlocks checked scan by scan."),
    "Latching and Unlatching": (SV, ["07"], "SR latch and seal-in."),
    "Timers and Counters": (SV, ["07"], "TON, TOF, CTU against hand-derived timing diagrams."),
    "Event-Driven Control": (IP, ["05"], "Event-driven state machine; no event-queue study."),
    "Sequence Control": (SV, ["07"], "Batching station timeline."),
    "Fault Handling": (IP, ["05"], "Start failure and overload trips; not a diagnostic system."),
    "Recovery Sequences": (IP, ["05"], "Trip and manual reset only."),
    "Watchdog Mechanisms": (IP, ["05"], "A start-confirmation timeout; no hardware watchdog."),
    "PLC Scan Cycle": (SV, ["07"], "Scan-time effect on timer accuracy and missed pulses."),
    "IEC 61131-3": (IP, ["07"], "Function block behaviour of TON, TOF, CTU, R_TRIG, F_TRIG, SR only; the standard's text was not read."),
    "Liquid Level Control": (SV, ["08"], "Nonlinear tank with PI, scheduled PI and fuzzy PI."),
    "Model Predictive Process Control": (IP, ["12"], "Linear MPC demonstrated on a conveyor, not on a process plant."),
    "Conveyor Speed Control": (SV, ["11", "12"], "Payload change, item drop, unreachable reference, PI and MPC."),
    "Filtering": (IP, ["03"], "Derivative filter only."),
    "Emergency Stop": (IP, ["05"], "Latching e-stop behaviour in a state machine. Not a safety-rated design."),
    "Safety Interlocks": (IP, ["05", "07"], "Logic-level interlocks. Not functional safety."),
}

PROJECTS = [
    (1, "DC Motor Speed Control Using PID", "In Progress", ["06"], "Model, tuning, simulation and validation exist as notebook 06. No embedded implementation and no 20-part project write-up."),
    (2, "Conveyor Speed Control System", "In Progress", ["11"], "Simulation with payload change, encoder resolution and anti-windup. No PWM stage, no real encoder."),
    (3, "Conveyor Sorting Using FSM", "Planned", ["05"], "Notebook 05 covers a motor-starter FSM only."),
    (4, "Fuzzy Logic Motor Controller", "In Progress", ["08"], "Fuzzy PI against PI on a tank, not on a motor."),
    (5, "BLDC Motor Control", "In Progress", ["09"], "Six-step model with Hall offset. No firmware, no sensorless."),
    (6, "PMSM Field-Oriented Control", "In Progress", ["10"], "Transforms, SVPWM, current and speed loops in simulation. No firmware."),
    (7, "PLC-Based Conveyor Automation", "In Progress", ["07"], "Python emulation of the scan cycle and a batching program. No PLC vendor project, no HMI."),
    (8, "Multi-Motor Conveyor Synchronization", "Planned", [], "No content."),
    (9, "Industrial Pump Control", "Planned", [], "No content."),
    (10, "PLC-Based Tank Level Control", "Planned", ["08"], "The tank plant exists in notebook 08; no PLC program, alarms or interlocks."),
    (11, "Servo Positioning System", "Planned", [], "No content."),
    (12, "AGV Motion Control", "Planned", [], "No content."),
    (13, "Model Predictive Process Control", "In Progress", ["12"], "Linear MPC on a conveyor. No process plant."),
    (14, "Industrial Motor Condition Monitoring", "Blocked", [], "Needs real or public current, temperature and vibration data. No dataset was located or verified in this build, and synthetic data would not demonstrate fault detection."),
    (15, "Intelligent Control Comparison", "In Progress", ["08", "12"], "PI against fuzzy PI (notebook 08) and PI against MPC (notebook 12), on different plants. No single common benchmark."),
]


def topic_rows():
    rows = []
    for section, topics in SECTIONS.items():
        for t in topics:
            st, nbs, note = COVERAGE.get(t, ("Planned", [], ""))
            rows.append(dict(section=section, topic=t, status=st, notebooks=nbs, note=note))
    unknown = set(COVERAGE) - {t for ts in SECTIONS.values() for t in ts}
    if unknown:
        raise ValueError(f"coverage entries not in taxonomy: {sorted(unknown)}")
    return rows


ROADMAP_LEVELS = {
    0: "Engineering Fundamentals", 1: "Classical Control", 2: "Sequential and Industrial Control",
    3: "Motor and Motion Control", 4: "Advanced Control", 5: "Intelligent Control",
    6: "Industrial Integration", 7: "Advanced Engineering Projects",
}

# notebook number -> roadmap level it belongs to (author's placement)
NOTEBOOK_ROADMAP = {"01": 1, "02": 1, "03": 1, "04": 1, "05": 2, "06": 3, "07": 2, "08": 5,
                    "09": 3, "10": 3, "11": 3, "12": 4}


def difficulty(spec_level: int) -> str:
    """Author's judgement from the spec level (0-5); not a measured quantity."""
    return "Beginner" if spec_level <= 1 else "Intermediate" if spec_level <= 3 else "Advanced"


# which lab page of the preview app shows each notebook
LAB_OF = {"01": "process", "02": "process", "03": "process", "04": "process", "05": "plc",
          "06": "motor", "07": "plc", "08": "process", "09": "motor", "10": "motor",
          "11": "motor", "12": "motor"}
