class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.last_error = 0
        self.integral = 0

    def control(self, setpoint, feedback):
        error = setpoint - feedback

        # Proportional term
        p_term = self.Kp * error

        # Integral term
        self.integral += error
        i_term = self.Ki * self.integral

        # Derivative term
        d_term = self.Kd * (error - self.last_error)
        self.last_error = error

        # Control output
        control_output = p_term + i_term + d_term

        return control_output


# Create a PID controller instance with specific gains
# Kp está relacionado a elasticidade do sinal - quanto maior mais elástico, e quanto menor menos elastico
# Kd está relacionado com a resistencia a mudança do sinal - oposto ao Kp
# Ki é usado para compensar o efeito das forças desconhecidas

pid_controller = PIDController(Kp=0.5, Ki=0.2, Kd=0.1)

# Set the desired setpoint and initial feedback value
setpoint = 10.0
feedback = 0.0

# Loop to simulate the control process
for _ in range(100):
    # Compute the control output
    control_output = pid_controller.control(setpoint, feedback)

    # Apply the control output to the system and obtain the new feedback value
    # In this example, we assume a simple system where the feedback is just a function of the control output
    feedback = control_output * 0.1

    # Print the current setpoint, feedback, and control output
    print(f"Setpoint: {setpoint}, Feedback: {feedback}, Control Output: {control_output}")
