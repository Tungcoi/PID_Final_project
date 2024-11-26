# Control and Trajectory Tracking for Autonomous Vehicle
**`Step-by-Step Instructions`**

## STEP1: Setup Environment
### 1. Log into VM Workspace

Open the VM workspace and log into the VM to practice the current project. 
Once you log into the VM, open a Terminal window. 

<br/><br/>

### 2. Clone the Repository

Fork the repository to your Github account and clone the repository to the workspace using the commands below. 

```bash
git clone https://github.com/udacity/nd013-c6-control-starter.git
```

Change to the project directory.
```bash
cd nd013-c6-control-starter/project
```

<br/><br/>

### 3. Review the starter files
You will find the following files in the project directory.

```bash
.
├── cserver_dir
├── install-ubuntu.sh
├── manual_control.py
├── pid_controller/     # TODO Files
├── plot_pid.py
├── run_main_pid.sh
├── simulatorAPI.py
├── steer_pid_data.txt
└── throttle_pid_data.txt
```

<br/><br/>

### 4. Start the Carla Server
Create a script file ‘run_carla.sh’ with the following shell script

```bash
#!/bin/bash
SDL_VIDEODRIVER=offscreen /opt/carla-simulator/CarlaUE4.sh -opengl&
```

Then execute "run_carla.sh" to start the Carla server.
```bash
chmod +x run_carla.sh
./run_carla.sh
```

<br/><br/>

### 5. Install Dependencies
Open another Terminal tab, and change to the **nd013-c6-control-starter/project**  directory. Execute the following shell script to install the project-specific dependencies. 
```bash
./install-ubuntu.sh
```
This file will install utilities such as, `libuv1-dev`, `libssl-dev`, `libz-dev`, `uWebSockets`. 

<br/><br/>

## Step 2. Update the Project Code

Change to the **pid_controller/** directory.
```bash
cd pid_controller/
```
Sovle TODO markers as well in these files.

### pid_controller.cpp

**`void PID::Init()`**
```c++
void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   this->k_p = Kpi;
   this->k_i = Kii;
   this->k_d = Kdi;
   this->lim_max_output = output_lim_maxi;
   this->lim_min_output = output_lim_mini;
   this->cte = 0.0;
   this->diff_cte = 0.0;
   this->sum_cte = 0.0;
   this->delta_t = 0.0;
   if (is_log) 
   {    
        std::cout<<"k_p = "<< k_p << ",k_i = " << k_i<< ",k_d = " << k_d<<endl;
        std::cout<<"lim_max_output = "<< lim_max_output << ",lim_min_output = " << lim_min_output<<endl;
        std::cout<<"cte = "<< cte << ",diff_cte = " << diff_cte<< ",sum_cte = " << sum_cte<< ",delta_t = " << delta_t<<endl;
   }
}
```

**`PID::UpdateError()`**
```c++
void PID::UpdateError(double update_cte) {
    if (this->cte == 0.0 && this->sum_cte == 0.0) {
        if (is_log) std::cout<<"UpdateError first time"<<endl;
        this->cte = update_cte;
    }

    if (std::abs(this->delta_t) > 0.0001) {
        this->diff_cte = (update_cte - this->cte) / this->delta_t;
    } else {
        this->diff_cte = 0.0;
    }

    this->sum_cte += cte * this->delta_t;
    this->cte = update_cte;
    if (is_log) std::cout<<"UpdateError diff_cte = "<< this->diff_cte << ", sum_cte = "<< this->sum_cte << ", cte = "<< this->cte <<endl;
}
```
**`PID::TotalError()`**
```c++
double PID::TotalError() {
   // // Calculate the PID control value and limit it within the output range
    double control = (this->k_p * this->cte) + (this->k_d * this->diff_cte) + (this->k_i * this->sum_cte);
    if (is_log) std::cout<<"TotalError control = "<< control<<endl;

    // Check and limit the control value within the range lim_min_output and lim_max_output
    if (control > this->lim_max_output) {
        return this->lim_max_output;
    } else if (control < this->lim_min_output) {
        return this->lim_min_output;
    }
    return control;
}
```

**`PID::UpdateDeltaTime()`**
``` c++
double PID::UpdateDeltaTime(double new_delta_time) {
   this->delta_t = new_delta_time;
   return this->delta_t;
}
```

**`Constructor`**
Uncomment constructor (it will be implemented in header file)
```c++
// PID::PID() {}
```

### pid_controller.h
Add properties and constructor
``` c++
class PID {
public:
    double cte;
    double diff_cte;
    double sum_cte;
    double k_p;
    double k_i;
    double k_d;
    double lim_max_output;
    double lim_min_output;
    double delta_t;
    bool is_log = false;
    PID(int isLog = false): is_log(isLog) {};
    virtual ~PID();
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);
    void UpdateError(double cte);
    double TotalError();
    double UpdateDeltaTime(double new_delta_time);
};
```

### main.cpp
**`Add parameter handling to set PID controllers at runtime`**

```c++
int main(int argc, char* argv[]) 
{
    // Vector for default PID parameters
    std::vector<std::vector<double>> pid_steer_params = {
        {0.35, 0.001, 0.2}  // Default
    };
    std::vector<std::vector<double>> pid_throttle_params = {
        {0.7, 0.10, 0.2}  // Default
    };

    // Override default if arguments are provided
    if (argc > 6) {
        pid_steer_params[0][0] = std::stod(argv[1]);  // Kp (steer)
        pid_steer_params[0][1] = std::stod(argv[2]);  // Ki (steer)
        pid_steer_params[0][2] = std::stod(argv[3]);  // Kd (steer)
        pid_throttle_params[0][0] = std::stod(argv[4]);  // Kp (throttle)
        pid_throttle_params[0][1] = std::stod(argv[5]);  // Ki (throttle)
        pid_throttle_params[0][2] = std::stod(argv[6]);  // Kd (throttle)
        cout << "Using custom PID parameters." << endl;
    } else {
        cout << "Using default PID parameters." << endl;
    }
    
    // Get PID parameters for `steer` and `throttle`
    auto steer_params = pid_steer_params[0];
    auto throttle_params = pid_throttle_params[0];
    // .... //

    // initialize pid steer
    double max_steer = 1.2;
    PID pid_steer = PID();
    double steer_kp = steer_params[0];
    double steer_ki = steer_params[1];
    double steer_kd = steer_params[2];
    pid_steer.Init(steer_kp, steer_ki, steer_kd, max_steer, -max_steer);   // Initialize the steering controller with PID parameters

    // initialize pid throttle
    double max_throttle = 1.0;
    double max_brake = -1.0;
    PID pid_throttle = PID(true);
    double throttle_kp = throttle_params[0];
    double throttle_ki = throttle_params[1];
    double throttle_kd = throttle_params[2];
    pid_throttle.Init(throttle_kp, throttle_ki, throttle_kd, max_throttle, max_brake);

    //.. //
}
```

**`Get position and velocity and target`**
```c++
   double target_x = x_points.back();
   double target_y = y_points.back();
   double target_v = v_points.back();
   double current_x = x_position;
   double current_y = y_position;
   double current_v = velocity;
```

**`Steering control`**

``` c++
   pid_steer.UpdateDeltaTime(new_delta_time);  // Update delta time for the steering controller
   double error_steer = 0.0;
   double steer_output = 0.0;
   error_steer = angle_between_points(current_x, current_y, target_x, target_y) - yaw;
   pid_steer.UpdateError(error_steer);
   steer_output = pid_steer.TotalError(); // Get steering control output from PID
```

**`Throttle control`**

``` c++
   pid_throttle.UpdateDeltaTime(new_delta_time);  // Update delta time for the throttle controller
               
   double error_throttle = 0.0;
   double distance_to_target = utils::distance(current_x, current_y, target_x, target_y);

   // Check if the distance is almost zero to avoid division by zero
   if (std::abs(distance_to_target) < DBL_EPSILON) {
         error_throttle = 0.0;
   } else {
         // Compute the acceleration required to reach the target speed
         error_throttle = (std::pow(target_v, 2) - std::pow(current_v, 2)) / (2 * distance_to_target);
   }
   std::cout<<"error_throttle = "<< error_throttle<<endl;
   // Clamp the error_throttle value to avoid over-correction
   error_throttle = utils::clampD(error_throttle, -1.0, 1.0);
   pid_throttle.UpdateError(error_throttle);  // Update the PID controller error for throttle
   double throttle = pid_throttle.TotalError(); // Get throttle control output from PID

   double throttle_output = 0.0;
   double brake_output = 0.0;
   // Adapt the negative throttle to break
   if (throttle > 0.0) {
         throttle_output = throttle;
         brake_output = 0;
   } else {
         throttle_output = 0;
         brake_output = -throttle;
   }
```

## Step 3. Build and Execute the Project
**`Build`** by executing the project using the commands below

```bash
# Build the project
# Run the following commands from the pid_controller/ directory
cmake .
# The command below compiles your c++ code. Run it after each time you edit the CPP or Header files
make
```

**`Run`** the project
```bash
# Run the project
cd ..
```

**`Update`** script "run_main_pid.sh" to run with params:
``` bash 
#!/bin/bash

# Default PID parameters
STEER_KP=0.35
STEER_KI=0.001
STEER_KD=0.2
THROTTLE_KP=0.7
THROTTLE_KI=0.1
THROTTLE_KD=0.2

# Check if custom parameters are provided as arguments
if [ "$#" -eq 6 ]; then
  STEER_KP=$1
  STEER_KI=$2
  STEER_KD=$3
  THROTTLE_KP=$4
  THROTTLE_KI=$5
  THROTTLE_KD=$6
elif [ "$#" -ne 0 ]; then
  echo "Invalid number of arguments! Provide either 0 or 6 arguments."
  echo "Usage: ./run_main_pid.sh [STEER_KP STEER_KI STEER_KD THROTTLE_KP THROTTLE_KI THROTTLE_KD]"
  exit 1
fi

# Run the pid_controller with the provided or default PID parameters
./pid_controller/pid_controller $STEER_KP $STEER_KI $STEER_KD $THROTTLE_KP $THROTTLE_KI $THROTTLE_KD &
sleep 1.0

# Run the simulator API and append stdout to the same log file
python3 simulatorAPI.py
```
**`Run project with default params:`**
```bash
# Run the following commands from the nd013-c6-control-starter/project directory
./run_main_pid.sh
```

**`Or tune the PID Parameters (example):`**
```bash
# Run the following commands from the nd013-c6-control-starter/project directory
./run_main_pid.sh 0.5 0.02 0.4 0.6 0.2 0.4
```

**`Result`**
![Run The Project](/Screen.png)

If the execution fails silently, you can use **ctrl + C** to stop, and try again. 

Another possible error you may get is `bind failed. Error: Address already in use`. In such a case, you can kill the process occupying the required port using the commands below.

```bash
ps -aux | grep carla
# Use the IDs displayed in the output of the last command. 
kill id     
```

<br/><br/>

## Step 4. Analyze and Evaluate the Controller's Performance

### Plot the Results
   - The error values and control outputs for the throttle and steering are saved in `throttle_data.txt` and `steer_data.txt`, respectively.  
   - Use the following command to plot these values:  
     ```bash
     python3 plot_pid.py
      ```
**`Steering`**

![Steering](/Steering.png)

**`Throttle`**

![Throttle](/Throttle.png)


## Anser question

### 1. Add the plots to your report and explain them
1. Steering Control Plot

	•	Error Steering vs. Steering Output:  
   This plot illustrates the PID controller’s performance in steering the vehicle toward the desired path:  
	•	Early Iterations (0–40):  
The error steering is small and the steering output closely tracks it, indicating good initial performance.  
	•	Middle Iterations (40–100):  
Oscillations become prominent, suggesting the controller is struggling with cross-track error corrections. These oscillations could be due to suboptimal tuning of the derivative or proportional terms.  
	•	Later Iterations (100–160):  
The controller reduces error but still exhibits periodic oscillations. This might indicate insufficient damping or a potential delay in feedback.  
	•	General Observations:  
While the steering controller maintains control, its oscillations indicate the need for better tuning, particularly with the derivative gain. However, this is the most stable performance I achieved within the constraints of time.

2. Throttle Control Plot

	•	Error Throttle, Brake Output, and Throttle Output:  
This plot reflects the controller’s performance in managing speed:  
	•	Early Iterations (0–40):  
Sudden large oscillations in throttle error and brake output occur, possibly due to rapid changes in the desired velocity or overly aggressive PID gains.  
	•	Middle Iterations (40–100):  
The throttle output stabilizes but still fluctuates slightly as the vehicle adapts to the desired speed. Brake output remains near zero, showing minimal deceleration was required during this phase.  
	•	Later Iterations (100–160):    
Throttle output begins to stabilize better, indicating improved convergence toward the desired velocity.  
	•	General Observations:   
The throttle controller shows an overall improvement after initial instability. The chosen PID parameters balance acceleration and braking reasonably well, though further optimization could reduce oscillations in error.

### 2. What is the effect of the PID according to the plots, how each part of the PID affects the control command?
   1.	Proportional (P) Component:  
   The P component provides a quick response to the error, which is why the system reacts promptly to deviations, as seen in the steering plot around iterations 20-50. However, during tuning, higher P values led to excessive oscillations, while lower values made the response too slow. Due to time constraints, this value was selected as it delivered acceptable responsiveness with manageable oscillations.
   
   2.	Integral (I) Component:  
   The I component addresses accumulated errors over time. In the plots, it helps stabilize the system in later iterations by correcting small deviations that P alone cannot resolve (e.g., in both steering and throttle outputs). Higher I values during testing caused the system to overshoot, while lower values made convergence too slow. This chosen value is not perfect but works adequately within the available testing time.
   
   3.	Derivative (D) Component:  
   The D component helps to dampen oscillations and prevent excessive corrections. In the throttle plot, for instance, D reduces the impact of rapid changes in error, leading to smoother outputs by iteration 100. Experiments with different D values showed that higher D values slowed the response excessively, while lower values allowed instability. This value was chosen as a reasonable compromise given the time limitations.

**`Summary:`**
The current PID parameters are not necessarily optimal but represent the best results achievable within the constraints of limited tuning time. Each component plays a critical role: P ensures a rapid initial response, I minimizes residual errors, and D smooths the overall behavior. While further tuning might improve performance, these values were sufficient for balancing responsiveness, stability, and steady-state accuracy within the available time.

### 3. How would you design a way to automatically tune the PID parameters?

To design an automatic PID tuning method, there are several theoretical approaches that I have learned but have not yet implemented due to time and resource limitations:

1.	Ziegler-Nichols Method:  
	   This method starts with setting the I and D terms to zero and gradually increasing the P term until the system exhibits sustained oscillations. From the observed oscillation period and ultimate gain, the PID parameters can be calculated. It is straightforward and widely used, but it might not work well for all systems.

2.	Cohen-Coon Method:  
	   This empirical method requires analyzing the system’s step response to create a process reaction curve. From this curve, tuning parameters are derived to achieve optimal control. It provides more refined tuning compared to Ziegler-Nichols but requires more detailed system analysis.

3.	Iterative Heuristic Methods:  
	   Using optimization algorithms like Genetic Algorithms (GA) or Particle Swarm Optimization (PSO), one could design a program that iteratively tests and refines the PID parameters. These methods are flexible and work well for complex systems, but they are computationally intensive.

4.	Adaptive Tuning Tools:  
	   Implementing software or controllers that dynamically adjust PID parameters in real-time based on the system’s performance metrics. This approach ensures the system adapts to changing conditions but requires advanced programming and real-time computation.

While these strategies are well-documented and theoretically sound, I have not been able to implement any of them so far due to time constraints and limited hands-on experience with the necessary tools and algorithms.

### 4. PID controller is a model-free controller; could you explain the pros and cons of this type of controller?
Pros of a Model-Free PID Controller:

1.	Simplicity:  
•	The PID controller doesn’t require a mathematical model of the system, which makes it easy to understand and implement.
•	It works effectively for a wide range of systems without needing to derive complex equations.

2.	Flexibility:  
•	The same PID structure can be applied to various systems, from temperature control to robotics, by simply tuning the parameters.
•	It’s versatile and can adapt to many control scenarios with trial-and-error tuning.

3.	Widely Used and Tested:   
•	Because it’s so common, there are many resources, tools, and guides available for implementation.
•	Most software frameworks and hardware systems support PID natively.

4.	Robustness:  
•	A well-tuned PID controller can handle disturbances and uncertainties reasonably well without needing an exact model.

Cons of a Model-Free PID Controller:

1.	Tuning is Difficult:  
•	Finding the right parameters for Kp, Ki, and Kd can be time-consuming, especially for complex systems.
•	Without a systematic tuning method, trial-and-error might not yield optimal results.

2.	Performance Limitations:  
•	PID controllers struggle with highly dynamic or nonlinear systems where a more advanced model-based controller might excel.
•	They can’t predict future behavior, which limits their efficiency in systems requiring foresight.

3.	Sensitivity to Changes:  
•	If the system dynamics change significantly (e.g., due to wear and tear, load variations), the controller may require retuning.
•	PID lacks adaptability to handle large variations without manual intervention.

4.	No Explicit Optimization:  
•	Unlike model-based approaches, a PID controller doesn’t guarantee the best performance, such as minimizing energy use or maximizing response speed.

While I understand these pros and cons from theory and basic practice, my limited experience has shown me that the main challenge is in tuning and maintaining stability, especially in systems that behave unexpectedly. As I gain more experience, I hope to explore more sophisticated techniques to overcome these limitations.

### 5. (Optional) What would you do to improve the PID controller?

To improve the PID controller, I would need more time to study and apply the concepts discussed in Question 4, such as advanced tuning methods, gain scheduling, and noise filtering. These approaches seem promising, but I currently lack the expertise and time to implement them effectively. For now, I would prioritize understanding these techniques better and experimenting with them when I have the opportunity.


 