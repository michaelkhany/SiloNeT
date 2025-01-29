# **SiloNeT - Simulation & Localization Network Tool**

## **Overview**
**SiloNeT** is a Python-based simulation framework designed for **offline localization testing** in vehicle networks. It allows students and researchers to experiment with different localization strategies, estimating unknown vehicle positions using known **anchor nodes** and distance measurements.

The simulation provides:
1. **Network Topology Visualization**: Displays vehicle positions, communication links, and distance constraints.
2. **Localization Results**: Compares estimated positions with actual locations to evaluate algorithm accuracy.

---

## **Features**
1. **Configurable Network Simulation**:
   - Adjustable number of vehicles, communication range, and simulation area.
   - Ensures vehicles maintain sufficient connectivity for localization.
2. **Graphical Representation**:
   - Visualizes actual and estimated vehicle positions.
   - Highlights anchor nodes and estimated locations.
3. **Custom Localization Algorithms**:
   - Students and researchers can implement and test their own localization strategies.
   - Algorithms must conform to a predefined interface.
4. **Accuracy Evaluation**:
   - Computes localization accuracy by comparing estimated and actual positions.
   - Generates reports and visualizations to analyze performance.

---

## **Project Structure**
```
D:.
│   main.py                 # Main script for running the simulation
│   readme.md               # Documentation
│
├───output
│       localization_result.png   # Visualization of estimated positions
│       network_topology.png      # Visualization of actual network topology
│
├───simulator
│   │   network_simulator.py      # Manages vehicle placement and network structure
│   │   visualization.py          # Handles graph visualization
│   └───__pycache__               # Python cache directory
│
└───student_algorithms
    │   beliefPropagationLocalization.py  # Belief propagation-based localization
    │   trilateration_offline_localization.py  # Trilateration-based localization
    │   sample_algorithm.py  # Example localization algorithm
```

---

## **Setup and Installation**

### **1. Prerequisites**
Ensure you have **Python 3.7 or higher**, then install dependencies:
```bash
pip install numpy matplotlib scipy
```

---

### **2. Running the Simulation**
1. **Navigate to the project directory**:
   ```bash
   cd D:.
   ```
2. **Run the simulation**:
   ```bash
   python main.py
   ```
   This will:
   - Generate the vehicle network and display its topology.
   - Apply an offline localization algorithm.
   - Save and display localization results.

---

## **Implementing a Custom Localization Algorithm**
Students should place their localization algorithms in the `/student_algorithms` folder and follow these steps:

### **Step 1: Create a New Algorithm**
- Create a new Python file (e.g., `custom_localization.py`) in the `student_algorithms` directory.
- Implement a class with the following structure:
  ```python
  class CustomLocalization:
      def locate_other_vehicles(self, known_positions, distances):
          """
          Parameters:
              known_positions (dict): Anchor nodes with known positions.
              distances (dict): Distance constraints between vehicles.
          Returns:
              dict: Estimated positions of unknown vehicles. Format: {vehicle_id: (x, y)}.
          """
          estimated_positions = {}
          # Implement your localization logic here
          return estimated_positions
  ```

### **Step 2: Update `main.py` to Use Your Algorithm**
Modify the **import statement** in `main.py` to use your algorithm:
```python
# Replace this line to switch the localization strategy
from student_algorithms.custom_localization import OfflineLocalization
```
Ensure the script initializes the correct class:
```python
localization = OfflineLocalization()
```

⚠️ **Important**: Do not modify other sections of `main.py` unless necessary.

---

## **Customization Options**
Modify the simulation parameters in `main.py` to adjust the number of vehicles, communication range, and area size:
```python
sim = NetworkSimulator(num_vehicles=15, comm_range=80, area_size=150, num_anchors=4)
```
- **num_vehicles**: Total number of vehicles.
- **comm_range**: Maximum distance within which vehicles can communicate.
- **area_size**: Size of the simulated environment.
- **num_anchors**: Number of vehicles with known positions.

---

## **Output Files**
After running the simulation, results are saved in the `/output` directory:
- **`network_topology.png`**: Displays the actual positions of vehicles and their communication links.
- **`localization_result.png`**: Shows estimated positions compared to actual locations.

---

## **Example Workflow**
1. **Implement Your Algorithm**:
   - Save it in `/student_algorithms/`.
2. **Update `main.py`**:
   - Modify the import statement to use your algorithm.
3. **Run the Simulation**:
   ```bash
   python main.py
   ```
4. **Analyze the Results**:
   - Compare the actual and estimated positions.
   - Evaluate the localization accuracy.

---

## **Notes for Students & Researchers**
1. **Modify Only the Algorithm Selection**:
   - Only change the **import statement** in `main.py` to switch localization strategies.
2. **Handle Errors and Edge Cases**:
   - Ensure your algorithm works with incomplete or missing distance data.
3. **Evaluate Algorithm Performance**:
   - Accuracy is computed automatically.
   - Adjust parameters to test different scenarios.
4. **Add Comments and Solve Issues**:
   - Ensuring clarity, accuracy, and ease of use is our goal.
   - After trying the code, apply your revisions and fork the repo.
   - Wish you all the best. 🚀
