

# Autonomous Car Project


## Project Overview
This project aims to develop an autonomous car capable of navigating through various environments using computer vision, sensor fusion, and machine learning techniques. The car can detect and avoid obstacles, follow lanes, recognize traffic signs, and make driving decisions in real-time.

## Features
- **Lane Detection**: Identify and follow road lanes using computer vision techniques.
- **Object Detection**: Detect obstacles, pedestrians, and other vehicles.
- **Traffic Sign Recognition**: Recognize and respond to traffic signs.
- **Path Planning**: Plan and follow the safest route based on sensor data.
- **Sensor Fusion**: Combine data from multiple sensors (e.g., cameras, LiDAR, radar) for accurate perception.

## Technologies Used
- **Programming Languages**: Python, C++
- **Frameworks**: TensorFlow, OpenCV, ROS (Robot Operating System)
- **Simulation**: CARLA, Gazebo
- **Machine Learning**: Convolutional Neural Networks (CNNs), Support Vector Machines (SVMs)
- **Sensor Technologies**: LiDAR, Radar, GPS, IMU (Inertial Measurement Unit)
- **Other Tools**: Docker, Git

## Installation

### Prerequisites
- Python 3.8+
- Docker
- ROS Noetic
- CARLA Simulator
- CUDA (for GPU support)

### Steps
1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/autonomous-car.git
   cd autonomous-car
   ```

2. **Set up a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Install ROS dependencies**:
   ```bash
   sudo apt-get update
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. **Run Docker container** (if applicable):
   ```bash
   docker-compose up
   ```

## Usage

### Running the Simulation
1. Launch the CARLA simulator.
2. Run the ROS nodes:
   ```bash
   roslaunch autonomous_car simulation.launch
   ```
3. Start the autonomous driving:
   ```bash
   python scripts/autonomous_drive.py
   ```

### Real-World Deployment
1. Connect the sensors and hardware.
2. Start the ROS nodes for data collection and processing.
3. Initiate the autonomous driving system.

## Project Structure
```
autonomous-car/
│
├── data/                # Dataset for training and testing
├── docs/                # Documentation files
├── models/              # Pretrained and custom models
├── scripts/             # Python scripts for data processing and model training
├── src/                 # Source code for ROS nodes
├── tests/               # Test cases
├── Dockerfile           # Dockerfile for containerized setup
├── requirements.txt     # Python dependencies
├── setup.py             # Setup script for the package
└── README.md            # This README file
```

## Data
The project uses datasets from various sources, including:
- **KITTI Dataset**: For object detection and depth estimation.
- **COCO Dataset**: For traffic sign and pedestrian detection.
- **Custom Datasets**: Collected using the car's sensors in real-world environments.

## Model Training
The models are trained using a combination of supervised learning techniques, with data augmentation applied to enhance performance in diverse conditions. For example, the lane detection model uses a convolutional neural network trained on annotated road images.

### Training Procedure
1. Prepare the dataset and split it into training, validation, and test sets.
2. Train the models using `train_model.py`:
   ```bash
   python scripts/train_model.py --config configs/lane_detection.yaml
   ```

3. Evaluate the models:
   ```bash
   python scripts/evaluate_model.py --model models/lane_detection.pth
   ```

## Simulation
The CARLA simulator is used to validate the car's performance in various driving scenarios. The simulation environment replicates urban and rural road conditions, providing a safe and controlled setting for testing.

### Running Simulations
- **Basic Driving**: Test the car's ability to follow lanes and avoid obstacles.
- **Complex Scenarios**: Simulate challenging situations like intersections, roundabouts, and traffic.

## Testing
Automated tests are included to ensure the reliability of the system. Unit tests cover individual components, while integration tests validate the interactions between them.

### Running Tests
```bash
pytest tests/
```

## Contributing
Contributions are welcome! Please read the [CONTRIBUTING.md](CONTRIBUTING.md) file for guidelines on how to contribute to this project.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- Thanks to the open-source community for providing tools and datasets.
- Special thanks to the contributors who helped in the development of this project.

