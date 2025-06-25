# DynNPC: Finding More Violations Caused by ADS in Simulation Testing through Dynamic NPC Behavior Generation

In this work, we propose a novel scenario-based testing framework, DynNPC, to generate more violation scenarios caused by the ADS. Specifically, DynNPC allows NPC vehicles to dynamically generate behaviors using different driving strategies during simulation execution according to traffic signals and the realtime behavior of the Ego vehicle. We compare DynNPC with four state-of-the-art scenario-based testing approaches. Our evaluation has demonstrated the effectiveness and efficiency of DynNPC in finding more violation scenarios caused by the ADS.

![Overview Image](img/Overview.png)

**Note: This is the first version of DynNPC, and we plan to release updates in the future.**

## Experiment Environment
We conduct all the experiments on a system with the following specifications:

- Operating System: Ubuntu 22.04 LTS
- GPU: NVIDIA GeForce RTX 3090
- CPU: Intel Core i9-13900K (32) @ 5.500GHz
- Memory: 64GB




## Installation & Run DynNPC

### 1. Install the LGSVL Simulator
LGSVL simulator can be installed from https://github.com/lgsvl/simulator .We are using the latest version, 2021.3.\
LGSVL has made the difficult decision to suspend active development of SVL Simulator, as of January 1, 2022. The cloud had stopped running on June 30, 2022.Therefore, we use SORA-SVL to build our own server as a replacement.SORA-SVL can be installed from https://github.com/YuqiHuai/SORA-SVL  
### 2. Install SVL Python API
```
git clone https://github.com/lgsvl/PythonAPI.git
```
Following the installation procedure at [https://github.com/lgsvl/PythonAPI](https://github.com/lgsvl/PythonAPI)
### 3. Install Apollo 7.0
clone source code
```sh
$ git checkout r7.0.0
$ git clone https://github.com/ApolloAuto/apollo.git
```
pull docker image and enter the container(This step may take a long time)
```sh
$ sudo bash ./docker/scripts/dev_start.sh
$ sudo bash ./docker/scripts/dev_into.sh
```
build Apollo
```sh
sudo ./apollo.sh build_opt
```
start dreamviewer
```sh
sudo bash scripts/bootstrap.sh
```
After completion, open localhost:8888 and you can see the Dreamviewer Interface.\
bridge Apollo with LGSVL
```sh
bash scripts/bridge.sh
``` 
### 4. Run DynNPC

DynNPC is a novel simulation testing approach, to generate adversarial scenarios on main lanes (e.g., urban roads and highways). DynNPC allows NPC vehicles to dynamically interact with the EGO vehicle and regulates the behaviors of NPC vehicles, finding more violation scenarios caused by the EGO vehicle more quickly.

To set up DynNPC:

1. Clone the repository:
   ```sh
   git clone https://github.com/DynNPC/DynNPC.git
   ```

2. Navigate to the project directory:
   ```sh
   cd DynNPC/
   ```

3. Install the required dependencies:
   ```sh
   pip install -r requirements.txt
   ```

4. Configure DynNPC to work with the specific ADS and simulation environment：
      - *Straight Road* : Modify the `src/configs/config.yaml` file to specify the simulation environment and the ADS to be tested.
      - *Intersection* : Modify the `src/configs/config_int.yaml` file to specify the simulation environment and the ADS to be tested.
      - Modify the `src/settings` file to specify the map and scenario to be tested.

5. Run DynNPC:
   ```sh
   python src/main.py
   ```

**Note:** Records of our experiments can be found in the `data` directory.




## Baselines

### 1. AV-FUZZER

AV-Fuzzer is a well-known search-based testing technique, which uses genetic algorithm to evolve NPC vehicles’ movements to expose safety violations of ADSs. It can be found at [https://github.com/cclinus/AV-Fuzzer](https://github.com/cclinus/AV-Fuzzer).

To set up AV-FUZZER:

1. Clone the repository:
   ```sh
   git clone https://github.com/cclinus/AV-Fuzzer.git
   ```

2. Follow the installation instructions provided in the repository's README file.

3. Configure AV-FUZZER to work with the specific ADS and simulation environment.

4. Run AV-FUZZER according to the usage instructions in the repository.

Note: Make sure your system meets the requirements specified in the AV-FUZZER repository before installation.


### 2. ADFuzz

AutoFuzz is a novel fuzzing testing technique
guided by a neural network evolutionary search method. It can be found at [https://github.com/AIasd/ADFuzz](https://github.com/AIasd/ADFuzz).

To set up ADFuzz:

1. Clone the repository:
   ```sh
   git clone https://github.com/AIasd/ADFuzz.git
   ```

2. Follow the installation instructions provided in the repository's README file.

3. Configure ADFuzz to work with the specific ADS and simulation environment.

4. Run ADFuzz according to the usage instructions in the repository.

Note: Ensure that your system meets the requirements specified in the ADFuzz repository before installation.


### 3. CRISCO
CRISCO is "A Road Structure-based Approach to Test Planning Module of Autonomous Driving System". It can be found at [https://github.com/criscotesting/CRISCO](https://github.com/criscotesting/CRISCO).  
(Now the latest version is available at [https://gitlab.com/tianhaoxiang20/racer](https://gitlab.com/tianhaoxiang20/racer))

To set up CRISCO:

1. Clone the repository:
   ```sh
   git clone https://gitlab.com/tianhaoxiang20/racer.git
   ```

2. Follow the installation instructions provided in the repository's README file.

3. Configure CRISCO to work with the specific ADS and simulation environment.

4. Run CRISCO according to the usage instructions in the repository.

Note: Make sure your system meets the requirements specified in the CRISCO repository before installation.

### 4. BehAVExplor
BehAVExplor is a novel behavior-guided fuzzing technique to explore the different behaviors of the ego vehicle (i.e., the vehicle controlled by the ADS under test) and detect diverse violations. It can be found at [https://github.com/MingfeiCheng/BehAVExplor](https://github.com/MingfeiCheng/BehAVExplor).  


To set up BehAVExplor:

1. Clone the repository:
   ```sh
   git clone https://github.com/MingfeiCheng/BehAVExplor
   ```

2. Follow the installation instructions provided in the repository's README file.

3. Configure BehAVExplor to work with the specific ADS and simulation environment.

4. Run BehAVExplor according to the usage instructions in the repository.

Note: Make sure your system meets the requirements specified in the BehAVExplor repository before installation.

### 5. DoppelTest
DoppelTest is a Python framework implemented to evaluate a novel autonomous driving software (ADS) testing approach. It can be found at [https://github.com/Software-Aurora-Lab/DoppelTest](https://github.com/Software-Aurora-Lab/DoppelTest).
