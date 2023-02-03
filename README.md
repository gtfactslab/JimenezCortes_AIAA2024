# JimenezCortes_CCTA2023
This code supplements the CCTA submission  "Prescribed-Time Control Barrier Functions for Semiautonomous Navigation" by Carmen Jimenez Cortes and Samuel Coogan. It consists of a Simulink project to run the example in Microsoft Flight Simulator.

## Requirements
1. Matlab and Simulink
2. Microsoft Flight Simulator (MSFS)
3. SimConnect Toolbox for Simulink

### SimConnect Toolbox
The simulink file PC_MSFS_STATION.slx uses two blocks from the SimConnect Toolbox (https://github.com/aguther/simconnect-toolbox). This library needs to be installed for running the example.

## Project Structure
The project PTS_SIM_CCTA_EXAMPLE consists of 6 folders:
1. BUS DEFINITIONS: BusDefinition.m - This file generates automatically the buses during the initializacion process. It doesn't need to be edited.
2. CONFIGURATION: CONFIG_MSFS.m - Master file. This file needs to be run before the Simulink file, it will call all the configuration files, create and initialize the buses and launch the PC_MSFS_STATION.slx.
3. EXTRA: Additional files - not needed for the example
4. SIMULINK: PC_MSFS_STATION.slx - Simulator file, once the simulation is launched in MSFS, running this file will launch the Navigation Control with Prescribed-Time Control Barrier Functions
5. SOFTWARE_COMPONENTS
  5.1. CONTROL: CONFIG_CONTROL.m - File with parameter, input, output, and target variables of the control bus
  5.2. MODEL: CONFIG_MODEL.m - File with parameter, input, output, and target variables of the model bus
6. TEST_FILES: CCTA_TRAJECTORY_DATA.mat - Recorded data from the experiment, after loading it into the workspace SATELLITE_VIEW_TRAJECTORY.m can be used to visualize it on a satellite view
