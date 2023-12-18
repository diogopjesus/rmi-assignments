# Intelligent And Mobile Robotics
Assignments developed for the Intelligent And Mobile Robotics class.
The assignments can be found on **archive**.
# How to use
The robotic agents were developed using the CiberRato simulation environment.
The easiest way to test this code is to add this repository as a subomdule of a CiberRatoTools repository.
First, change directory into the ciberRatoTools repository, and then run the following command:
```
git submodule add git@github.com:diogopjesus/rmi-assignments.git agent
```
Then, go into the **agent** directory and build the agents:
```
./build.sh
```
Finally, run the agent (**X** is a value between 1 and 4, corresponding to the challenge):
```
./run.sh -cX [-h host | -r robname | -p pos | -f outfile]  # X between 1 and 4
```
To test the scores of the generated best paths and perceived maps, run the following:
```
./test-run.sh -cX [-f outfile]  # X between 1 and 4
```
To automate runs, you can apply the **automateRuns_CiberRatoTools.patch** (available on patches) to the ciberRatoTools repository, and then run the **loop.sh** script (**do not forget to re-build the simulator)**.
```
./loop.sh -cX # X between 1 and 4
```

## Assignment 1
3 different robotic agents, each one that aim to solve one of the following problems:
- Localization,
- Mapping,
- Planning.

Programmed in Python.

**Final Grade:** 17.1/20
  
## Assignment 2
Develop a single agent that solves the three problems presented in assignment 1 with only a line sensor, a noisy compass and the movement model.

To approximate the values of the compass a Kalman filter was implemented.

Programmed in C++.

**Final Grade:** ??/20
