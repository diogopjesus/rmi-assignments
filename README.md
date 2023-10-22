# How to use
Add this repository as a submodule of [ciberRatoTools](git@github.com:iris-ua/ciberRatoTools.git).
```bash
git submodule add git@github.com:diogopjesus/rmi-assignments.git agent
```

Run the build.sh script to set up the environment.
```bash
./build.sh
```

Run the run.sh script with the assigned flag to run the agent for the specific challenge.
```bash
./run.sh -c1    # run the agent for the first challenge
./run.sh -c2    # run the agent for the second challenge
./run.sh -c3    # run the agent for the third challenge
```

