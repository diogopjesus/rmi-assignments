# How to use
Add this repository as a submodule of [ciberRatoTools](git@github.com:iris-ua/ciberRatoTools.git).
```bash
git submodule add git@github.com:diogopjesus/rmi-assignment1.git agent
```

Run the build.sh script to set up the environment.
```bash
./build.sh
```

Run the run.sh script with the assigned flag to run the agent for the specific challenge.
```bash
./run.sh -c[1|2|3] [-h host | -r robname | -p pos | -f outfile ]
```
