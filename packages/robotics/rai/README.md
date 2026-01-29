# RAI - Robotec AI Framework

RAI is a flexible AI agent framework to develop and deploy Embodied AI features for robots.

## Build

We build RAI on top of the ROS Ryzer as a dependency:

```bash
ryzers build ros rai
```

## Run

```bash
# Run the test script
ryzers run
```

The default test will verify the environment and installed packages.

## Manipulation demo

To run the manipulation demo we need to add O3DE to the build 

```bash
ryzers build ros o3de rai
ryzers run /ryzers/manipulation_demo.sh
```

This will launch a streamlit app where you can chat with the robot agent and ask it to perform tasks on the table bench. Note that the simulation can take a while to load, give it a minute or two until the items spawn on the table. 

Once everything is loaded you can use the streamlit chat window to tell the robot agent to, for example, "stack the cubes".

<img src="images/manipulation_demo.gif">

## Documentation

- Official docs: https://robotecai.github.io/rai/
- GitHub: https://github.com/RobotecAI/rai
