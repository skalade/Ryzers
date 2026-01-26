# RAI - Robotec AI Framework

RAI is a flexible AI agent framework to develop and deploy Embodied AI features for robots.

## Build

RAI is a composable package that requires ROS 2 as a base:

```bash
ryzers build ros rai
```

## Run

```bash
# Run the test script
ryzers run

# Interactive shell
ryzers run bash

# Run with Ollama for local LLM
ryzers build ros rai ollama
ryzers run bash
```

## Configuration

Set your LLM API key before running:

```bash
export OPENAI_API_KEY=your-key-here
# or use Ollama for local inference
```

Initialize RAI configuration:

```bash
rai-config-init
```

## Documentation

- Official docs: https://robotecai.github.io/rai/
- GitHub: https://github.com/RobotecAI/rai
