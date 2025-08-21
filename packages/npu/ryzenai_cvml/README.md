# AMD Ryzen AI CVML

[![NPU](https://img.shields.io/badge/ryzenai-npu-blue)](#)

---

The Ryzen AI CVML libraries build on top of the Ryzen AI drivers and execution infrastructure to provide powerful AI capabilities to C++ applications without having to worry about training specific AI models and integrating them to the Ryzen AI framework.

## Building and Running the Docker Container

The docker image **must be built on top of the XDNA docker** which contains the installed drivers and runtime to interact with the NPU. Make sure to compose the ryzers together as follows:

```bash
ryzers build xdna ryzenai_cvml
ryzers run
```

The default test will run a sample depth perception app.

---

For further details, refer to the official [Ryzen AI CVML documentation](https://github.com/amd/RyzenAI-SW/tree/main/Ryzen-AI-CVML-Library).
