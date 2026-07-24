# ROSCon 2026 AI PC Workshop

Hands-on materials for the ROSCon 2026 workshop on Embodied AI on AMD Ryzen AI hardware.

Each section ships a standalone `Dockerfile` (plain `docker build`, no `ryzers`
CLI) and its own `README.md`. Drop workshop notebooks into a section's `notebooks/`.

| Section | Description |
|---------|-------------|
| [`finetuning`](finetuning) | [MolmoAct2](https://huggingface.co/collections/allenai/molmoact) VLA ROCm environment. |
| [`local_inference`](local_inference) | [ROS 2](https://docs.ros.org/) + [O3DE](https://o3de.org/) + [RAI](https://github.com/RobotecAI/rai) served by [Lemonade](https://lemonade-server.ai/). |
