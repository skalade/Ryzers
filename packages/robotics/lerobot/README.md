## LeRobot Docker Setup

Contains everything you need to build & run a ROCm-enabled LeRobot container.

### 1. Reference & Config
- **Guide:** Hugging Face “Imitation Learning on Real-World Robots”  
  <https://huggingface.co/docs/lerobot/en/il_robots>  
- **`config.yaml`:**  
  - Pay attention to the TODO items - add your own `HF_TOKEN` from Hugging Face, and map your robot and video devices accordingly. Step 2. makes this simpler and more reproducible.

---

### 2. USB device mapping (optional)

Your serial and video devices may change indexes in `/dev` between sessions or when you re-plug them. To save the hassle of trying to figure out the device index every time we can map them to consistent named pointers by their serial IDs.

#### USB-serial mapping

1. **Record serial IDs**
   ```bash
   ls -l /dev/serial/by-id/
   ```
2. **Create or edit** `99-usb-serial.rules`:
   ```ini
   SUBSYSTEM=="tty", ATTRS{serial}=="<leader-serial>",   SYMLINK+="ttyACM_leader"
   SUBSYSTEM=="tty", ATTRS{serial}=="<follower-serial>", SYMLINK+="ttyACM_follower"
   ```
   Replace `<leader-serial>` and `<follower-serial>` with the values from step 1.
3. **Install & update devices**
   ```bash
   sudo cp 99-usb-serial.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

#### USB-webcam mapping

1. **List webcam details**

```bash
for dev in /dev/video*; do
    echo "=== $dev ==="
    udevadm info --query=all --name=$dev | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|DEVPATH"
done
```

2. **Create or edit** `/etc/udev/rules.d/99-usb-video.rules`. You can use `ID_SERIAL_SHORT` from step 1. as the serial number.

```ini
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", ATTRS{serial}==<webcam1-serial-short>, SYMLINK+="webcam_top"
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", ATTRS{serial}==<webcam2-serial-short>, SYMLINK+="webcam_front"
```

3. **Install & update devices**
   ```bash
   sudo cp 99-usb-video.rules /etc/udev/rules.d/99-usb-video.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

Your cameras, model files, and USB-serial ports will now mount exactly as specified. The ports will be available as `/dev/ttyACM_leader` and `/dev/ttyACM_follower`.

---

### 3. Build & Run

To verify the lerobot installation simply run the built ryzer -- this will run one of the lerobot examples as a test.
```bash
ryzers build lerobot
ryzers run
```

To run an interactive session start with bash:
```bash
ryzers run bash
```

---

### 4. Collecting dataset

For this example we use the [koch v1.1](https://github.com/jess-moss/koch-v1-1) leader and follower arms, however you can easily swap them with different ones by changing the `robot.type` and `teleop.type` parameters.

#### Teleoperation (optional)

Before starting any data collection tasks you can make sure your setup works by running teleoperation.

```bash
lerobot-teleoperate \
    --robot.type=koch_follower \
    --robot.port=/dev/ttyACM_follower \
    --robot.id=my_awesome_follower_arm \
    --teleop.type=koch_leader \
    --teleop.port=/dev/ttyACM_leader \
    --teleop.id=my_awesome_leader_arm \
    --fps=20 \
    --robot.cameras="{ top: {type: opencv, index_or_path: /dev/webcam_front, width: 640, height: 480, fps: 20}, front: {type: opencv, index_or_path: /dev/webcam_top, width: 640, height: 480, fps: 20}}" \
    --display_data=true
```

#### Record a dataset

Make sure to set `robot.cameras` with the resolution and index according to your setup. Adjust dataset parameters like number of episodes or durations as needed for your task.

```bash
lerobot-record \
    --robot.type=koch_follower \
    --robot.port=/dev/ttyACM_follower \
    --robot.id=my_awesome_follower_arm \
    --teleop.type=koch_leader \
    --teleop.port=/dev/ttyACM_leader \
    --teleop.id=my_awesome_leader_arm \
    --robot.cameras="{ top: {type: opencv, index_or_path: /dev/webcam_front, width: 640, height: 480, fps: 20}, front: {type: opencv, index_or_path: /dev/webcam_top, width: 640, height: 480, fps: 20}}" \
    --play_sound=False \
    --dataset.repo_id=${HF_USER}/push_cube \
    --dataset.num_episodes=30 \
    --dataset.single_task="keep cube in square" \
    --dataset.episode_time_s=10 \
    --dataset.reset_time_s=5 \
    --dataset.push_to_hub=False
```

### 5. Train a policy

Using the collected dataset you can use it to train a policy like [ACT](https://github.com/tonyzhaozh/act) or [pi0](https://www.physicalintelligence.company/blog/pi0). Depending on your dataset size you should be able to train a small policy like ACT within an hour on a Strix Halo iGPU.

```bash
lerobot-train \
    --dataset.repo_id=${HF_USER}/push_cube \
    --policy.type=act \
    --output_dir=/ryzers/mounted/outputs/train/push_cube_act \
    --job_name=push_cube_act \
    --policy.device=cuda \
    --policy.repo_id=${HF_USER}/push_cube_act \
    --steps=2000
```

### 6. Run inference

To deploy the model we re-use the `lerobot-record` command with a `policy.path` parameter set.

```bash
lerobot-record \
    --robot.type=koch_follower \
    --robot.port=/dev/ttyACM_follower \
    --robot.id=my_awesome_follower_arm \
    --robot.cameras="{ top: {type: opencv, index_or_path: /dev/webcam_front, width: 640, height: 480, fps: 20}, front: {type: opencv, index_or_path: /dev/webcam_top, width: 640, height: 480, fps: 20}}" \
    --dataset.repo_id=${HF_USER}/eval_push_cube \
    --dataset.single_task="keep cube in square" \
    --policy.path=/ryzers/mounted/outputs/train/push_cube_act2/checkpoints/last/pretrained_model/
```

## Troubleshooting

If there's a big difference between movemetns of the leader and follower you can re-run calibration:
```bash
lerobot-calibrate  --teleop.type=koch_leader     --teleop.port=/dev/ttyACM_leader     --teleop.id=my_awesome_leader_arm
lerobot-calibrate  --robot.type=koch_follower    --robot.port=/dev/ttyACM_follower    --robot.id=my_awesome_follower_arm
```

If you run into motor bus timeout issues, you may need to increase the number of communication retries, here's a oneliner to make that change from an interactive session:
```bash
find . -type f -name "*.py" -exec sed -i.bak 's/num_retry: int = 0/num_retry: int = 10/g' {} +
```