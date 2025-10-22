import re
from pathlib import Path

# Input New dockerfile
INPUT_FILE = "Dockerfile.gpu"
# 输出：合并后的 Dockerfile
OUTPUT_FILE = "Dockerfile"

with open(INPUT_FILE, "r") as f:
    content = f.read()



# Step 1. Insert necessary ENV
if "HSA_OVERRIDE_GFX_VERSION" not in content:
    content = re.sub(r"(ENV\s+DEBIAN_FRONTEND=.*\n)", r"\1ENV HSA_OVERRIDE_GFX_VERSION=11.0.0\n", content)

if "PYTHONUNBUFFERED" not in content:
    content = re.sub(r"(ENV\s+DEBIAN_FRONTEND=.*\n)", r"\1ENV PYTHONUNBUFFERED=1\n", content)
    
if "PYTHONPATH" not in content:
    content +='''
# Make ROS environment variables available to all users and shells
#ENV ROS_DISTRO=kilted
ENV AMENT_PREFIX_PATH=/opt/ros/$ROS_DISTRO:$AMENT_PREFIX_PATH
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.12/site-packages:$PYTHONPATH
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /ryzers/notebooks/vlm_ros/install/setup.bash" >> /etc/bash.bashrc

    '''

# Step 2. Insert jupyterhub rely
if "jupyterhub" not in content:
    content += """
# === JupyterHub Integration ===
RUN pip install --no-cache-dir --break-system-packages \\
    jupyterhub==4.0.2 \\
    jupyterlab==4.0.9 \\
    notebook==7.0.6 \\
    ipywidgets==8.1.1 \\
    ipykernel==6.27.1
"""

# Step 3. add user jovyan
if "useradd -m" not in content and "jovyan" not in content:
    content += """
# === Create jovyan user for JupyterHub ===
ARG NB_USER=jovyan
ARG NB_UID=1000
ARG NB_GID=100

RUN if getent passwd 1000 > /dev/null; then \\
        olduser=$(getent passwd 1000 | cut -d: -f1); \\
        echo "Removing existing user: $olduser (UID 1000)"; \\
        userdel -r $olduser || true; \\
    fi && \\ 
    useradd -m -s /bin/bash -N -u $NB_UID -g $NB_GID $NB_USER && \\
    echo "$NB_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \\
    usermod -aG video,render,kvm root
"""

# Step 4. add jupyterhub boot up
if "entrypoint.sh" not in content:
    content += """
# === EntryPoint for JupyterHub singleuser ===
RUN echo '#!/bin/bash' > /entrypoint.sh && \\
    echo 'chmod 666 /dev/kfd 2>/dev/null || true' >> /entrypoint.sh && \\
    echo 'chmod 666 /dev/dri/renderD* 2>/dev/null || true' >> /entrypoint.sh && \\
    echo 'export USER=jovyan' >> /entrypoint.sh && \\
    echo 'export SHELL=/bin/bash' >> /entrypoint.sh && \\
    echo 'export HSA_OVERRIDE_GFX_VERSION=11.0.0' >> /entrypoint.sh && \\
    echo 'source /opt/ros/kilted/setup.bash' >> /entrypoint.sh && \\
    echo 'exec python3 -m jupyterhub.singleuser --ip=0.0.0.0 --port=8888 --allow-root "$@"' >> /entrypoint.sh && \\
    chmod +x /entrypoint.sh
"""

# Step 5. Change sudo path

if "secure_path" not in content:
    content += '''
# === Fix sudo PATH issue ===
ENV PATH="/opt/venv/bin:${PATH}" 
ENV VIRTUAL_ENV="/opt/venv" 

RUN echo 'Defaults env_keep += "PATH VIRTUAL_ENV"' >> /etc/sudoers && \\
    sed -i '/secure_path/d' /etc/sudoers && \\
    echo 'Defaults secure_path="/opt/venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"' >> /etc/sudoers

'''

# Step 6. add CMD、EXPOSE、USER
if "EXPOSE 8888" not in content:
    content += "\nEXPOSE 8888\n"
if "USER jovyan" not in content:
    content += "\nUSER jovyan\n"
if "CMD" not in content:
    content += "\nCMD [\"/bin/bash\", \"/entrypoint.sh\"]\n"

# write out
Path(OUTPUT_FILE).write_text(content)
print(f"✅ Merged Dockerfile written to {OUTPUT_FILE}")

