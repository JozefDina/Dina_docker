# KPAaM_docker
Docker development environment based on Ubuntu 24 for students with GUI support. This is only supported on Linux, tested on Ubuntu.

## Prerequisites
1. **Docker** — [Install Docker Engine](https://docs.docker.com/engine/install/ubuntu/)
2. **VS Code** — [Download](https://code.visualstudio.com/)
3. **Dev Containers extension** for VS Code:
   ```bash
   ext install ms-vscode-remote.remote-containers
   ```

## Getting Started
### 1. Clone the repository
```bash
git clone https://github.com/PeterJan1/KPAaM_docker.git
cd KPAaM_docker
```

### 2. Open in VS Code
```bash
code .
```

### 3. Open the Dev Container
When you open the folder in VS Code, you should see a notification in the bottom-right corner:

> **Folder contains a Dev Container configuration file. Reopen folder to develop in a container.**

Click **"Reopen in Container"**.

Alternatively, open the Command Palette (`Ctrl+Shift+P`) and run:
```
Dev Containers: Reopen in Container
```

VS Code will build the Docker image and start the container. This may take a few minutes on the first run.


### 4. Verify the setup
Open a terminal inside VS Code (`Ctrl+``) and run:
```bash
python --version
```

## Rebuilding the Container
If the `Dockerfile` or `devcontainer.json` has been modified, you need to rebuild:
1. Open the Command Palette (`Ctrl+Shift+P`)
2. Run: **Dev Containers: Rebuild Container**

To rebuild without cache (clean build):
1. Open the Command Palette (`Ctrl+Shift+P`)
2. Run: **Dev Containers: Rebuild Container Without Cache**



## Project Structure
```
KPAaM_docker/
├── .devcontainer/
│   ├── devcontainer.json    # Container configuration
│   └── Dockerfile           # Docker image definition
├── projects/                # Mounted workspace (your code goes here)
└── README.md
```


## GUI Applications (PyBullet, matplotlib, etc.)
This container is configured for X11 forwarding, which allows GUI applications running inside the container to display windows on your host machine.
### How it works
The `devcontainer.json` includes:
- `initializeCommand: "xhost +local:docker"` — automatically grants the container access to your X11 display when the container starts.
- The host's X11 socket (`/tmp/.X11-unix`) is mounted into the container.
- The `DISPLAY` environment variable is passed through from the host.


## VS Code Extensions (included automatically)
The following extensions are installed inside the container:
- **clangd** — C/C++ language support
- **CMake Tools** — CMake build integration
- **Python** — Python language support
- **Jupyter** — Notebook support
- **Markdown All in One** — Markdown editing
- **vscode-icons** — File icons

## Troubleshooting
### GUI window doesn't appear / script hangs
1. **Check DISPLAY is set** inside the container:
   ```bash
   echo $DISPLAY
   ```
   It should not be empty. If it is, try reopening the container.

2. **Check X11 permissions** on the host:
   ```bash
   xhost +local:docker
   ```
3. Rebuild and reopen the container