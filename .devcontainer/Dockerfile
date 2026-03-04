FROM ubuntu:noble

ARG USERNAME=kpaam
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# preventing to ask for user input
ENV DEBIAN_FRONTEND=noninteractive

# Delete user if it exists in container
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Install system packages
RUN apt-get update \
    && apt-get install -y \
    sudo \ 
    wget \
    git \
    zsh \
    curl \
    ca-certificates \
    bash-completion \
    build-essential \
    python-is-python3 \
    python3-pip \ 
    python3.12-venv \
    # GUI / OpenGL dependencies
    libgl1 \
    libgl1-mesa-dri \
    mesa-utils \
    libx11-xcb1 \
    libxrender1 \
    libxi6 \
    libxrandr2 \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME 

# Set zsh as default
RUN chsh -s /usr/bin/zsh $USERNAME

RUN mkdir -p /home/$USERNAME/projects
RUN chown -R $USERNAME:$USER_GID /home/$USERNAME/projects

USER $USERNAME
WORKDIR /home/$USERNAME/projects

# Make zsh look nicer
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# Zsh plugins
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' ~/.zshrc


CMD ["/usr/bin/zsh", "-l"]
