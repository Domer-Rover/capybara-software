#!/usr/bin/env bash
# add_dev_user.sh — onboard a developer onto the Jetson.
#
# Creates a Linux user with their own home, clones the repo into it, sets their
# git author, sources ROS in their shell, and generates an SSH key for GitHub.
#
# Usage (run on the Jetson as a sudoer):
#   sudo scripts/add_dev_user.sh <username> "<Full Name>" <git-email>
#
# You will be prompted for the new user's login password.
# GitHub no longer accepts account passwords for git; the SSH key printed at the
# end is what the user adds to https://github.com/settings/keys.
set -euo pipefail

GITHUB_ORG="Domer-Rover"
REPOS=(capybara-software gerbil-software phoenix-software)
WORK_DIR="domerrover"   # repos land in ~/domerrover/<repo>
ROS_SETUP="/opt/ros/humble/setup.bash"
GROUPS_TO_ADD="dialout,video"

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo." >&2
    exit 1
fi
if [[ $# -ne 3 ]]; then
    echo "Usage: sudo $0 <username> \"<Full Name>\" <git-email>" >&2
    exit 1
fi

USERNAME="$1"
FULLNAME="$2"
EMAIL="$3"
HOME_DIR="/home/$USERNAME"

# 1. Linux account
if id "$USERNAME" &>/dev/null; then
    echo "User $USERNAME already exists, skipping account creation."
else
    adduser --gecos "$FULLNAME" "$USERNAME"
fi
usermod -aG "$GROUPS_TO_ADD" "$USERNAME"

# 2. ROS in every login shell
if ! grep -q "$ROS_SETUP" "$HOME_DIR/.bashrc"; then
    echo "source $ROS_SETUP" >> "$HOME_DIR/.bashrc"
    chown "$USERNAME:$USERNAME" "$HOME_DIR/.bashrc"
fi

# 3. SSH key for GitHub
SSH_DIR="$HOME_DIR/.ssh"
KEY="$SSH_DIR/id_ed25519"
if [[ ! -f "$KEY" ]]; then
    sudo -u "$USERNAME" mkdir -p "$SSH_DIR"
    chmod 700 "$SSH_DIR"
    sudo -u "$USERNAME" ssh-keygen -t ed25519 -C "$EMAIL" -N "" -f "$KEY" -q
fi
# Trust github.com so the first clone doesn't prompt
sudo -u "$USERNAME" bash -c "ssh-keyscan -t ed25519 github.com 2>/dev/null >> '$SSH_DIR/known_hosts'"

# 4. Repo clones. The SSH key is not on GitHub yet, so clone over HTTPS
#    (public read works without auth) and point the push URL at SSH so
#    pushes work as soon as the key is added.
sudo -u "$USERNAME" mkdir -p "$HOME_DIR/$WORK_DIR"
for repo in "${REPOS[@]}"; do
    dest="$HOME_DIR/$WORK_DIR/$repo"
    if [[ -d "$dest/.git" ]]; then
        echo "$repo already cloned."
        continue
    fi
    sudo -u "$USERNAME" git clone "https://github.com/$GITHUB_ORG/$repo.git" "$dest"
    sudo -u "$USERNAME" git -C "$dest" remote set-url --push origin "git@github.com:$GITHUB_ORG/$repo.git"
done

# 5. Git author (global for this user, so it applies to any repo they clone)
sudo -u "$USERNAME" git config --global user.name "$FULLNAME"
sudo -u "$USERNAME" git config --global user.email "$EMAIL"

echo
echo "================================================================"
echo "User $USERNAME is set up."
echo
echo "Give them this public key to add at https://github.com/settings/keys :"
echo
cat "$KEY.pub"
echo
echo "Then they log in with:   ssh $USERNAME@$(hostname -I | awk '{print $1}')"
echo "Repos:                   ~/$WORK_DIR/{${REPOS[*]// /,}}"
echo "Note: group changes (serial ports, camera) apply on their first login."
echo "================================================================"
