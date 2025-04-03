#!/bin/bash

# # Source the specified environment variables and scripts into ~/.bashrc
sudo apt install -y ssh-client 

# Start ssh-agent
eval $(ssh-agent -s)

# Add ssh key to agent
ssh-add

# Notify the user of completion
echo ' '
echo 'Setup complete! Resource your .bashrc with "source ~/.bashrc"'
