#!/bin/bash

# Check if there is a git directory...
if [ -d .git ]
then
    # There is a git directory. Pull from the repo.
    echo "Git repository found. Updating..."
    git pull

else
    # There is no directory. Clone from the repo.
    echo "Git repository not found. Pulling from https://github.com/AlexanderBeck0/GRACE.git..."
    git clone https://github.com/AlexanderBeck0/GRACE.git
fi

bash dependency_install.sh
bash install_submodules.sh