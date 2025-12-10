# Pywin

In Git Bash, you can install pyenv-win using the Git method:

  git clone https://github.com/pyenv-win/pyenv-win.git "$HOME/.pyenv"

Then add these lines to your ~/.bashrc (or ~/.bash_profile):

  export PYENV="$HOME/.pyenv/pyenv-win"
  export P$PYENV/shims:$PAATH="$PYENV/bin:TH"

Reload your shell:

  source ~/.bashrc

Now you can use pyenv:

```bash
# List available versions
  pyenv install --list | grep 3.9

# Install Python 3.9.13
  pyenv install 3.9.13

# Set it locally for your Project AirSim directory
  cd /f/Users/tomtz/dev/clones/ProjectAirSim
  pyenv local 3.9.13

# Verify
  python --version  # Should show 3.9.13

# Create venv
  python -m venv airsim-venv
  source airsim-venv/Scripts/activate
  pip install -e client/python/projectairsim
```

Note: On Windows/Git Bash, the venv activation script is at Scripts/activate (not bin/activate like on Linux).

