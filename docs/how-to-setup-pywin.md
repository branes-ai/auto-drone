# Pywin

## Installing via Powershell

The pyenv tool itself doesn’t support Windows, but `pyenv-win` is a pyenv port for Windows. The recommended way 
to install this tool on Windows is to run the following command in a PowerShell terminal:

```ps
# Open a Powershell as Administrator.
PS> Invoke-WebRequest -UseBasicParsing -Uri "https://raw.githubusercontent.com/pyenv-win/pyenv-win/master/pyenv-win/install-pyenv-win.ps1" -OutFile "./install-pyenv-win.ps1"; &"./install-pyenv-win.ps1"
```

## Installing on Git bash

In Git Bash, you can install pyenv-win using the Git method:

```bash
  git clone https://github.com/pyenv-win/pyenv-win.git "$HOME/.pyenv"
```

Then add these lines to your ~/.bashrc (or ~/.bash_profile):

```bash
  export PYENV="$HOME/.pyenv/pyenv-win"
  export P$PYENV/shims:$PAATH="$PYENV/bin:TH"
```

Reload your shell:

```bash
  source ~/.bashrc
```

## Usage

Here are some useful pyenv management commands:

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

**Note**: On Windows/Git Bash, the venv activation script is at Scripts/activate (not bin/activate like on Linux).

