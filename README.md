# MPC Differential-Drive Robot

Model Predictive Control (MPC) for a differential-drive robot using `acados`.

This project is based on previous work from a university group case study and is
being independently refactored and extended. The current focus is on the MPC
implementation and its integration with a digital twin.

## 1. Clone the repository

Clone the repository including all submodules:

```bash
git clone --recursive git@github.com:charlottemerkl/case-study-extended.git
cd case-study-extended
```

If the repository has already been cloned without `--recursive`, initialize
the submodules manually:

```bash
git submodule update --init --recursive
```

Afterwards, acados should be available at:

```text
external/acados/
```

---

## 2. Install system dependencies

The following instructions assume Ubuntu or WSL with Ubuntu.

Install the tools required to build acados:

```bash
sudo apt update
sudo apt install -y build-essential cmake git
```

## 3. Create the Python virtual environment

From the project root, create a virtual environment using Python 3.12:

```bash
python3.12 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```

Upgrade the Python packaging tools:

```bash
python -m pip install --upgrade pip setuptools wheel
```

---

## 4. Build acados

acados is included as a Git submodule under `external/acados`.

From the project root:

```bash
cd external/acados
git submodule update --init --recursive
```

Configure the build:

```bash
cmake -S . -B build \
    -DACADOS_WITH_QPOASES=ON
```

Build and install acados:

```bash
cmake --build build --target install -j4
```

Return to the project root:

```bash
cd ../..
```

---

## 5. Install Python dependencies

Make sure the virtual environment is active:

```bash
source .venv/bin/activate
```

Install the project dependencies:

```bash
pip install -r requirements.txt
```

Install the acados Python interface from the local acados submodule:

```bash
pip install -e external/acados/interfaces/acados_template
```

---

## 6. Activate the project environment

The project provides an activation script that activates the Python virtual
environment and configures the paths required by acados.

From the project root, run:

```bash
source scripts/activate.sh
```

The script performs the equivalent of:

```bash
source .venv/bin/activate

export ACADOS_SOURCE_DIR="$PROJECT_ROOT/external/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
```

After activation, the terminal should use the Python 3.12 environment located
in `.venv`.

For every new terminal session, it is sufficient to run:

```bash
source scripts/activate.sh
```

When finished working on the project, deactivate the Python environment with:

```bash
deactivate
```

---

## 7. Verify the acados installation

Before running the MPC implementation, the acados installation can be tested
using one of the examples shipped with acados.

For example:

```bash
python external/acados/examples/acados_python/getting_started/minimal_example_ocp.py
```

If the example executes successfully, both the acados C libraries and the
Python interface are configured correctly.