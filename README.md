# OpenCat-Imitation

Let [OpenCat](https://github.com/PetoiCamp/OpenCat) robot follow your posture.

[![Teach Nybble to learn from human moves](https://i.ytimg.com/vi/4oNJMGLDIT4/maxresdefault.jpg)](https://www.youtube.com/watch?v=4oNJMGLDIT4)


## How to use

Clone repo and update

```bash
git clone https://github.com/TomCC7/OpenCat-Imitation
git submodule update --init --recursive --remote
```

Prepare python environment

+ create virtual environment

```bash
virtualenv -p python3 venv
source ./venv/bin/activate
pip install -r requirements.txt
```

+ follow [this guide](https://github.com/axinc-ai/ailia-models/blob/master/TUTORIAL.md) to install AILIA sdk, which will be used for pose detection.

Run the program, make sure you already connected the robot

```bash
# set python import path
export PYTHONPATH=$PWD:$PWD/serialMaster
python opencat_imitation/imitation.py -v0
```

