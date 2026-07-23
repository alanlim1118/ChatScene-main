source /home/mifcom2/yungloon/ChatScene-main/chatscene-v3/bin/activate
export CARLA_ROOT=/home/mifcom2/yungloon/CARLA_0.9.15
export MODEL_DEVICE=cuda
export PYTHONPATH="/home/mifcom2/yungloon/ChatScene-main"
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla:${PYTHONPATH}"
# CARLA's compiled libcarla was built against Ubuntu 20.04's libtiff5/libjpeg8-9,
# which Ubuntu 24.04 no longer ships (only libtiff6). Without these, importing
# `carla` fails with a misleading ModuleNotFoundError from Scenic's wrapper.
export LD_LIBRARY_PATH="/home/mifcom2/yungloon/ChatScene-main/chatscene-v3/carla_compat_libs:${LD_LIBRARY_PATH}"
export SDL_VIDEODRIVER=dummy