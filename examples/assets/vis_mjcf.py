from typing import Optional
import typer
import mujoco
from mujoco import viewer as mj_viewer
import numpy as np


def main(
    model_path: str,
    ipdb: bool = False,
):
    """Visualize mjcf.
    
    >>> python -m mujoco.viewer --mjcf mjcf/X7S.xml
    """
    assert model_path.endswith(".xml"), "input model_path should endswith .xml"

    typer.echo(typer.style(f"load {model_path}", fg=typer.colors.GREEN))
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    viewer = mj_viewer.launch_passive(model, data)

    if ipdb:
        __import__('ipdb').set_trace()

    while True:
        mujoco.mj_step(model, data)
        viewer.sync()
    viewer.close()


if __name__ == "__main__":
    typer.run(main)
