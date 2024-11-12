"""**plots.py**

======  ============================================================================================
file    sturdr/utils/plots.py 
brief   Common plotting functions.
date    November 2024
======  ============================================================================================
"""
import numpy as np
import matplotlib.pyplot as plt
from collections.abc import Iterable
import matplotlib.patheffects as pe

def skyplot(
    az: np.ndarray,
    el: np.ndarray,
    name: str | list = None,
    deg: bool = True,
    fig: plt.Figure = None,
    ax: plt.Axes = None,
    **kwargs
):
    """Creates a polar plot given azimuth and elevation

    Parameters
    ----------
    az : np.ndarray
        size NxM array of azimuth angles
    el : np.ndarray
        size NxM array of elevation angles
    name : str | list, optional
        size N string list of names, by default None
    deg : bool, optional
        Input in degrees?, by default True
    fig : plt.Figure, optional
        Matplotlib figure to plot onto, by default None
    ax : plt.Axes, optional
        Matplotlib axes to plot onto, by default None

    Returns
    -------
    fig : plt.Figure
        Matplotlib figure object
    ax : plt.Axes
        Matplotlib polar axes object
    """
    # if isinstance(plt.gca(), plt.PolarAxes):
    #     ax = plt.gca()
    # else:
    #     plt.close()
    #     fig = plt.gcf()
    #     ax = fig.add_subplot(projection="polar")
    if fig == None and ax == None:
        fig = plt.figure()
        ax = fig.add_subplot(projection="polar")

    if deg:
        az = np.deg2rad(az)
    else:
        el = np.rad2deg(el)

    # format polar axes
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_rlim(91, 1)

    degree_sign = "\N{DEGREE SIGN}"
    r_labels = [
        # "0" + degree_sign,
        "",
        "",
        "30" + degree_sign,
        "",
        "60" + degree_sign,
        "",
        "90" + degree_sign,
    ]
    ax.set_rgrids(range(1, 106, 15), r_labels, angle=22.5)

    ax.set_axisbelow(True)

    # plot
    ax.scatter(az, el, **kwargs)

    # annotate object names
    if name is not None:
        if not isinstance(name, Iterable):
            name = (name,)

        for obj, n in enumerate(name):
            ax.annotate(
                n,
                (az[obj, 0], el[obj, 0]),
                fontsize="x-small",
                path_effects=[pe.withStroke(linewidth=3, foreground="w")],
            )

    ax.figure.canvas.draw()

    return fig, ax