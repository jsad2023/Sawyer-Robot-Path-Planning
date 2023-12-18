"""
Main results
"""
from rrt import run
from polygon import plot_sphere, plot_cylinder
from sawyer import plot_sawyer


if __name__ == "__main__":
    plot_cylinder()
    plot_sphere()
    plot_sawyer()
    run()
