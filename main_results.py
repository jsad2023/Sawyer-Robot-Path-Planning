"""
Main results
"""
from src.rrt import run
from src.polygon import plot_sphere, plot_cylinder
from src.sawyer import plot_sawyer


if __name__ == "__main__":
    plot_cylinder()
    plot_sphere()
    plot_sawyer()
    run()
