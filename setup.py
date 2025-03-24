from setuptools import setup, find_packages
from pybind11.setup_helpers import Pybind11Extension, build_ext
import pybind11
import sys

extra_link_args_graphics = []

# annoying macos stuff
if sys.platform == "darwin":
    extra_link_args_graphics = ["-framework", "GLUT", "-framework", "OpenGL"]

ext_modules = [
    Pybind11Extension(
        "physics",
        ["src/physics.cpp"],
        include_dirs=[pybind11.get_include()],
        language="c++",
    ),
    Pybind11Extension(
        "graphics",
        ["src/graphics.cpp"],
        include_dirs=[pybind11.get_include()],
        language="c++",
        extra_link_args=extra_link_args_graphics,
    ),
]

setup(
    name="adcs_simulator",
    version="1",
    author="illumination",
    author_email="https://github.com/ilumn",
    description="Extensible ADCS simulator, physics and graphics in cpp lib, controller and main loop in python",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    packages=find_packages(),
    install_requires=[
        "pybind11>=2.6.0",
        "numpy",
        "PyOpenGL",
        "PyOpenGL_accelerate",
    ],
    zip_safe=False,
)