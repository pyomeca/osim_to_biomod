from setuptools import setup

setup(
    name="osim_to_biomod",
    description="Convert opensim model file (.osim) to biorbd model file (.bioMod)",
    version="0.2",
    author="Aceglia",
    author_email="amedeo.ceglia@umontreal.ca",
    url="https://github.com/pyomeca/osim_to_biomod",
    license="Apache 2.0",
    packages=["osim_to_biomod"],
    keywords="osim_to_biomod",
    classifiers=[
        "Programming Language :: Python :: 3.10",
    ],
)
