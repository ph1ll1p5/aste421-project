from setuptools import setup, find_packages

setup(
    name="aste421-imu-gds-plugin",
    version="0.1.0",
    packages=find_packages(),
    install_requires=["fprime-gds"],
    entry_points={
        "fprime_gds": [
            "imu_receiver = gdsPlugins.gdsReceiver:Gdsreceiver",
        ],
    },
)