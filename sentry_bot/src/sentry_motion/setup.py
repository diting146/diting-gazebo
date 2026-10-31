from setuptools import setup
from glob import glob

setup(
    name="sentry_motion",
    version="0.1.0",
    packages=["sentry_motion"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sentry_motion"]),
        ("share/sentry_motion", ["package.xml"]),
        ("share/sentry_motion/launch", glob("launch/*.py")),
        ("share/sentry_motion/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tunnel_controller = sentry_motion.tunnel_controller:main",
            "nav_velocity_adapter = sentry_motion.nav_velocity_adapter:main",
            "velocity_arbiter = sentry_motion.velocity_arbiter:main",
        ]
    },
)
