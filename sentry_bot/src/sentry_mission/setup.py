from setuptools import setup
from glob import glob

setup(
    name="sentry_mission",
    version="0.1.0",
    packages=["sentry_mission"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sentry_mission"]),
        ("share/sentry_mission", ["package.xml"]),
        ("share/sentry_mission/launch", glob("launch/*.py")),
        ("share/sentry_mission/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["opening_mission = sentry_mission.opening_mission:main"]
    },
)
