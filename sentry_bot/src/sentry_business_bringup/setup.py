from setuptools import setup
from glob import glob

setup(
    name="sentry_business_bringup",
    version="0.1.0",
    packages=["sentry_business_bringup"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/sentry_business_bringup"],
        ),
        ("share/sentry_business_bringup", ["package.xml"]),
        ("share/sentry_business_bringup/launch", glob("launch/*.py")),
        ("share/sentry_business_bringup/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
