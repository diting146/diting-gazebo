from setuptools import setup

setup(
    name='sentry_runtime',
    version='0.1.0',
    packages=['sentry_runtime'],
    entry_points={
        'console_scripts': [
            'tunnel_region_monitor = sentry_runtime.tunnel_region_monitor:main',
            'referee_state_adapter = sentry_runtime.referee_state_adapter:main',
        ],
    },
)
