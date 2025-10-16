from setuptools import setup

package_name = 'example_sweeping_generator'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tomas Baca',
    maintainer_email='tomas.baca@fel.cvut.cz',
    description='Example Sweeping Generator',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sweeping_generator = example_sweeping_generator.sweeping_generator:main',
        ],
    },
)
