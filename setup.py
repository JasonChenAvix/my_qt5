from setuptools import setup

package_name = 'qt5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'qt5 =qt5.qt5:main',
        'gtk3 = qt5.gtk3:main',
        'test_gtk3 = qt5.gtk3_test:main',
        'v1 = qt5.gtk3_testv1:main',

        ],
    },
)
