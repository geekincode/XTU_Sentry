from setuptools import find_packages, setup

package_name = 'sentry_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm',
    maintainer_email='2949542342@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_pose=sentry_api.nav_to_pose:main',
            'judge=sentry_api.judge:main'
        ],
    },
)
