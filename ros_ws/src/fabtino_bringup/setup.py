from setuptools import setup
from glob import glob
from pathlib import Path
package_name = 'fabtino_bringup'
setup(tests_require=["pytest"],name=package_name, version='1.0.0', packages=['fabtino_bringup'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/fabtino_bringup']),
                  ('share/fabtino_bringup', ['package.xml']),
      ('share/fabtino_bringup/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': {}})
