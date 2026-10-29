from setuptools import setup

setup(name='robotx_task1', version='0.1.0', packages=['robotx_task1'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robotx_task1']),
                  ('share/robotx_task1', ['package.xml'])],
      entry_points={'console_scripts': [
          'known_observer = robotx_task1.observer:main',
          'mission = robotx_task1.mission:main',
          'judge = robotx_task1.judge:main']})
