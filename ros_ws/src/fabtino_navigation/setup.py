from setuptools import setup
package_name="fabtino_navigation"
setup(tests_require=["pytest"],name=package_name,version="1.0.0",packages=[package_name],data_files=[("share/ament_index/resource_index/packages",["resource/fabtino_navigation"]),("share/fabtino_navigation",["package.xml"])],install_requires=["setuptools"],zip_safe=True,entry_points={"console_scripts":['fabtino-global-planner = fabtino_navigation.global_planner_node:main', 'fabtino-local-planner = fabtino_navigation.local_planner_node:main', 'fabtino-mission-manager = fabtino_navigation.mission_manager_node:main']})
