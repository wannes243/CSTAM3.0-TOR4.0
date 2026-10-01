from setuptools import setup
package_name="fabtino_safety"
setup(tests_require=["pytest"],name=package_name,version="1.0.0",packages=[package_name],data_files=[("share/ament_index/resource_index/packages",["resource/fabtino_safety"]),("share/fabtino_safety",["package.xml"])],install_requires=["setuptools"],zip_safe=True,entry_points={"console_scripts":['fabtino-safety = fabtino_safety.safety_supervisor_node:main']})
