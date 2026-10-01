from setuptools import setup
package_name="fabtino_webots_bridge"
setup(tests_require=["pytest"],name=package_name,version="1.0.0",packages=[package_name],data_files=[("share/ament_index/resource_index/packages",["resource/fabtino_webots_bridge"]),("share/fabtino_webots_bridge",["package.xml"])],install_requires=["setuptools"],zip_safe=True,entry_points={"console_scripts":['fabtino-webots-bridge = fabtino_webots_bridge.webots_bridge_node:main']})
