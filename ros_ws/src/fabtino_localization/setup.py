from setuptools import setup
package_name="fabtino_localization"
setup(tests_require=["pytest"],name=package_name,version="1.0.0",packages=[package_name],data_files=[("share/ament_index/resource_index/packages",["resource/fabtino_localization"]),("share/fabtino_localization",["package.xml"])],install_requires=["setuptools"],zip_safe=True,entry_points={"console_scripts":['fabtino-localization = fabtino_localization.localization_node:main']})
