from setuptools import setup, find_packages
package_name="fabtino_core"
setup(tests_require=["pytest"],name=package_name,version="1.0.0",packages=find_packages(),data_files=[("share/ament_index/resource_index/packages",["resource/fabtino_core"]),("share/fabtino_core",["package.xml"])],install_requires=["setuptools","numpy","PyYAML"],zip_safe=True)
