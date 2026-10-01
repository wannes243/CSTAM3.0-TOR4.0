from setuptools import setup
package_name="fabtino_viewer"
setup(tests_require=["pytest"],name=package_name,version="1.0.0",packages=[package_name],data_files=[("share/ament_index/resource_index/packages",["resource/fabtino_viewer"]),("share/fabtino_viewer",["package.xml"])],install_requires=["setuptools"],zip_safe=True,entry_points={"console_scripts":['fabtino-viewer-gateway = fabtino_viewer.viewer_gateway:main']})
