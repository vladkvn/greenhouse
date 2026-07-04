from setuptools import find_packages, setup

package_name = "rover_drivers"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Uladzislau Kudzelka",
    maintainer_email="vlad.kvn@gmail.com",
    description="Узлы-мосты ровера к железу: cmd_vel→ESP32 и BNO085→/imu/data.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "esp32_cmd_vel_bridge = rover_drivers.esp32_cmd_vel_bridge:main",
            "bno085_imu_node = rover_drivers.bno085_imu_node:main",
            "rplidar_scan_node = rover_drivers.rplidar_scan_node:main",
            "rover_webui = rover_drivers.rover_webui:main",
        ],
    },
)
