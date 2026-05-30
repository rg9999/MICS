from setuptools import find_packages, setup

package_name = "mics_nodes"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mics.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MICS",
    maintainer_email="dev@example.com",
    description="MICS ROS2 nodes wrapping the mics core.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "attacker_sim = mics_nodes.attacker_sim_node:main",
            "target_ingest = mics_nodes.target_ingest_node:main",
            "track_manager = mics_nodes.track_manager_node:main",
            "allocator = mics_nodes.allocator_node:main",
            "aircraft = mics_nodes.aircraft_node:main",
            "monitor = mics_nodes.monitor_node:main",
        ],
    },
)
