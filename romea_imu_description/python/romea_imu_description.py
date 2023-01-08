# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import xacro

from ament_index_python.packages import get_package_share_directory


def urdf(prefix, name, type, model, rate, parent_link, xyz, rpy):

    xacro_file = (
        get_package_share_directory("romea_imu_description")
        + "/urdf/"
        + type
        + "_"
        + model
        + ".xacro.urdf"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "name": name,
            "rate": str(rate),
            "parent_link": parent_link,
            "xyz": " ".join(map(str, xyz)),
            "rpy": " ".join(map(str, rpy)),
            "mesh_visual": str(False),
        },
    )

    return urdf_xml.toprettyxml()
