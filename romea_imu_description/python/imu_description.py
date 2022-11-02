#!/usr/bin/env python3

import xacro

from ament_index_python.packages import get_package_share_directory

def urdf(prefix,name,type, model, rate, parent_link, xyz,rpy):

    xacro_file =  get_package_share_directory("romea_imu_description")+ "/urdf/"+type+"_"+model+".xacro.urdf"

    print(xyz[0])
    print(rpy[1])
    print(' '.join(map(str, xyz)))
    print("prefix", prefix)
    print("name", name)

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "name": name,
            "rate": str(rate),
            "parent_link": parent_link,
            "xyz": ' '.join(map(str, xyz)),
            "rpy": ' '.join(map(str, rpy)),
        },
    )

    return urdf_xml.toprettyxml()
