TEMPLATE = subdirs

SUBDIRS = UAV_Core Voxels

UAV_Core.depends += Voxels
