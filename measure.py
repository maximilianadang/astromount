from astromount import Mount
from astromount_config import PORT, BASELINE, FRAME
from astromount_control import Reference

reference = Reference.from_baseline(BASELINE)
with Mount(PORT) as mount:
    print(mount.identity())
    print(mount.position())
    h, d, status = mount.joint_sample()
    print({'status': status, 'estimated_azel_degrees': FRAME.forward(*reference.offsets(h, d))})
    print(mount.tracking())
