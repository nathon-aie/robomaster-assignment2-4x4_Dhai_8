"""Project-side compatibility for RoboMaster SDK connection type identity checks.

Some SDK versions compare strings with `is`. YAML strings must be mapped
to the SDK's exact constant objects before initialize(), without modifying the SDK.
"""

import sys


def load_robot_sdk():
    """Load RoboMaster SDK while keeping camera media codec optional.

    The DJI SDK imports camera/media from robomaster.robot even when the
    robot tasks only need sensor and chassis modules.
    Some Linux installs do not ship libmedia_codec, so provide a small no-op
    codec module that lets Robot() construct camera/liveview objects. Camera
    streaming remains unavailable in that environment, but the robot tasks
    do not use it.
    """
    try:
        from robomaster import robot
        return robot
    except ModuleNotFoundError as exc:
        if exc.name != "libmedia_codec":
            raise RuntimeError("RoboMaster SDK is not installed correctly: {}".format(exc))

    import types

    class _NoCameraCodec(object):
        def __init__(self, *args, **kwargs):
            pass

        def decode(self, *args, **kwargs):
            return None

        def stop(self, *args, **kwargs):
            pass

        def display(self, *args, **kwargs):
            pass

    codec = types.ModuleType("libmedia_codec")
    codec.H264Decoder = _NoCameraCodec
    codec.OpusDecoder = _NoCameraCodec
    codec.AudioDecoder = _NoCameraCodec
    sys.modules["libmedia_codec"] = codec

    try:
        from robomaster import robot
        return robot
    except ImportError as exc:
        raise RuntimeError("RoboMaster SDK is not installed correctly: {}".format(exc))


def canonical_connection_type(value):
    from robomaster import conn
    modes = {
        'ap': conn.CONNECTION_WIFI_AP,
        'sta': conn.CONNECTION_WIFI_STA,
        'rndis': conn.CONNECTION_USB_RNDIS,
    }
    if not isinstance(value, str) or value not in modes:
        raise ValueError('Unsupported robot connection type: {!r}'.format(value))
    return modes[value]


def initialize_robot(robot, conn_type):
    canonical = canonical_connection_type(conn_type)
    try:
        initialized = robot.initialize(conn_type=canonical)
    except Exception as exc:
        raise RuntimeError(
            'เชื่อมต่อ RoboMaster ไม่สำเร็จ ({}): {}. '
            'ตรวจเครือข่ายของหุ่นและ robot.conn_type ใน config/settings.yaml'.format(conn_type, exc)
        ) from exc
    if not initialized:
        raise RuntimeError(
            'RoboMaster SDK เชื่อมต่อไม่สำเร็จ ({}); ตรวจเครือข่ายของหุ่น'.format(conn_type))


def chassis_speed_has_no_ack(chassis):
    """This SDK's speed PUSH returns False because no response exists.

    Restrict this exception to the real SDK method and its no-ACK protocol;
    other drivers returning False still indicate failure.
    """
    from robomaster.chassis import Chassis
    from robomaster import protocol
    return (isinstance(chassis, Chassis)
            and getattr(chassis.drive_speed, '__func__', None) is Chassis.drive_speed
            and getattr(chassis.client, "_running", False) is True
            and protocol.ProtoChassisSpeedMode._cmdtype == protocol.DUSS_MB_TYPE_PUSH)


def cancel_chassis_speed_timer(chassis):
    """Drain the SDK auto-stop timer before a position-action-only Gimbal scan.

    drive_speed(timeout=...) leaves a timer even after drive_speed(0,0,0).
    Cancelling its runtime timer requires no SDK source changes.
    """
    from robomaster.chassis import Chassis
    if not isinstance(chassis, Chassis):
        return
    timer = getattr(chassis, '_auto_timer', None)
    if timer is not None:
        timer.cancel()
        if timer.is_alive():
            timer.join(timeout=0.5)
            if timer.is_alive():
                raise RuntimeError('Pending chassis auto-stop timer did not finish')
        chassis._auto_timer = None


def stop_chassis_wheels(chassis):
    """End speed control with the SDK wheel-stop request, which has an ACK."""
    cancel_chassis_speed_timer(chassis)
    if chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0) is not True:
        raise RuntimeError('Chassis wheel stop was not acknowledged')
