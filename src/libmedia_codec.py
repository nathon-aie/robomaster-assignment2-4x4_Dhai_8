"""RoboMaster media codec backed by DJI's prebuilt Python 3.8 decoder wheels.

The SDK expects the combined ``libmedia_codec`` extension. DJI also ships
separate H.264 and Opus decoder wheels with the same decoding operations. This
adapter converts their RGB video frames to the tightly packed BGR frames that
``robomaster.media.LiveView`` expects.
"""

import libh264decoder
import numpy as np
import opus_decoder


class H264Decoder:
    def __init__(self, output_format="BGR", verbose=True):
        if output_format not in ("BGR", "RGB"):
            raise ValueError("Unsupported video output format: {!r}".format(output_format))
        self.output_format = output_format
        self._decoder = libh264decoder.H264Decoder()
        if not verbose:
            libh264decoder.disable_logging()

    def decode(self, data):
        frames = []
        for rgb_bytes, width, height, line_stride in self._decoder.decode(data):
            if rgb_bytes is None:
                continue
            # The separate DJI decoder can pad each row. LiveView reshapes
            # directly to (height, width, 3), so remove padding here.
            rgb = np.ndarray(
                (height, width, 3), dtype=np.uint8, buffer=rgb_bytes,
                strides=(line_stride, 3, 1))
            if self.output_format == "BGR":
                pixels = rgb[:, :, ::-1].copy()
            else:
                pixels = rgb.copy()
            frames.append((pixels.tobytes(), width, height, width * 3))
        return frames


OpusDecoder = opus_decoder.opus_decoder
AudioDecoder = OpusDecoder
