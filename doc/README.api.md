C interface for embedding TMC13
===============================

`tmc3/include/tmc3_api.h` declares a C interface to the encoder and
decoder, so that the codec can run inside another application or language
(eg, Python through `ctypes`, `cffi`, pybind11 or nanobind) instead of as
the `tmc3` program reading and writing ply files.

Design goals:

- **No copies at the boundary.**  Point data is described by strided
  arrays that point at the caller's memory, so arrays from other
  environments (numpy arrays, including structured arrays, column-major
  arrays, DLPack tensors) are passed as they are.  Decoded points are
  returned as arrays that point at the codec's own storage.
- **Same output as `tmc3`.**  For the same points and options, the library
  produces the same bitstream, reconstruction and decoded points as the
  `tmc3` program, byte for byte.
- **Plain C ABI.**  No C++ types, exceptions or callbacks cross the
  interface; every call returns a status code.

The interface is a library only: no language binding is part of this
project.


Building
--------

CMake builds three targets from `tmc3/CMakeLists.txt`:

| Target | Type | Contents |
|---|---|---|
| `tmc3core` | static library | the codec |
| `tmc3` | program | the command line application |
| `tmc3codec` | shared library | the C interface; exports only the `tmc3_*` functions |

`make install` installs `libtmc3codec` and `tmc3_api.h`.  The test driver
`tools/tmc3_api_test.c` is built with `make tmc3_api_test`.

The application and the library share the option parsing and the input and
output conversions (`tmc3/app_params.cpp`), so that both behave the same.

CMake 3.10 or later is required.  The targets have been built and tested
with CMake 4.4 on macOS; Linux and Windows (MSVC) builds of the library
have not been tested yet.


Point data
----------

### Strided arrays

```c
typedef struct {
  void* data;           /* NULL: absent */
  tmc3_dtype dtype;     /* UINT8, UINT16, INT32, FLOAT32, FLOAT64 */
  int64_t stride;       /* bytes between consecutive points */
  int64_t offset[3];    /* byte offset of each component */
} tmc3_array;
```

Component `c` of point `i` is at `(char*)data + i * stride + offset[c]`.
A zero stride means packed points, and all-zero offsets mean packed
components, so a plain `(N, 3)` array only needs `data` and `dtype`.

| Memory layout | `stride` | `offset` |
|---|---|---|
| `float32` array of shape `(N, 3)` | 0 | 0, 0, 0 |
| structured array with fields `x, y, z` (eg. a ply vertex record, a lidar driver packet) | record size | field offsets |
| column-major `(N, 3)`, or separate `x[N], y[N], z[N]` in one allocation | element size | 0, N·size, 2·N·size |

A numpy array maps directly: `data` is `a.ctypes.data` (or the buffer
protocol pointer), `stride` is `a.strides[0]`, and the offsets are the field
offsets of a structured dtype or `k * a.strides[1]`.

### Input clouds

```c
typedef struct {
  int64_t num_points;
  tmc3_array position;     /* x, y, z: any dtype */
  tmc3_array colour;       /* r, g, b: UINT8 or UINT16 */
  tmc3_array reflectance;  /* UINT8 or UINT16 */
} tmc3_cloud;
```

The arrays are only read during `tmc3_encoder_encode`.  The encoder reorders
and quantises points in its own working copy, so the input is read once,
directly into that copy, as `tmc3` reads a ply file:

- positions are multiplied by `inputScale` in double precision and
  truncated to integers;
- the input is sorted by azimuth if `sortInputByAzimuth` is set;
- colours are converted according to `colourMatrix` (with
  `convertPlyColourspace`) and reflectance is scaled with
  `attrOffset`/`attrScale`.

Attributes that are not coded (no `attribute=color` or
`attribute=reflectance` option) are ignored; a coded attribute must be
given.

### Decoded frames

A decoded frame (`tmc3_frame`) owns the decoded points.  Its arrays point
at that storage and remain valid until `tmc3_frame_release`:

```c
typedef struct {
  int64_t num_points;
  tmc3_array position;     /* x, y, z: INT32 */
  tmc3_array colour;       /* r, g, b: UINT16, or absent */
  tmc3_array reflectance;  /* UINT16, or absent */
  int32_t frame_num;
  double scale;            /* output = position * scale + origin */
  double origin[3];
} tmc3_frame_info;
```

- Positions are the integer values decoded by the codec, in the output
  coordinate system of `tmc3` (`outputScaling`, `outputUnitLength`,
  `outputPrecisionBits`): `value * scale + origin` is what `tmc3` writes to
  its output ply.  `tmc3_frame_get_positions` writes these values as
  float32 or float64 into a caller array in one pass.
- The offsets describe the codec's storage: the components are given in
  x, y, z (and r, g, b) order whatever the storage order is (eg, the
  `geometry_axis_order` option, or colours stored as g, b, r).
- Colour conversion and reflectance scaling for output have already been
  applied, as for the `tmc3` output.

The frame is not modified after it is returned, so its arrays can be read
from any thread, and a binding can expose them as read-only views whose
lifetime holds a reference to the frame.


Encoding
--------

```c
const char* options[] = {"config=lidar.cfg", "numThreads=4"};
tmc3_encoder* enc;
tmc3_encoder_create(options, 2, &enc);

tmc3_cloud cloud = {num_points, {xyz, TMC3_FLOAT32}, {0}, {refl, TMC3_UINT16}};
tmc3_packet* packet;
tmc3_encoder_encode(enc, &cloud, &packet, NULL);
send(tmc3_packet_data(packet), tmc3_packet_size(packet));
tmc3_packet_release(packet);
...
tmc3_encoder_destroy(enc);
```

- Options are the `tmc3` command line options as `name=value` strings
  (a leading `--` is optional), including `config=<file>`.  Options are
  applied in order, as on the command line.  The mode and file paths
  cannot be set.  Warnings from option parsing are available with
  `tmc3_last_error` after a successful call.
- A packet holds the data units of one frame in the format of `tmc3`
  bitstream files: packets concatenated in order form a bitstream that
  `tmc3` decodes.
- If `reconstruction` is not NULL, it receives the encoder's
  reconstruction of the frame, as a frame object.
- Frames are coded in the order they are passed.  Inter prediction and
  `randomAccessPeriod` work as in `tmc3`.  Bi-prediction, which codes frames
  out of order, is not supported (`TMC3_ERR_UNSUPPORTED`).


Decoding
--------

```c
tmc3_decoder* dec;
tmc3_decoder_create(NULL, 0, &dec);

tmc3_decoder_push(dec, data, size);   /* any amount of data */
tmc3_decoder_end_frame(dec);          /* optional: data completes a frame */

tmc3_frame* frame;
while (tmc3_decoder_next_frame(dec, &frame) == TMC3_OK) {
  tmc3_frame_info info;
  tmc3_frame_get_info(frame, &info);
  ...
  tmc3_frame_release(frame);
}

tmc3_decoder_flush(dec);              /* end of stream */
```

- `tmc3_decoder_push` accepts data in pieces of any size: a whole file, one
  packet, or arbitrary chunks.  An incomplete data unit at the end is kept
  until the next push.  Each complete data unit is copied once into the
  codec's payload buffer (as when `tmc3` reads a file).
- Decoded frames are queued; `tmc3_decoder_next_frame` returns them in
  order, or `TMC3_NO_FRAME`.
- **Frame latency.**  The bitstream marks the start of a frame, not its
  end, so the decoder otherwise outputs a frame only when data of the next
  frame (or the end of the stream) arrives.  When the application knows
  that it has pushed a complete frame, eg. a packet from
  `tmc3_encoder_encode`, `tmc3_decoder_end_frame` outputs the frame
  immediately.  The decoded frames are identical either way.  With
  bi-prediction it has no effect, since frames are output in display order.
- **After `tmc3_decoder_end_frame`, the next data starts a new frame,**
  whatever its frame counter.  The decoder otherwise recognises a new frame
  only by a change of the counter, so frames of several streams pushed in
  turn, each counting from zero, were taken as one frame.  With frames coded
  on their own (no inter prediction, `randomAccessPeriod=1`), one decoder
  then takes the frames of several encoders interleaved, and any frame of a
  stream as its first.
- `tmc3_decoder_flush` ends the stream; data pushed afterwards is rejected.
- The decoder is the reference decoder: it is not hardened against
  malformed streams.


Ownership and lifetime
----------------------

| Object | Created by | Released by | Notes |
|---|---|---|---|
| encoder, decoder | `*_create` | `*_destroy` | one call at a time per object |
| packet | `tmc3_encoder_encode` | `tmc3_packet_release` | data valid until release |
| frame | `tmc3_decoder_next_frame`, `tmc3_encoder_encode` | `tmc3_frame_release` | arrays valid until release; independent of the codec object |
| input arrays | caller | caller | only read during the call |


Threads
-------

- Codec objects are independent: different objects can be used from
  different threads at the same time.  An object must not be used by two
  threads at once.
- No call takes a callback into the application, and no call needs a lock
  held by the application: a Python binding can release the GIL for every
  call (`ctypes` does so).
- The number of threads used inside the codec is process-wide:
  `tmc3_set_num_threads`, or the `numThreads` option of a codec, which
  raises it when the codec is created.  The output does not depend on the
  number of threads.  The pool can be resized while codecs run on other
  threads.
- The codec needs a large stack (its functions use large stack frames).
  Threads created by applications get a small stack on some systems
  (512 kB on macOS); when the calling thread's stack is smaller than that
  of the main thread, the call runs on a temporary thread with a large
  stack, and returns as usual.


Errors and messages
-------------------

- Every function returns a `tmc3_status`: `TMC3_OK`, `TMC3_NO_FRAME`, or a
  negative error code.  `tmc3_last_error()` returns the message of the last
  failed call on the calling thread.
- Option `log=quiet` (default), `log=stdout` or `log=stderr` selects where
  the codec's progress messages go.  A few rare warnings from the codec are
  written to `stderr` whatever this option.


Example: numpy
--------------

A sketch of zero-copy use from Python with `ctypes` and numpy (the
structures are those of `tmc3_api.h`):

```python
import ctypes as C
import numpy as np

lib = C.CDLL("libtmc3codec.so")

class Array(C.Structure):
    _fields_ = [("data", C.c_void_p), ("dtype", C.c_int),
                ("stride", C.c_int64), ("offset", C.c_int64 * 3)]

class Cloud(C.Structure):
    _fields_ = [("num_points", C.c_int64), ("position", Array),
                ("colour", Array), ("reflectance", Array)]

class FrameInfo(C.Structure):
    _fields_ = [("num_points", C.c_int64), ("position", Array),
                ("colour", Array), ("reflectance", Array),
                ("frame_num", C.c_int32), ("scale", C.c_double),
                ("origin", C.c_double * 3)]

UINT16, INT32, FLOAT32 = 2, 3, 4

# Input: a structured array (eg, from a lidar driver), passed in place
pts = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                         ("intensity", "<u2")])
f = pts.dtype.fields
base = pts.ctypes.data
cloud = Cloud(len(pts),
              Array(base, FLOAT32, pts.itemsize,
                    (f["x"][1], f["y"][1], f["z"][1])),
              Array(),
              Array(base + f["intensity"][1], UINT16, pts.itemsize))

# Output: a view over the frame's storage (keep the frame until released)
info = FrameInfo()
lib.tmc3_frame_get_info(frame, C.byref(info))
storage = (C.c_char * (info.num_points * info.position.stride)).from_address(
    info.position.data)
xyz = np.ndarray(info.num_points, buffer=storage, dtype=np.dtype({
    "names": ["x", "y", "z"], "formats": ["<i4"] * 3,
    "offsets": list(info.position.offset),
    "itemsize": info.position.stride}))
```

A binding written with pybind11 or nanobind can tie the lifetime of such
views to the frame (a capsule as the array's base object) and release the
frame when the last view is gone.


Verification
------------

The interface was checked against the `tmc3` program (Apple M5 Pro), with
the test driver `tools/tmc3_api_test.c`:

- Encoding 10 lidar frames with the four lidar configurations, four sector
  slices with 4 threads, inter prediction, attribute scaling and
  `geometry_axis_order=0`, and a dense colour cloud with RAHT, lifting and
  trisoup, gives bitstreams and reconstructions identical to `tmc3`, with
  the input passed as the ply records in place, as packed float64 arrays,
  and as column-major float32 arrays.
- Decoding these bitstreams pushed whole, in random pieces and one data
  unit at a time, and frame by frame with `tmc3_decoder_end_frame` (each
  frame returned at once), gives output identical to `tmc3`.
- Four threads, each with its own encoder and decoder, with a pool of four
  threads, give the same results as sequential coding; ThreadSanitizer
  reports no races and AddressSanitizer no errors.
- From Python with `ctypes` (input passed from a `bytearray` in place,
  output read through a `memoryview` of the frame storage), the bitstream
  is identical to `tmc3`, also with three Python threads at once.
- Option, argument and state errors return the documented codes; a
  truncated stream is reported at `tmc3_decoder_flush`.
- The library adds no measurable time: decoding 30 frames with 4 sector
  slices on 4 threads takes 36.7 ms per frame through the library and
  37.3 ms with `tmc3`.
