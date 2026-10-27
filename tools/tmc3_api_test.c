/* Test driver for the tmc3 C interface (tmc3/include/tmc3_api.h).
 *
 * Encodes and decodes through the library, writing the same files as the
 * tmc3 application, so that the results can be compared byte for byte:
 *
 *   tmc3_api_test encode <layout> <stream.bin> <recon-%04d.ply|-> \
 *     <input-%04d.ply> <first> <count> [option ...]
 *       layout: record    the ply vertex records, passed in place
 *               packed    positions as float64 (N, 3), separate attributes
 *               columns   positions as column-major float32 (3, N)
 *
 *   tmc3_api_test decode <whole|random|unit> <stream.bin> <out-%04d.ply|-> \
 *     [option ...]
 *       whole:  push the stream at once
 *       random: push pieces of random size
 *       unit:   push one data unit at a time
 *
 *   tmc3_api_test live <stream.bin> <out-%04d.ply> <input-%04d.ply> \
 *     <first> <count> [option ...] -- [decoder option ...]
 *       encode each frame, then decode its packet, ending the frame
 *       explicitly; each decoded frame must be returned at once.
 *
 * Only binary little-endian ply files with float/double positions and
 * uchar/ushort attributes are supported.
 */

#include "tmc3_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*--------------------------------------------------------------------------*/

static void
die(const char* what)
{
  fprintf(stderr, "error: %s: %s\n", what, tmc3_last_error());
  exit(1);
}

static void
check(tmc3_status status, const char* what)
{
  if (status != TMC3_OK)
    die(what);
}

/*--------------------------------------------------------------------------*/
/* Minimal binary ply reader */

typedef struct {
  int64_t count;
  int record;  /* bytes per vertex */
  char* data;  /* count * record bytes */
  int pos;     /* offset of x (y, z follow by the type size) or -1 */
  int posSize; /* 4 or 8 */
  int off[3];  /* offsets of x, y, z */
  int rgb[3];  /* offsets of red, green, blue, or -1 */
  int refl;    /* offset of reflectance, or -1 */
  int reflSize;
} Ply;

static int
typeSize(const char* t)
{
  if (!strcmp(t, "float") || !strcmp(t, "float32") || !strcmp(t, "int")
      || !strcmp(t, "int32") || !strcmp(t, "uint") || !strcmp(t, "uint32"))
    return 4;
  if (!strcmp(t, "double") || !strcmp(t, "float64"))
    return 8;
  if (!strcmp(t, "uchar") || !strcmp(t, "uint8") || !strcmp(t, "char")
      || !strcmp(t, "int8"))
    return 1;
  if (!strcmp(t, "ushort") || !strcmp(t, "uint16") || !strcmp(t, "short")
      || !strcmp(t, "int16"))
    return 2;
  return 0;
}

static int
readPly(const char* name, Ply* ply)
{
  FILE* f = fopen(name, "rb");
  char line[256], type[64], prop[64];
  int inVertex = 0;
  if (!f)
    return 0;

  memset(ply, 0, sizeof(*ply));
  ply->off[0] = ply->off[1] = ply->off[2] = -1;
  ply->rgb[0] = ply->rgb[1] = ply->rgb[2] = -1;
  ply->refl = -1;

  while (fgets(line, sizeof(line), f)) {
    long long n;
    if (!strncmp(line, "end_header", 10))
      break;
    if (sscanf(line, "element vertex %lld", &n) == 1) {
      ply->count = n;
      inVertex = 1;
    } else if (!strncmp(line, "element", 7)) {
      inVertex = 0;
    } else if (
      inVertex && sscanf(line, "property %63s %63s", type, prop) == 2) {
      int size = typeSize(type);
      if (!strcmp(prop, "x") || !strcmp(prop, "y") || !strcmp(prop, "z")) {
        ply->off[prop[0] - 'x'] = ply->record;
        ply->posSize = size;
      } else if (!strcmp(prop, "red"))
        ply->rgb[0] = ply->record;
      else if (!strcmp(prop, "green"))
        ply->rgb[1] = ply->record;
      else if (!strcmp(prop, "blue"))
        ply->rgb[2] = ply->record;
      else if (!strcmp(prop, "reflectance") || !strcmp(prop, "refc")) {
        ply->refl = ply->record;
        ply->reflSize = size;
      }
      ply->record += size;
    }
  }

  ply->data = malloc(ply->count * ply->record + 1);
  if (fread(ply->data, ply->record, ply->count, f) != (size_t)ply->count) {
    fclose(f);
    return 0;
  }
  fclose(f);
  return 1;
}

/*--------------------------------------------------------------------------*/
/* Ply output in the format written by tmc3 */

static void
writePly(const char* pattern, const tmc3_frame* frame)
{
  tmc3_frame_info info;
  char name[1024];
  FILE* f;
  double* pos;
  tmc3_array dst;
  int64_t i;

  check(tmc3_frame_get_info(frame, &info), "frame info");
  snprintf(name, sizeof(name), pattern, info.frame_num);
  f = fopen(name, "wb");
  if (!f)
    die("can't open output");

  fprintf(f, "ply\nformat binary_little_endian 1.0\nelement vertex %lld\n",
          (long long)info.num_points);
  fprintf(f, "property float64 x\nproperty float64 y\nproperty float64 z\n");
  if (info.colour.data)
    fprintf(
      f, "property uchar green\nproperty uchar blue\nproperty uchar red\n");
  if (info.reflectance.data)
    fprintf(f, "property uint16 refc\n");
  fprintf(f, "element face 0\nproperty list uint8 int32 vertex_index\n");
  fprintf(f, "end_header\n");

  /* positions converted in one call into a packed float64 array */
  pos = malloc(sizeof(double) * 3 * (info.num_points + 1));
  memset(&dst, 0, sizeof(dst));
  dst.data = pos;
  dst.dtype = TMC3_FLOAT64;
  check(tmc3_frame_get_positions(frame, &dst), "positions");

  /* attributes read in place */
  for (i = 0; i < info.num_points; i++) {
    fwrite(pos + 3 * i, sizeof(double), 3, f);
    if (info.colour.data) {
      const char* p = (const char*)info.colour.data + i * info.colour.stride;
      uint16_t c[3];
      uint8_t gbr[3];
      int k;
      for (k = 0; k < 3; k++)
        memcpy(&c[k], p + info.colour.offset[k], 2);
      gbr[0] = (uint8_t)c[1];
      gbr[1] = (uint8_t)c[2];
      gbr[2] = (uint8_t)c[0];
      fwrite(gbr, 1, 3, f);
    }
    if (info.reflectance.data)
      fwrite(
        (const char*)info.reflectance.data + i * info.reflectance.stride, 2, 1,
        f);
  }
  free(pos);
  fclose(f);
}

static void
drainFrames(tmc3_decoder* dec, const char* pattern, int* numFrames)
{
  tmc3_frame* frame;
  tmc3_status status;
  while ((status = tmc3_decoder_next_frame(dec, &frame)) == TMC3_OK) {
    if (strcmp(pattern, "-"))
      writePly(pattern, frame);
    tmc3_frame_release(frame);
    ++*numFrames;
  }
  if (status != TMC3_NO_FRAME)
    die("next frame");
}

/*--------------------------------------------------------------------------*/
/* Input clouds in several layouts */

typedef struct {
  tmc3_cloud cloud;
  void* buffers[3];
} Input;

static void
makeInput(const Ply* ply, const char* layout, Input* in)
{
  tmc3_cloud* c = &in->cloud;
  int64_t n = ply->count, i;
  int k;

  memset(in, 0, sizeof(*in));
  c->num_points = n;

  if (!strcmp(layout, "record")) {
    /* the file's records, in place: a structured array */
    c->position.data = ply->data;
    c->position.dtype = ply->posSize == 4 ? TMC3_FLOAT32 : TMC3_FLOAT64;
    c->position.stride = ply->record;
    for (k = 0; k < 3; k++)
      c->position.offset[k] = ply->off[k];
    if (ply->rgb[0] >= 0) {
      c->colour.data = ply->data;
      c->colour.dtype = TMC3_UINT8;
      c->colour.stride = ply->record;
      for (k = 0; k < 3; k++)
        c->colour.offset[k] = ply->rgb[k];
    }
    if (ply->refl >= 0) {
      c->reflectance.data = ply->data + ply->refl;
      c->reflectance.dtype = ply->reflSize == 1 ? TMC3_UINT8 : TMC3_UINT16;
      c->reflectance.stride = ply->record;
    }
    return;
  }

  if (!strcmp(layout, "packed")) {
    /* float64 (N, 3) positions: packed (stride and offsets 0) */
    double* pos = malloc(sizeof(double) * 3 * n);
    for (i = 0; i < n; i++)
      for (k = 0; k < 3; k++) {
        const char* p = ply->data + i * ply->record + ply->off[k];
        if (ply->posSize == 4) {
          float v;
          memcpy(&v, p, 4);
          pos[3 * i + k] = v;
        } else
          memcpy(&pos[3 * i + k], p, 8);
      }
    c->position.data = in->buffers[0] = pos;
    c->position.dtype = TMC3_FLOAT64;
  } else if (!strcmp(layout, "columns") && ply->posSize == 4) {
    /* float32 column-major: x[N], y[N], z[N] */
    float* pos = malloc(sizeof(float) * 3 * n);
    for (i = 0; i < n; i++)
      for (k = 0; k < 3; k++)
        memcpy(&pos[k * n + i], ply->data + i * ply->record + ply->off[k], 4);
    c->position.data = in->buffers[0] = pos;
    c->position.dtype = TMC3_FLOAT32;
    c->position.stride = sizeof(float);
    for (k = 0; k < 3; k++)
      c->position.offset[k] = k * n * (int64_t)sizeof(float);
  } else {
    fprintf(stderr, "unsupported layout %s\n", layout);
    exit(1);
  }

  if (ply->rgb[0] >= 0) {
    uint16_t* rgb = malloc(sizeof(uint16_t) * 3 * n);
    for (i = 0; i < n; i++)
      for (k = 0; k < 3; k++)
        rgb[3 * i + k] = (uint8_t)ply->data[i * ply->record + ply->rgb[k]];
    c->colour.data = in->buffers[1] = rgb;
    c->colour.dtype = TMC3_UINT16;
  }
  if (ply->refl >= 0) {
    uint16_t* refl = malloc(sizeof(uint16_t) * n);
    for (i = 0; i < n; i++) {
      const char* p = ply->data + i * ply->record + ply->refl;
      if (ply->reflSize == 1)
        refl[i] = (uint8_t)*p;
      else
        memcpy(&refl[i], p, 2);
    }
    c->reflectance.data = in->buffers[2] = refl;
    c->reflectance.dtype = TMC3_UINT16;
  }
}

static void
freeInput(Input* in)
{
  int k;
  for (k = 0; k < 3; k++)
    free(in->buffers[k]);
}

/*--------------------------------------------------------------------------*/

static int
encode(int argc, char** argv)
{
  const char* layout = argv[2];
  FILE* out = fopen(argv[3], "wb");
  const char* recon = strcmp(argv[4], "-") ? argv[4] : NULL;
  int first = atoi(argv[6]), count = atoi(argv[7]), i;
  tmc3_encoder* enc;

  if (!out)
    die("can't open stream");
  check(
    tmc3_encoder_create((const char* const*)argv + 8, argc - 8, &enc),
    "encoder options");

  for (i = first; i < first + count; i++) {
    char name[1024];
    Ply ply;
    Input in;
    tmc3_packet* packet;
    tmc3_frame* frame = NULL;

    snprintf(name, sizeof(name), argv[5], i);
    if (!readPly(name, &ply))
      die("can't read input");
    makeInput(&ply, layout, &in);
    check(
      tmc3_encoder_encode(enc, &in.cloud, &packet, recon ? &frame : NULL),
      "encode");
    fwrite(tmc3_packet_data(packet), 1, tmc3_packet_size(packet), out);
    tmc3_packet_release(packet);
    if (frame) {
      writePly(recon, frame);
      tmc3_frame_release(frame);
    }
    freeInput(&in);
    free(ply.data);
  }

  tmc3_encoder_destroy(enc);
  fclose(out);
  return 0;
}

/*--------------------------------------------------------------------------*/

static int
decode(int argc, char** argv)
{
  const char* mode = argv[2];
  const char* pattern = argv[4];
  FILE* f = fopen(argv[3], "rb");
  char* data;
  long size, pos = 0;
  int numFrames = 0;
  tmc3_decoder* dec;

  if (!f)
    die("can't open stream");
  fseek(f, 0, SEEK_END);
  size = ftell(f);
  fseek(f, 0, SEEK_SET);
  data = malloc(size + 1);
  if (fread(data, 1, size, f) != (size_t)size)
    die("can't read stream");
  fclose(f);

  check(
    tmc3_decoder_create((const char* const*)argv + 5, argc - 5, &dec),
    "decoder options");

  srand(1);
  while (pos < size) {
    long len = size - pos;
    if (!strcmp(mode, "random"))
      len = 1 + rand() % 20000;
    else if (!strcmp(mode, "unit"))
      len = 5
        + (((long)(uint8_t)data[pos + 1] << 24)
           | ((long)(uint8_t)data[pos + 2] << 16)
           | ((long)(uint8_t)data[pos + 3] << 8) | (uint8_t)data[pos + 4]);
    if (len > size - pos)
      len = size - pos;
    check(tmc3_decoder_push(dec, data + pos, len), "push");
    pos += len;
    drainFrames(dec, pattern, &numFrames);
  }
  check(tmc3_decoder_flush(dec), "flush");
  drainFrames(dec, pattern, &numFrames);

  tmc3_decoder_destroy(dec);
  free(data);
  printf("%d frames\n", numFrames);
  return 0;
}

/*--------------------------------------------------------------------------*/

static int
live(int argc, char** argv)
{
  FILE* out = fopen(argv[2], "wb");
  const char* pattern = argv[3];
  int first = atoi(argv[5]), count = atoi(argv[6]), i, sep, numFrames = 0;
  tmc3_encoder* enc;
  tmc3_decoder* dec;

  for (sep = 7; sep < argc && strcmp(argv[sep], "--"); sep++)
    ;
  if (!out)
    die("can't open stream");
  check(
    tmc3_encoder_create((const char* const*)argv + 7, sep - 7, &enc),
    "encoder options");
  check(
    tmc3_decoder_create(
      (const char* const*)argv + sep + 1, sep < argc ? argc - sep - 1 : 0,
      &dec),
    "decoder options");

  for (i = first; i < first + count; i++) {
    char name[1024];
    Ply ply;
    Input in;
    tmc3_packet* packet;
    tmc3_frame* frame;

    snprintf(name, sizeof(name), argv[4], i);
    if (!readPly(name, &ply))
      die("can't read input");
    makeInput(&ply, "record", &in);
    check(tmc3_encoder_encode(enc, &in.cloud, &packet, NULL), "encode");
    fwrite(tmc3_packet_data(packet), 1, tmc3_packet_size(packet), out);

    check(
      tmc3_decoder_push(
        dec, tmc3_packet_data(packet), tmc3_packet_size(packet)),
      "push");
    check(tmc3_decoder_end_frame(dec), "end frame");
    if (tmc3_decoder_next_frame(dec, &frame) != TMC3_OK) {
      fprintf(stderr, "error: frame %d not returned at once\n", i);
      return 1;
    }
    writePly(pattern, frame);
    tmc3_frame_release(frame);
    numFrames++;

    tmc3_packet_release(packet);
    free(ply.data);
  }

  check(tmc3_decoder_flush(dec), "flush");
  drainFrames(dec, pattern, &numFrames);
  tmc3_encoder_destroy(enc);
  tmc3_decoder_destroy(dec);
  fclose(out);
  printf("%d frames\n", numFrames);
  return 0;
}

/*--------------------------------------------------------------------------*/

int
main(int argc, char** argv)
{
  if (argc >= 8 && !strcmp(argv[1], "encode"))
    return encode(argc, argv);
  if (argc >= 5 && !strcmp(argv[1], "decode"))
    return decode(argc, argv);
  if (argc >= 7 && !strcmp(argv[1], "live"))
    return live(argc, argv);

  fprintf(stderr, "usage: see tools/tmc3_api_test.c\n");
  return 1;
}
