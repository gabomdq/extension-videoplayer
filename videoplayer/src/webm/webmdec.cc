/*
 *  Copyright (c) 2013 The WebM project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webm/webmdec.h"

#include <stdio.h> // printf

#include <stdint.h>
#include <cstring>
#include <cstdio>

#include "webm/mkvparser/mkvparser.h"
#include "webm/mkvparser/mkvreader.h"

// EBML element IDs used for VP9+alpha BlockAdditional extraction
#define EBML_ID_BLOCK_GROUP      0xA0u
#define EBML_ID_BLOCK            0xA1u
#define EBML_ID_SIMPLE_BLOCK     0xA3u
#define EBML_ID_BLOCK_ADDITIONS  0x75A1u
#define EBML_ID_BLOCK_MORE       0xA6u
#define EBML_ID_BLOCK_ADD_ID     0xEEu
#define EBML_ID_BLOCK_ADDITIONAL 0xA5u
// 4-byte EBML IDs (cluster/segment level) have 0x1 as their top nibble
#define EBML_ID_IS_SEGMENT_LEVEL(id) (((id) >> 28) == 0x1u)

// DEFOLD ADDITION
namespace mkvparser
{
  class MkvBufferReader : public IMkvReader {
  public:
    MkvBufferReader();
    explicit MkvBufferReader(uint8_t* buffer, long long length)
    : m_length(length)
    , m_buffer(buffer) {}

    virtual int Read(long long position, long length, unsigned char* buffer)
    {
      if (m_buffer == NULL)
        return -1;

      if (position < 0)
        return -1;

      if (length < 0)
        return -1;

      if (length == 0)
        return 0;

      if (position >= m_length)
        return -1;

      memcpy(buffer, &m_buffer[position], length);

      return 0;
    }
    virtual int Length(long long* total, long long* available)
    {
      if (m_buffer == NULL)
        return -1;

      if (total)
        *total = m_length;

      if (available)
        *available = m_length;

      return 0;
    }

  private:
    MkvBufferReader(const MkvBufferReader&);
    MkvBufferReader& operator=(const MkvBufferReader&);

    long long m_length;
    uint8_t* m_buffer;
  };
}

namespace {

// Read an EBML element header (ID + data size) from reader at pos.
// Returns 0 on success, -1 on error.
static int ebml_read_header(mkvparser::IMkvReader* reader, long long pos,
                            uint32_t* out_id, uint64_t* out_size, int* out_header_len) {
  unsigned char buf[12] = {0};
  if (reader->Read(pos, 12, buf) != 0) return -1;

  unsigned char b0 = buf[0];
  int id_len;
  uint32_t id;
  if      (b0 & 0x80) { id = b0;                                                                                      id_len = 1; }
  else if (b0 & 0x40) { id = ((uint32_t)b0 << 8)  | buf[1];                                                          id_len = 2; }
  else if (b0 & 0x20) { id = ((uint32_t)b0 << 16) | ((uint32_t)buf[1] << 8) | buf[2];                                id_len = 3; }
  else if (b0 & 0x10) { id = ((uint32_t)b0 << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];    id_len = 4; }
  else return -1;

  unsigned char s0 = buf[id_len];
  int size_len;
  uint64_t sz;
  if      (s0 & 0x80) { sz = s0 & 0x7F;                                                                                             size_len = 1; }
  else if (s0 & 0x40) { sz = ((uint64_t)(s0 & 0x3F) << 8)  | buf[id_len+1];                                                        size_len = 2; }
  else if (s0 & 0x20) { sz = ((uint64_t)(s0 & 0x1F) << 16) | ((uint64_t)buf[id_len+1] << 8) | buf[id_len+2];                       size_len = 3; }
  else if (s0 & 0x10) { sz = ((uint64_t)(s0 & 0x0F) << 24) | ((uint64_t)buf[id_len+1] << 16) | ((uint64_t)buf[id_len+2] << 8) | buf[id_len+3]; size_len = 4; }
  else return -1;

  *out_id = id; *out_size = sz; *out_header_len = id_len + size_len;
  return 0;
}

// Scan EBML elements immediately after the Block payload for BlockAdditions (0x75A1).
// If found, extract the BlockAdditional (VP9 alpha bitstream) into webm_ctx->alpha_buffer.
static void find_alpha_data(mkvparser::IMkvReader* reader,
                            const mkvparser::Block* block,
                            struct WebmInputContext* webm_ctx) {
  webm_ctx->alpha_frame_size = 0;
  long long pos = block->m_start + block->m_size;

  for (int guard = 0; guard < 8; ++guard) {
    uint32_t id; uint64_t elem_size; int hl;
    if (ebml_read_header(reader, pos, &id, &elem_size, &hl) != 0) return;

    if (id == EBML_ID_BLOCK_ADDITIONS) {
      long long ba_pos = pos + hl, ba_end = ba_pos + (long long)elem_size;
      while (ba_pos < ba_end) {
        uint32_t bm_id; uint64_t bm_size; int bm_hl;
        if (ebml_read_header(reader, ba_pos, &bm_id, &bm_size, &bm_hl) != 0) return;
        if (bm_id == EBML_ID_BLOCK_MORE) {
          long long it_pos = ba_pos + bm_hl, it_end = it_pos + (long long)bm_size;
          while (it_pos < it_end) {
            uint32_t it_id; uint64_t it_size; int it_hl;
            if (ebml_read_header(reader, it_pos, &it_id, &it_size, &it_hl) != 0) return;
            if (it_id == EBML_ID_BLOCK_ADDITIONAL) {
              size_t alpha_size = (size_t)it_size;
              if (alpha_size > webm_ctx->alpha_buffer_size) {
                delete[] webm_ctx->alpha_buffer;
                webm_ctx->alpha_buffer      = new uint8_t[alpha_size];
                webm_ctx->alpha_buffer_size = alpha_size;
              }
              if (reader->Read(it_pos + it_hl, (long)alpha_size, webm_ctx->alpha_buffer) == 0)
                webm_ctx->alpha_frame_size = alpha_size;
              return;
            }
            it_pos += it_hl + (long long)it_size;
          }
        }
        ba_pos += bm_hl + (long long)bm_size;
      }
      return;
    }
    // Stop if we've hit another block-group-level or cluster-level element
    if (id == EBML_ID_BLOCK_GROUP || id == EBML_ID_BLOCK ||
        id == EBML_ID_SIMPLE_BLOCK || EBML_ID_IS_SEGMENT_LEVEL(id)) return;
    pos += hl + (long long)elem_size;
  }
}

void reset(struct WebmInputContext *const webm_ctx) {
  if (webm_ctx->reader != NULL) {
    mkvparser::MkvBufferReader *const reader = reinterpret_cast<mkvparser::MkvBufferReader *>(webm_ctx->reader);
    delete reader;
  }
  if (webm_ctx->segment != NULL) {
    mkvparser::Segment *const segment =
        reinterpret_cast<mkvparser::Segment *>(webm_ctx->segment);
    delete segment;
  }
  if (webm_ctx->buffer != NULL) {
    delete[] webm_ctx->buffer;
  }
  if (webm_ctx->alpha_buffer != NULL) {
    delete[] webm_ctx->alpha_buffer;
  }
  webm_ctx->reader = NULL;
  webm_ctx->segment = NULL;
  webm_ctx->buffer = NULL;
  webm_ctx->cluster = NULL;
  webm_ctx->block_entry = NULL;
  webm_ctx->block = NULL;
  webm_ctx->block_frame_index = 0;
  webm_ctx->video_track_index = 0;
  webm_ctx->timestamp_ns = 0;
  webm_ctx->is_key_frame = false;
  webm_ctx->alpha_buffer      = NULL;
  webm_ctx->alpha_buffer_size = 0;
  webm_ctx->alpha_frame_size  = 0;
}

void get_first_cluster(struct WebmInputContext *const webm_ctx) {
  mkvparser::Segment *const segment =
      reinterpret_cast<mkvparser::Segment *>(webm_ctx->segment);
  const mkvparser::Cluster *const cluster = segment->GetFirst();
  webm_ctx->cluster = cluster;
}

void rewind_and_reset(struct WebmInputContext *const webm_ctx,
                      struct VpxInputContext *const vpx_ctx) {
  //rewind(vpx_ctx->file);
  reset(webm_ctx);
}

}  // namespace

int file_is_webm(struct WebmInputContext *webm_ctx,
                 struct VpxInputContext *vpx_ctx) {
  //mkvparser::MkvBufferReader* const reader = new mkvparser::MkvBufferReader(vpx_ctx->buffer, vpx_ctx->length);
  mkvparser::IMkvReader* reader;
  /*if( vpx_ctx->file )
    reader = new mkvparser::MkvReader(vpx_ctx->file);
  else*/
  if(vpx_ctx->buffer)
    reader = new mkvparser::MkvBufferReader(vpx_ctx->buffer, vpx_ctx->length);
  else {
    printf("%s %d: %s: unknown error\n", __FILE__, __LINE__, __FUNCTION__);
    return 0;
  }

  webm_ctx->reader = reader;
  webm_ctx->reached_eos = 0;

  mkvparser::EBMLHeader header;
  long long pos = 0;
  if (header.Parse(reader, pos) < 0) {
    printf("%s %d: %s: Parser error\n", __FILE__, __LINE__, __FUNCTION__);
    rewind_and_reset(webm_ctx, vpx_ctx);
    return 0;
  }

  mkvparser::Segment *segment;
  if (mkvparser::Segment::CreateInstance(reader, pos, segment)) {
    printf("%s %d: %s: Instance error\n", __FILE__, __LINE__, __FUNCTION__);
    rewind_and_reset(webm_ctx, vpx_ctx);
    return 0;
  }
  webm_ctx->segment = segment;
  if (segment->Load() < 0) {
    printf("%s %d: %s: Load error\n", __FILE__, __LINE__, __FUNCTION__);
    rewind_and_reset(webm_ctx, vpx_ctx);
    return 0;
  }

  const mkvparser::Tracks *const tracks = segment->GetTracks();
  const mkvparser::VideoTrack *video_track = NULL;
  for (unsigned long i = 0; i < tracks->GetTracksCount(); ++i) {
    const mkvparser::Track *const track = tracks->GetTrackByIndex(i);
    if (track->GetType() == mkvparser::Track::kVideo) {
      video_track = static_cast<const mkvparser::VideoTrack *>(track);
      webm_ctx->video_track_index = static_cast<int>(track->GetNumber());
      break;
    }
  }

  if (video_track == NULL || video_track->GetCodecId() == NULL) {
    printf("%s %d: %s: Track/Codec error\n", __FILE__, __LINE__, __FUNCTION__);
    rewind_and_reset(webm_ctx, vpx_ctx);
    return 0;
  }

  if (!strncmp(video_track->GetCodecId(), "V_VP8", 5)) {
    vpx_ctx->fourcc = VP8_FOURCC;
  } else if (!strncmp(video_track->GetCodecId(), "V_VP9", 5)) {
    vpx_ctx->fourcc = VP9_FOURCC;
  } else {
    printf("%s %d: %s: Bad fourcc\n", __FILE__, __LINE__, __FUNCTION__);
    rewind_and_reset(webm_ctx, vpx_ctx);
    return 0;
  }

  vpx_ctx->framerate.denominator = 0;
  vpx_ctx->framerate.numerator = 0;
  vpx_ctx->width = static_cast<uint32_t>(video_track->GetWidth());
  vpx_ctx->height = static_cast<uint32_t>(video_track->GetHeight());

  get_first_cluster(webm_ctx);

  return 1;
}

int webm_read_frame(struct WebmInputContext *webm_ctx, uint8_t **buffer,
                    size_t *buffer_size) {
  // This check is needed for frame parallel decoding, in which case this
  // function could be called even after it has reached end of input stream.
  if (webm_ctx->reached_eos) {
    return 1;
  }
  mkvparser::Segment *const segment =
      reinterpret_cast<mkvparser::Segment *>(webm_ctx->segment);
  const mkvparser::Cluster *cluster =
      reinterpret_cast<const mkvparser::Cluster *>(webm_ctx->cluster);
  const mkvparser::Block *block =
      reinterpret_cast<const mkvparser::Block *>(webm_ctx->block);
  const mkvparser::BlockEntry *block_entry =
      reinterpret_cast<const mkvparser::BlockEntry *>(webm_ctx->block_entry);
  bool block_entry_eos = false;
  do {
    long status = 0;
    bool get_new_block = false;
    if (block_entry == NULL && !block_entry_eos) {
      status = cluster->GetFirst(block_entry);
      get_new_block = true;
    } else if (block_entry_eos || block_entry->EOS()) {
      cluster = segment->GetNext(cluster);
      if (cluster == NULL || cluster->EOS()) {
        *buffer_size = 0;
        webm_ctx->reached_eos = 1;
        return 1;
      }
      status = cluster->GetFirst(block_entry);
      block_entry_eos = false;
      get_new_block = true;
    } else if (block == NULL ||
               webm_ctx->block_frame_index == block->GetFrameCount() ||
               block->GetTrackNumber() != webm_ctx->video_track_index) {
      status = cluster->GetNext(block_entry, block_entry);
      if (block_entry == NULL || block_entry->EOS()) {
        block_entry_eos = true;
        continue;
      }
      get_new_block = true;
    }
    if (status || block_entry == NULL) {
      return -1;
    }
    if (get_new_block) {
      block = block_entry->GetBlock();
      webm_ctx->block_frame_index = 0;
    }
  } while (block->GetTrackNumber() != webm_ctx->video_track_index ||
           block_entry_eos);

  webm_ctx->cluster = cluster;
  webm_ctx->block_entry = block_entry;
  webm_ctx->block = block;

  const mkvparser::Block::Frame &frame =
      block->GetFrame(webm_ctx->block_frame_index);
  ++webm_ctx->block_frame_index;
  if (frame.len > static_cast<long>(*buffer_size)) {
    delete[] * buffer;
    *buffer = new uint8_t[frame.len];
    if (*buffer == NULL) {
      return -1;
    }
    webm_ctx->buffer = *buffer;
  }
  *buffer_size = frame.len;
  webm_ctx->timestamp_ns = block->GetTime(cluster);
  webm_ctx->is_key_frame = block->IsKey();

  mkvparser::MkvBufferReader *const reader =
      reinterpret_cast<mkvparser::MkvBufferReader *>(webm_ctx->reader);
  int result = frame.Read(reader, *buffer) ? -1 : 0;
  if (result == 0)
    find_alpha_data(reader, block, webm_ctx);
  else
    webm_ctx->alpha_frame_size = 0;
  return result;
}

int webm_guess_framerate(struct WebmInputContext *webm_ctx,
                         struct VpxInputContext *vpx_ctx) {
  uint32_t i = 0;
  uint8_t *buffer = NULL;
  size_t buffer_size = 0;
  while (webm_ctx->timestamp_ns < 1000000000 && i < 50) {
    if (webm_read_frame(webm_ctx, &buffer, &buffer_size)) {
      break;
    }
    ++i;
  }
  vpx_ctx->framerate.numerator = (i - 1) * 1000000;
  vpx_ctx->framerate.denominator =
      static_cast<int>(webm_ctx->timestamp_ns / 1000);
  delete[] buffer;

  get_first_cluster(webm_ctx);
  webm_ctx->block = NULL;
  webm_ctx->block_entry = NULL;
  webm_ctx->block_frame_index = 0;
  webm_ctx->timestamp_ns = 0;
  webm_ctx->reached_eos = 0;

  return 0;
}

void webm_free(struct WebmInputContext *webm_ctx)
{
  reset(webm_ctx);
}
