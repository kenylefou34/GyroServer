#ifndef DECODER_AVDECODER_HPP
#define DECODER_AVDECODER_HPP

#include <iostream>
#include <stdexcept>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

class AVDecoder {
 public:
  AVDecoder()
  {
    // Find the H.264 codec
    m_codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!m_codec) {
      throw std::runtime_error("Codec 'AV_CODEC_ID_H264' not found!");
    }

    // Allocate codec context
    m_codec_context = avcodec_alloc_context3(m_codec);
    if (!m_codec_context) {
      throw std::runtime_error("Could not allocate codec context!");
    }

    // Open the codec
    if (avcodec_open2(m_codec_context, m_codec, nullptr) < 0) {
      avcodec_free_context(&m_codec_context);
      throw std::runtime_error("Could not open codec!");
    }
  };

  AVDecoder(AVDecoder &&other) = delete;
  AVDecoder(const AVDecoder &other) = delete;

  AVDecoder &operator=(AVDecoder &&other) = delete;
  AVDecoder &operator=(const AVDecoder &other) = delete;

  ~AVDecoder() { cleanup(); }

 public:
  inline bool readSPSandPPS(std::vector<std::uint8_t> &sps, std::vector<std::uint8_t> &pps)
  {
    if (m_is_sps_pps_submitted) {
      return false;
    }

    if (!m_codec_context) {
      return false;
    }

    // Encapsuler SPS dans un AVPacket
    AVPacket *sps_packet = av_packet_alloc();
    sps_packet->data = sps.data();
    sps_packet->size = sps.size();

    // Envoyer le SPS au décodeur
    if (avcodec_send_packet(m_codec_context, sps_packet) < 0) {
      return false;
    }
    av_packet_free(&sps_packet);

    // Encapsuler PPS dans un AVPacket
    AVPacket *pps_packet = av_packet_alloc();
    pps_packet->data = pps.data();
    pps_packet->size = pps.size();

    // Envoyer le PPS au décodeur
    if (avcodec_send_packet(m_codec_context, pps_packet) < 0) {
      return false;
    }
    av_packet_free(&pps_packet);

    m_is_sps_pps_submitted = true;
    return m_is_sps_pps_submitted;
  };

  inline bool receiveFrameBuffer(std::vector<std::uint8_t> &data_buffer)
  {
    // Create an AVPacket and initialize it
    m_packet = av_packet_alloc();
    if (!m_packet) {
      std::cerr << "Could not allocate AVPacket!" << std::endl;
      return false;
    }

    // Set data and size for the packet
    m_packet->data = data_buffer.data();
    m_packet->size = data_buffer.size();

    // Send the packet to the decoder
    if (avcodec_send_packet(m_codec_context, m_packet) < 0) {
      std::cerr << "Error sending packet for decoding\n";
      av_packet_free(&m_packet); // Free packet if send failed
      return false;
    }

    if (m_packet) {
      av_packet_free(&m_packet);
    }

    // Allocate an AVFrame for decoding
    m_frame = av_frame_alloc();
    if (!m_frame) {
      avcodec_free_context(&m_codec_context);
      throw std::runtime_error("Could not allocate frame!");
    }

    // Receive the decoded frame
    if (avcodec_receive_frame(m_codec_context, m_frame) == 0) {
      m_sws_context = sws_getContext(m_frame->width,
                                     m_frame->height,
                                     m_codec_context->pix_fmt,
                                     m_frame->width,
                                     m_frame->height,
                                     AV_PIX_FMT_BGR24,
                                     SWS_BILINEAR,
                                     nullptr,
                                     nullptr,
                                     nullptr);
      return true;
    }
    // Failed to receive
    else {
      if (m_frame) {
        av_frame_free(&m_frame);
      }
      return false;
    }
  };

  inline void scaleFrame(std::vector<std::uint8_t *> dest, std::vector<int> linesize)
  {
    if (m_sws_context && m_frame) {
      sws_scale(m_sws_context,
                m_frame->data,
                m_frame->linesize,
                0,
                m_frame->height,
                &dest[0],
                &linesize[0]);
      sws_freeContext(m_sws_context);
    }
  }

  inline void cleanup()
  {
    if (m_packet) {
      av_packet_free(&m_packet);
    }
    if (m_frame) {
      av_frame_free(&m_frame);
    }
    if (m_codec_context) {
      avcodec_free_context(&m_codec_context);
    }
  };

  // GETTERS & SETTERS
 protected:
  inline AVCodec *codec() { return m_codec; };
  inline AVCodecContext *codecContext() { return m_codec_context; };

 public:
  inline AVFrame *frame() { return m_frame; };

 private:
  /// Find the H.264 codec
  AVCodec *m_codec = nullptr;
  /// Codec context
  AVCodecContext *m_codec_context = nullptr;
  /// Frame to allocate
  AVFrame *m_frame = nullptr;
  /// SwS context
  SwsContext *m_sws_context = nullptr;
  /// The packet received
  AVPacket *m_packet = nullptr;
  /// SPS & PPS submitted
  bool m_is_sps_pps_submitted{false};
};

#endif // DECODER_AVDECODER_HPP
