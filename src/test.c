/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2010-2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <stdio.h>
#include <opencv/highgui.h>
#include <sys/time.h> 
#include "libuvc/libuvc.h"

// FFmpeg library
#include <libavutil/imgutils.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>


AVCodec* codec;
AVCodecContext* codec_context;
AVCodecParserContext* parser;
AVFrame* picture;

void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;
  IplImage* cvImg;

#if 0
  static struct timeval tv_old, tv_new;
  printf("callback! length = %u, ptr = %d\n", frame->data_bytes, (int) ptr);
  gettimeofday(&tv_new, NULL);
  int sec = tv_new.tv_sec - tv_old.tv_sec;
  int usec = tv_new.tv_usec - tv_old.tv_usec;
  printf("time: %d [us]\n", sec * 1000000 + usec);
  tv_old = tv_new;
  return 0;
  
  int i;
  for (i=0; i<300; i++) {
    printf("%x ", ((unsigned char*)frame->data)[i]);
  }
  printf("\n");
#endif
   
#if 0
  AVPacket pkt;
  int got_picture = 0;
  int len = 0;
 
  av_init_packet(&pkt);
  pkt.data = frame->data;
  pkt.size = frame->data_bytes;
  pkt.pts = AV_NOPTS_VALUE;
  pkt.dts = AV_NOPTS_VALUE;

  len = avcodec_decode_video2(codec_context, picture, &got_picture, &pkt);
  if(len < 0) {
    fprintf(stderr, "Error while decoding a frame.\n");
  }
 
  fprintf(stderr, "got_picgure: %d\n", got_picture);
  if(got_picture == 0) {
    return;
  }
#endif
  
#if 0
  if (avcodec_send_packet(codec_context, &pkt) != 0) {
    printf("avcodec_send_packet failed\n");
  }
  while (avcodec_receive_frame(codec_context, picture) == 0) {
    printf("receive_frame\n");
  }
#endif

#if 1
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!");
    return;
  }

  // Decode motion jpeg
  //ret = uvc_any2bgr(frame, bgr);
  frame->frame_format = UVC_FRAME_FORMAT_MJPEG;
  ret = uvc_mjpeg2rgb(frame, bgr);
  if (ret) {
    uvc_perror(ret, "uvc_any2bgr");
    uvc_free_frame(bgr);
    return;
  }
#endif

  cvImg = cvCreateImageHeader(
      cvSize(bgr->width, bgr->height),
      IPL_DEPTH_8U,
      3);

  cvSetData(cvImg, bgr->data, bgr->width * 3); 

  cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
  cvShowImage("Test", cvImg);
  cvWaitKey(10);

  cvReleaseImageHeader(&cvImg);

  uvc_free_frame(bgr);
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_error_t res;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;

  // initialize codec
  avcodec_register_all();
   
  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if(!codec) {
    printf("Error: cannot find the h264 codec\n");
    return -1;
  }
  codec_context = avcodec_alloc_context3(codec);

  if(codec->capabilities & CODEC_CAP_TRUNCATED) {
    codec_context->flags |= CODEC_FLAG_TRUNCATED;
  }

  if(avcodec_open2(codec_context, codec, NULL) < 0) {
    fprintf(stderr, "Error: could not open codec.\n");
    return -1;
  }

  parser = av_parser_init(AV_CODEC_ID_H264);  
  if(!parser) {
    fprintf(stderr, "Erorr: cannot create H264 parser.\n");
    return -1;
  }

  picture = av_frame_alloc();

  // UVC settings
  
  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  res = uvc_find_device(
      ctx, &dev,
      0x05ca, 0x2711, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device");
  } else {
    puts("Device found");

    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open");
    } else {
      puts("Device opened");

      uvc_print_diag(devh, stderr);
#if 1
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, UVC_FRAME_FORMAT_H264, 1920, 1080, 29
      );
#else
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, UVC_FRAME_FORMAT_MJPEG, 1280, 720, 14
      );
#endif
      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode");
      } else {
        res = uvc_start_streaming(devh, &ctrl, cb, 12345, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming");
        } else {
          puts("Streaming for 10 seconds...");
          uvc_error_t resAEMODE = uvc_set_ae_mode(devh, 1);
          uvc_perror(resAEMODE, "set_ae_mode");
          int i;
          for (i = 1; i <= 10; i++) {
            /* uvc_error_t resPT = uvc_set_pantilt_abs(devh, i * 20 * 3600, 0); */
            /* uvc_perror(resPT, "set_pt_abs"); */
            uvc_error_t resEXP = uvc_set_exposure_abs(devh, 20 + i * 5);
            uvc_perror(resEXP, "set_exp_abs");
            
            sleep(1);
          }
          sleep(10);
          uvc_stop_streaming(devh);
	  puts("Done streaming.");
        }
      }

      uvc_close(devh);
      puts("Device closed");
    }

    uvc_unref_device(dev);
  }

  uvc_exit(ctx);
  puts("UVC exited");

  return 0;
}

